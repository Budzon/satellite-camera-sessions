using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using Astronomy;
using DBTables;
using Common;
using SessionsPlanning;

namespace SatelliteSessions
{
    /// <summary>
    /// сеанс связи
    /// </summary>
    public class CommunicationSession
    {         
        /// <summary>
        ///  тип сессии
        /// </summary>       
        public SessionsPlanning.CommunicationSessionStation Station { get; private set; }
        public SessionsPlanning.WorkingType tg = WorkingType.Formatting;
        /// <summary>
        ///  Время начала 7-градусной зоны
        /// </summary>
        public DateTime Zone5timeFrom { get; private set; }
        /// <summary>
        /// Время конца 7-градусной зоны
        /// </summary>
        public DateTime Zone5timeTo { get; private set; }
        /// <summary>
        /// Время начала 5-градусной зоны
        /// </summary>
        public DateTime Zone7timeFrom { get; private set; }
        /// <summary>
        /// Время конца 5-градусной зоны
        /// </summary>
        public DateTime Zone7timeTo { get; private set; }
        
        public TimePeriod DropInterval { get { return new TimePeriod(Zone7timeFrom, Zone7timeTo); } }         

        public CommunicationSession(
            SessionsPlanning.CommunicationSessionStation station,
            DateTime dtFrom5zone,
            DateTime dtTo5zone,
            DateTime dtFrom7zone,
            DateTime dtTo7zone
            )
        {
            Station = station;
            Zone5timeFrom = dtFrom5zone;
            Zone5timeTo = dtTo5zone;
            Zone7timeFrom = dtFrom7zone;
            Zone7timeTo = dtTo7zone;
        }

        private static DateTime getIntersectionTime(TrajectoryPoint first_point, TrajectoryPoint second_point, Vector3D centre, double zoneR)
        {
            double angle_zone = Math.Asin(zoneR / (OptimalChain.Constants.orbit_height + Astronomy.Constants.EarthRadius));

            double first_angle = AstronomyMath.ToRad(Vector3D.AngleBetween(first_point.Position.ToVector(), centre));
            double second_angle = AstronomyMath.ToRad(Vector3D.AngleBetween(second_point.Position.ToVector(), centre));

            double fullSpan = (second_point.Time - first_point.Time).TotalMilliseconds;

            double deltaTime = fullSpan * Math.Abs(angle_zone - first_angle) / Math.Abs(second_angle - first_angle);
            DateTime resTime = first_point.Time.AddMilliseconds(deltaTime);

            return resTime;
        }

        public static void getSessionFromZone(
            CommunicationZone zone,
            Trajectory trajectory,
            Dictionary<CommunicationSessionStation, List<CommunicationSession>> sessions,
            List<SessionsPlanning.CommunicationSessionStation> enabledStations)
        {
            if (enabledStations.Intersect(zone.Stations).Count() == 0)
                return;
            Vector3D centre = GeoPoint.ToCartesian(new GeoPoint(zone.CentreLat, zone.CentreLon), 1);

            bool prevIn5zone = false;
            bool prevIn7zone = false;
            bool is7zoneSet = false;
             
            int count = trajectory.Count;
            var points = trajectory.Points;

            DateTime Zone5timeFrom, Zone5timeTo, Zone7timeFrom, Zone7timeTo; 
            Zone5timeFrom = Zone5timeTo = Zone7timeFrom = Zone7timeTo = new DateTime();

            for (int i = 0; i < count; i++)
            { 
                bool in5zone = zone.isPointInZone(points[i].Position, zone.Radius5);
                bool in7zone = zone.isPointInZone(points[i].Position, zone.Radius7);

                if (in5zone && !prevIn5zone) // текущая точка в 5ти гр. зоне, причем предыдущая не в пятиградусной зоне (или текущая точка - первая (i==0)  )
                {
                    if (i == 0)
                        Zone5timeFrom = points[i].Time;
                    else
                        Zone5timeFrom = getIntersectionTime(points[i - 1], points[i], centre, zone.Radius5);
                }

                if (in7zone && !prevIn7zone) // текущая точка в 7ти гр. зоне, причем предыдущая не в 7ти гр. зоне )
                {
                    if (i == 0)
                        Zone7timeFrom = points[i].Time;
                    else
                        Zone7timeFrom = getIntersectionTime(points[i - 1], points[i], centre, zone.Radius7);
                    is7zoneSet = true;
                }

                if (!in7zone && prevIn7zone) // вышли из семиградусной зоны (текущая точка не в семиградусной зоне, а предыдущая в семиградусной)
                {
                    Zone7timeTo = getIntersectionTime(points[i - 1], points[i], centre, zone.Radius7);
                }

                if ((!in5zone || i == count) && prevIn5zone) // предыдущая точка в пятиградусной зоне, а текущая точка последняя или находится вне пятиградусной зоны
                {
                    Zone5timeTo = getIntersectionTime(points[i - 1], points[i], centre, zone.Radius5);
                    if (is7zoneSet)  // добавляем только если удалось установить и 7 зону тоже      
                    {
                        foreach (var station in zone.Stations)
                        {
                            if (!enabledStations.Contains(station))
                                continue;
                           CommunicationSession session = new CommunicationSession(
                                station,
                                Zone5timeFrom,
                                Zone5timeTo,
                                Zone7timeFrom,
                                Zone7timeTo);
                           if (!sessions.ContainsKey(station))
                                sessions[station] = new List<CommunicationSession>();
                           sessions[station].Add(session);
                        }
                    }
                                
                    is7zoneSet = false;
                }

                prevIn5zone = in5zone;
                prevIn7zone = in7zone;
            }
        }


        public static void getAllSNKPOICommunicationSessions(
            DateTime timeFrom,
            DateTime timeTo,
            DIOS.Common.SqlManager managerDB,
            Dictionary<CommunicationSessionStation, List<CommunicationSession>> sessions,
            List<SessionsPlanning.CommunicationSessionStation> enabledStations)
        {            
            DataFetcher fetcher = new DataFetcher(managerDB);
            CommunicationZoneSNKPOI sZone;            
            CommunicationZone.getSNKPOICommunicationZones(managerDB, out sZone);
            Trajectory fullTrajectory = fetcher.GetTrajectorySat(timeFrom, timeTo);
            getSessionFromZone(sZone, fullTrajectory, sessions, enabledStations);            
        }

        public static void getAllMNKPOICommunicationSessions(
            DateTime timeFrom,
            DateTime timeTo,
            DIOS.Common.SqlManager managerDB,
            Dictionary<CommunicationSessionStation, List<CommunicationSession>> sessions,
            List<SessionsPlanning.CommunicationSessionStation> enabledStations)
        { 
            DataFetcher fetcher = new DataFetcher(managerDB);
            List<CommunicationZoneMNKPOI> mZones;
            CommunicationZone.getMNKPOICommunicationZones(managerDB, timeFrom, timeTo, out mZones);

            if (mZones == null)
                return;
            if (mZones.Count == 0)
                return;

            Trajectory fullTrajectory = fetcher.GetTrajectorySat(timeFrom, timeTo);

            foreach (var zone in mZones)
            {                
                Trajectory trajectory;
                if (zone.From > timeFrom || zone.To < timeTo) // если временной промежуток зоны более строгий, чем  timeFrom - timeTo
                    trajectory = fetcher.GetTrajectorySat(zone.From > timeFrom ? zone.From : timeFrom,
                                                          zone.To < timeTo ? zone.To : timeTo); // то загружаем траекторию с заданными временными промежутками
                else
                    trajectory = fullTrajectory;

                getSessionFromZone(zone, trajectory, sessions, enabledStations);
            }             
        }

        /// <summary>
        /// Расчет сеансов связи за заданный период времени
        /// </summary>
        /// <param name="timeFrom">Начало временного отрезка</param>
        /// <param name="timeTo">Конец временного отрезка</param>
        /// <returns>Все возможные сеансы связи за это время, сгруппированные по антеннам</returns>
        public static Dictionary<SessionsPlanning.CommunicationSessionStation, List<CommunicationSession>> createCommunicationSessions(
            DateTime timeFrom,
            DateTime timeTo,
            string connectStr,
            List<SessionsPlanning.CommunicationSessionStation> enabledStations)
        {
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(connectStr);
            var sessions = new Dictionary<CommunicationSessionStation, List<CommunicationSession>>();
            getAllSNKPOICommunicationSessions(timeFrom, timeTo, managerDB, sessions, enabledStations);
            getAllMNKPOICommunicationSessions(timeFrom, timeTo, managerDB, sessions, enabledStations);           
            return sessions;
        } 


        /// <summary>
        /// получить свободные промежутки времени из сессий связи
        /// </summary>
        /// <param name="sessions"> все сессии связи </param>
        /// <param name="occupiedPeriods"> промежутки времени, которые следует исключить из сессий связи </param>
        /// <returns> свободные промежутки времени </returns>
        public static List<TimePeriod> getFreeTimePeriodsOfSessions(List<CommunicationSession> sessions, List<TimePeriod> occupiedPeriods)
        {
            List<TimePeriod> freePeriods = sessions.Select(sess => sess.DropInterval).ToList();
            return getFreeTimePeriodsOfSessions(freePeriods, occupiedPeriods);  
        }

        public static List<TimePeriod> getFreeTimePeriodsOfSessions(List<TimePeriod> freePeriods, List<TimePeriod> occupiedPeriods)
        {
            List<TimePeriod> freeRangesForDrop = new List<TimePeriod>();
            var compressedOccupiedPeriods = TimePeriod.compressTimePeriods(occupiedPeriods);
            foreach (var timeSpan in freePeriods)
            {
                //List<TimePeriod> freeRangesForSession = getFreeTimeRanges(timeSpan, occupiedPeriods);
                List<TimePeriod> freeRangesForSession = TimePeriod.getFreeIntervals(compressedOccupiedPeriods, timeSpan.dateFrom, timeSpan.dateTo);

                freeRangesForDrop.AddRange(freeRangesForSession);
            }
            freeRangesForDrop = TimePeriod.compressTimePeriods(freeRangesForDrop);
            freeRangesForDrop = freeRangesForDrop.OrderByDescending(range => (range.dateTo - range.dateFrom).TotalSeconds).ToList();
            return freeRangesForDrop;
        }

    }
}
