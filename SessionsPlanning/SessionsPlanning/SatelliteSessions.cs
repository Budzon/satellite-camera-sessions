using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

using DataParsers;
using SatelliteTrajectory;
using Astronomy;
using Common;
using OptimalChain;

using DBTables;

using SphericalGeom;


namespace SatelliteSessions
{
    /// <summary>
    /// сеанс связи
    /// </summary>
    public class CommunicationSession
    {
        /// <summary>
        ///  Id антенны
        /// </summary>
        public int id { get; set; }
        /// <summary>
        ///  Время начала 7-градусной зоныc
        /// </summary>
        public DateTime Zone5timeFrom { get; set; }
        /// <summary>
        /// Время конца 7-градусной зоны
        /// </summary>
        public DateTime Zone5timeTo { get; set; }
        /// <summary>
        /// Время начала 5-градусной зоны
        /// </summary>
        public DateTime Zone7timeFrom { get; set; }
        /// <summary>
        /// Время конца 5-градусной зоны
        /// </summary>
        public DateTime Zone7timeTo { get; set; }
        /// <summary>
        /// Список маршрутов на сброс, участвующих в этом сеансе 
        /// </summary>
        public List<RouteMPZ> routesToReset { get; set; }
    }
     
    public class Sessions
    {
        /// <summary>
        /// возвращает реализуемость заказа
        /// </summary>
        /// <param name="request">Заказ</param>
        /// <param name="timeFrom">начало диапазона времени</param>
        /// <param name="timeTo">конец диапазона времени</param>
        /// <param name="coverage">Процент покрытия, которые можно получить.</param>
        /// <param name="possibleConfs">Список конфигураций, когда возможна съемка (хотя бы кусочка)</param>
        public static void isRequestFeasible(RequestParams request, DateTime timeFrom, DateTime timeTo, out double coverage, out List<CaptureConf> possibleConfs)
        {
            string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "trajectory_1day.dat";
            Astronomy.Trajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, timeFrom, timeTo); // @todo временно            
            double viewAngle = request.Max_SOEN_anlge + OptimalChain.Constants.camera_angle; 
            SatLane viewLane = new SatLane(trajectory, 0, viewAngle);
            List<CaptureConf> confs = viewLane.getCaptureConfs(request);

            double summ = 0;
            List<SphericalGeom.Polygon> region = new List<SphericalGeom.Polygon> { new SphericalGeom.Polygon(request.wktPolygon) };
            foreach (var conf in confs)
            {
                foreach (var order in conf.orders)
                {
                    var notCoveredBefore = new List<SphericalGeom.Polygon>();
                    var toBeCoveredAfter = new List<SphericalGeom.Polygon>();
                    for (int i = 0; i < region.Count; ++i)
                    {
                        var intAndSub = SphericalGeom.Polygon.IntersectAndSubtract(region[i], order.captured);
                        notCoveredBefore.AddRange(intAndSub.Item1);
                        toBeCoveredAfter.AddRange(intAndSub.Item2);
                    }
                    double areaNotCoveredBefore = 0;
                    for (int j = 0; j < notCoveredBefore.Count; ++j)
                    {
                        areaNotCoveredBefore += notCoveredBefore[j].Area;
                    }
                    summ += order.intersection_coeff * areaNotCoveredBefore / order.captured.Area;
                    notCoveredBefore.Clear();
                    region.Clear();
                    region = toBeCoveredAfter;
                }
            }
            coverage = summ;
            possibleConfs =  new List<CaptureConf>(); /// @todo заполнять список конфигураций            
        }

        public static List<CaptureConf> getCaptureConfArray(IList<RequestParams> requests, DateTime timeFrom, DateTime timeTo)
        {
            if (requests.Count == 0)
                throw new ArgumentException("Requests array is empty!");

            List<CaptureConf> captureConfs = new List<CaptureConf>();

            string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "\\" + "trajectory_1day.dat";
            Astronomy.Trajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, timeFrom, timeTo); // @todo временно

            // @todo вынести это в константы
            double viewAngle = OptimalChain.Constants.camera_angle; // угол обзора камеры
            double angleStep = viewAngle; // шаг равен углу обзора

            double Max_SOEN_anlge = requests[0].Max_SOEN_anlge;
            foreach (var req in requests)
            {
                if (req.Max_SOEN_anlge > Max_SOEN_anlge)
                    Max_SOEN_anlge = req.Max_SOEN_anlge;
            }

            double max_roll_angle = Math.Min(Max_SOEN_anlge, AstronomyMath.ToRad(45));
            double min_roll_angle = Math.Max(-Max_SOEN_anlge, AstronomyMath.ToRad(-45));
        
            int num_steps = (int)((max_roll_angle - min_roll_angle) / angleStep); /// @todo что делать с остатком от деления?
            //for (double rollAngle = min_roll_angle; rollAngle <= max_roll_angle; rollAngle += angleStep)    {        
            Parallel.For(0, num_steps, index =>
            {
                double rollAngle = min_roll_angle + index * angleStep;
                List<CaptureConf> laneCaptureConfs = new List<CaptureConf>(); // участки захвата для текущий линии захвата
                SatLane viewLane = new SatLane(trajectory, rollAngle, viewAngle, polygonStep: 15);
                foreach (var request in requests)
                {
                    if (Math.Abs(rollAngle) > Math.Abs(request.Max_SOEN_anlge))
                        continue;
                    List<CaptureConf> confs = viewLane.getCaptureConfs(request);
                    CaptureConf.compressCConfArray(ref confs);
                    CaptureConf.compressTwoCConfArrays(ref laneCaptureConfs, ref confs);
                    laneCaptureConfs.AddRange(confs);
                }

                foreach (var conf in laneCaptureConfs)
                { 
                    var pol =  viewLane.getSegment(conf.dateFrom, conf.dateTo);
                    TrajectoryPoint pointFrom = trajectory.GetPoint(conf.dateFrom);
                    TrajectoryPoint pointTo = trajectory.GetPoint(conf.dateTo); 

                    conf.wktPolygon = pol.ToWtk();
                    conf.square = pol.Area;
                    conf.rollAngle = rollAngle;

                    calculatePitchArrays(conf, rollAngle, pointFrom);
                }
                captureConfs.AddRange(laneCaptureConfs);
            }
            );

            for (int ci = 0; ci < captureConfs.Count; ci++)
                captureConfs[ci].id = ci;

            return captureConfs;
         }

        /// <summary>
        /// Планирование в автоматическом режиме
        /// </summary>
        /// <param name="requests">Список заказов с "неотснятыми полигонами"</param>
        /// <param name="timeFrom">Время начала промежутка планирования</param>
        /// <param name="timeTo">Время конца промежутка планирования</param>
        /// <param name="silentRanges">Список интервалов времени , в которые нельзя сбрасывать данные в СНКПОИ и/или МНКПОИ</param>
        /// <param name="inactivityRanges">Список интервалов, когда нельзя проводить съемку</param>
        /// <param name="routesToReset">Перечень маршрутов на сброс</param>
        /// <param name="routesToDelete">Перечень маршрутов на удаление</param>
        /// <param name="managerDB">параметры взаимодействия с БД</param>
        /// <param name="mpzArray">Набор МПЗ</param>
        /// <param name="sessions">Сеансы связи</param>
        public static void getMPZArray(
              IList<RequestParams> requests
            , DateTime timeFrom
            , DateTime timeTo
            , List<Tuple<DateTime, DateTime>> silentRanges
            , List<Tuple<DateTime, DateTime>> inactivityRanges
            , List<RouteMPZ> routesToReset
            , List<RouteMPZ> routesToDelete
            , DIOS.Common.SqlManager managerDB
            , out IList<OptimalChain.fakeMPZ> mpzArray
            , out List<CommunicationSession> sessions)
        {
            List<CaptureConf> captureConfs = getCaptureConfArray(requests, timeFrom, timeTo);
            ///@todo реализовать всё, что касается параметров silentRanges, inactivityRanges, routesToReset, routesToDelete, managerDB, sessions
            Graph g = new Graph(captureConfs);
            mpzArray = g.findOptimalChain();
            sessions = new List<CommunicationSession>();
        }

        /// <summary>
        /// Рассчитать полигон съемки/видимости для заданной конфигурации СОЭНc
        /// </summary>
        /// <param name="dateTime"> Момент времени DateTimec</param>
        /// <param name="rollAngle">Крен в радианах double</param>
        /// <param name="pitchAngle"> Тангаж в радианах double</param>
        /// <param name="duration">Продолжительность съемки в милисекундах</param>
        /// <param name="DBManager">Параметры подключения к БД</param>
        /// <returns> полигон в формате WKT</returns>
        public static string getSOENViewPolygon(DateTime dateTime, double rollAngle, double pitchAngle, int duration, DIOS.Common.SqlManager DBManager)
        {
            string wkt_string = "POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))";
            /// @todo реализовать 
            return wkt_string;
        }

        /// <summary>
        /// Проверка ПНб (программа наблюдений) на непротиворечивость
        /// </summary>
        /// <param name="MPZArray">Список МПЗ</param>
        /// <param name="isIncompatible"> Флаг наличия конфликтов (True/False)</param>
        /// <param name="incompatibleRoutes">Список конфликтов (может быть пустым)</param>
        public static void checkCocmpatibility(List<MPZ> MPZArray, out bool isIncompatible, out List<Tuple<RouteMPZ, RouteMPZ>> incompatibleRoutes)
        {
            isIncompatible = true; 
            incompatibleRoutes = new List<Tuple<RouteMPZ, RouteMPZ>>(); 
            /// @todo реализовать 
        }

        /// <summary>
        /// Нахождение в ПНб противоречий с заданным маршрутом
        /// </summary>
        /// <param name="MPZArray">Список МПЗ</param>
        /// <param name="route">Маршрут, который надо проверить на совместимость с этим МПЗ</param>
        /// <param name="isIncompatible">Флаг наличия конфликтов (True/False)</param>
        /// <param name="incompatibleRoutes">. Список маршрутов, с которым конфликтует заданны маршрут: c</param>
        public static void checkCocmpatibility(List<MPZ> MPZArray, RouteMPZ route, out bool isIncompatible, out List<RouteMPZ> incompatibleRoutes)
        {
            isIncompatible = true;
            incompatibleRoutes = new List<RouteMPZ>();
        }
        
        /// <summary>
        /// Разбиение полосы видимости КА под траекторией на полигоны освещенности.
        /// </summary>
        /// <param name="DBManager">Параметры подключения к БД</param>
        /// <param name="timeFrom">Начало временного промежутка</param>
        /// <param name="timeTo">Конец временного промежутка</param>
        /// <param name="turnNumber">Глобальный номер витка</param>
        /// <param name="partsLitAndNot">Список полигонов, помеченных флагом освещенности</param>
        public static void checkIfViewLaneIsLit(DIOS.Common.SqlManager DBManager, DateTime timeFrom, DateTime timeTo, out int turnNumber, out List<wktPolygonLit> partsLitAndNot)
        {
            turnNumber = OrbitTable.GetNumTurn(DBManager.GetSqlObject(OrbitTable.Name, "").Select()[0]);

            DataFetcher fetcher = new DataFetcher(DBManager);
            List<LanePos> lane = fetcher.GetViewLane(timeFrom, timeTo);
            List<SpaceTime> sunPositions = fetcher.GetPositionSun();

            int sunPositionsCount = sunPositions.Count();
            int curSunPositionIndex = 0;

            partsLitAndNot = new List<wktPolygonLit>();

            bool onLitStreak = false;
            int streakBegin = -1;

            for (int i = 0; i < lane.Count - 1; ++i)
            {
                while ((curSunPositionIndex < sunPositionsCount - 1) && sunPositions[curSunPositionIndex].Time < lane[i].Time)
                    curSunPositionIndex++;

                // can ignore scaling here as the distances are enormous both in kms and in units of Earth radius
                Vector3D sun = sunPositions[curSunPositionIndex].Position;
                SphericalGeom.Polygon sector = SatelliteTrajectory.TrajectoryRoutines.FormSectorFromLanePoints(lane, i, i + 1);

                var LitAndNot = SphericalGeom.Polygon.IntersectAndSubtract(sector, SphericalGeom.Polygon.Hemisphere(sun));
                bool allLit = LitAndNot.Item2.Count == 0;
                bool allUnlit = LitAndNot.Item1.Count == 0;

                if (streakBegin != -1)
                {
                    // On streak -- either continue one or make a master-sector.
                    if ((allLit && onLitStreak) || (allUnlit && !onLitStreak))
                    {
                        continue;
                    }
                    else
                    {
                        partsLitAndNot.Add(new wktPolygonLit
                        {
                            wktPolygon = SatelliteTrajectory.TrajectoryRoutines.FormSectorFromLanePoints(lane, streakBegin, i).ToWtk(),
                            sun = onLitStreak
                        });
                        streakBegin = -1;
                    }
                }

                // Not on streak here -- either start one or just add to output lists.
                if (allLit)
                {
                    onLitStreak = true;
                    streakBegin = i;
                }
                else if (allUnlit) // totally unlit
                {
                    onLitStreak = false;
                    streakBegin = i;
                }
                else
                {
                    foreach (SphericalGeom.Polygon p in LitAndNot.Item1)
                        partsLitAndNot.Add(new wktPolygonLit { wktPolygon = p.ToWtk(), sun = true });
                    foreach (SphericalGeom.Polygon p in LitAndNot.Item2)
                        partsLitAndNot.Add(new wktPolygonLit { wktPolygon = p.ToWtk(), sun = false });
                }
            }
        }

        /// @todo перенести в мат библиотеку
        private static void calculatePitchArrays(CaptureConf conf, double rollAngle, TrajectoryPoint pointFrom)
        {
            double maxAngle = conf.orders[0].request.Max_SOEN_anlge;
            foreach (var req in conf.orders)
            {
                if (req.request.Max_SOEN_anlge < maxAngle)
                    maxAngle = req.request.Max_SOEN_anlge;
            }

            double maxPitchAngle = Math.Abs(maxAngle) - Math.Abs(rollAngle);
            if (0 == maxPitchAngle)
            {
                conf.timeDelta = 0;
            }
            else
            {
                Vector3D rollPoint = LanePos.getSurfacePoint(pointFrom, rollAngle, 0);
                Vector3D PitchRollPoint = LanePos.getSurfacePoint(pointFrom, rollAngle, maxPitchAngle);
                rollPoint.Normalize();
                PitchRollPoint.Normalize();
                // расстояние в километрах между точкой c нулевым тангажом и точкой, полученной при максимальном угле тангажа
                double dist = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(rollPoint), GeoPoint.FromCartesian(PitchRollPoint)) * Astronomy.Constants.EarthRadius;
                // время, за которое спутник преодалевает dist по поверхности земли.
                conf.timeDelta = dist / pointFrom.Velocity.Length;
            }
            int pitchStep = OptimalChain.Constants.pitchStep;
            conf.pitchArray[0] = 0;

            Vector3D dirRollPoint = LanePos.getSurfacePoint(pointFrom, rollAngle, 0);
            for (int pitch_degr = pitchStep; pitch_degr <= AstronomyMath.ToDegrees(maxPitchAngle); pitch_degr += pitchStep)
            {
                double pitch = AstronomyMath.ToRad(pitch_degr);
                Vector3D dirPitchPoint = LanePos.getSurfacePoint(pointFrom, rollAngle, pitch);
                double distOverSurf = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(dirPitchPoint), GeoPoint.FromCartesian(dirRollPoint)) * Astronomy.Constants.EarthRadius;
                double t = distOverSurf / pointFrom.Velocity.Length;
                conf.pitchArray[pitch] = t;
            }
        }               
    } 
     

    public class SessionServices
    {
        public static SatelliteSessions.MPZ getdfsdf()
        {
            return new SatelliteSessions.MPZ(new List<RouteMPZ>());
        }
        public static List<MPZ> MakePlans(
            DateTime dateBegin,
            DateTime dateEnd,
            IList<RequestParams> requests,
            IList<TimeInterval> dumpProhibited,
            IList<TimeInterval> filmProhibited,
            IList<RouteMPZ> dumpRoutes,
            IList<RouteMPZ> deleteRoutes,
            DIOS.Common.SqlManager dbManager)
        {
            List<MPZ> bolvanka = new List<MPZ>
            {
                new MPZ(new List<RouteMPZ> {new RouteMPZ(RegimeTypes.SI),
                                            new RouteMPZ(RegimeTypes.ZI),
                                            new RouteMPZ(RegimeTypes.VI),
                                            new RouteMPZ(RegimeTypes.NP)} ),
                new MPZ(new List<RouteMPZ> {new RouteMPZ(RegimeTypes.ZI_cal),
                                            new RouteMPZ(RegimeTypes.ZI_fok_yust),
                                            new RouteMPZ(RegimeTypes.NP_fok_yust)}),
                new MPZ(new List<RouteMPZ> {new RouteMPZ(RegimeTypes.KPI_load),
                                            new RouteMPZ(RegimeTypes.KPI_unload),
                                            new RouteMPZ(RegimeTypes.PUF_control),
                                            new RouteMPZ(RegimeTypes.BBZU_control),
                                            new RouteMPZ(RegimeTypes.Special)})
            };
            return bolvanka;
        }
    }

    public class wktPolygonLit
    {
        public string wktPolygon {get;set;}
        public bool sun {get;set;}
    }

    public struct TimeInterval
    {
        public DateTime From {get;set;}
        public DateTime To {get;set;}
    }
 

 
}
