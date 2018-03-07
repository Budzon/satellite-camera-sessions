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
using Microsoft.SqlServer.Types;
using System.Data.SqlTypes;
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
        public int antennaId { get; set; }
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
        /// <param name="managerDB">бд</param>
        /// <param name="coverage">Процент покрытия, которые можно получить.</param>
        /// <param name="possibleConfs">Список конфигураций, когда возможна съемка (хотя бы кусочка)</param>
        public static void isRequestFeasible(RequestParams request, DateTime timeFrom, DateTime timeTo,  DIOS.Common.SqlManager managerDB, out double coverage, out List<CaptureConf> possibleConfs)
        {
            // string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "trajectory_1day.dat";     
            // Astronomy.Trajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, timeFrom, timeTo); // @todo временно      
            DataFetcher fetcher = new DataFetcher(managerDB);    
            Trajectory trajectory = fetcher.GetTrajectorySat(timeFrom, timeTo);

            double viewAngle = request.Max_SOEN_anlge + OptimalChain.Constants.camera_angle;     
            SatLane viewLane = new SatLane(trajectory, 0, 0, viewAngle);       
            possibleConfs = viewLane.getCaptureConfs(request);      
            double summ = 0;
            
            /*
            foreach (var conf in possibleConfs)
            {
                foreach (var order in conf.orders)
                {
                    summ += order.intersection_coeff;  
                }
            }
            if (summ > 1)
                summ = 1;
            */
                      
            List<SphericalGeom.Polygon> region = new List<SphericalGeom.Polygon> { new SphericalGeom.Polygon(request.wktPolygon) };
            foreach (var conf in possibleConfs)
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
                SatLane viewLane = new SatLane(trajectory, rollAngle, 0, viewAngle, polygonStep: 15);
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
              List<RequestParams> requests
            , DateTime timeFrom
            , DateTime timeTo
            , List<Tuple<DateTime, DateTime>> silentRanges
            , List<Tuple<DateTime, DateTime>> inactivityRanges
            , List<RouteMPZ> routesToReset
            , List<RouteMPZ> routesToDelete
            , DIOS.Common.SqlManager managerDB
            , out List<MPZ> mpzArray
            , out List<CommunicationSession> sessions)
        {
            List<CaptureConf> captureConfs = getCaptureConfArray(requests, timeFrom, timeTo);
            ///@todo реализовать всё, что касается параметров silentRanges, inactivityRanges, routesToReset, routesToDelete, managerDB, sessions
            Graph g = new Graph(captureConfs);
            List<OptimalChain.MPZParams> mpz_params = g.findOptimalChain();
            mpzArray = new List<MPZ>();
            foreach (var mpz_param in mpz_params)
            {
                mpzArray.Add(new MPZ(mpz_param));
            }
            // mpz_params[0].routes[0].ShootingConf

            { // тестовые данные для sessions
                sessions = new List<CommunicationSession>();
                double duration = (timeTo - timeFrom).TotalMinutes;
                {
                    CommunicationSession comSes = new CommunicationSession();
                    comSes.antennaId = 3;
                    comSes.Zone5timeFrom = timeFrom.AddMinutes(duration / 10);
                    comSes.Zone5timeTo = timeTo.AddMinutes(-duration / 10);
                    comSes.Zone7timeFrom = timeFrom.AddMinutes(duration / 8);
                    comSes.Zone7timeTo = timeTo.AddMinutes(-duration / 8);

                    comSes.routesToReset = new List<RouteMPZ>();
                    comSes.routesToReset.Add(new RouteMPZ(RegimeTypes.SI));
                    comSes.routesToReset.Add(new RouteMPZ(RegimeTypes.NP));
                    sessions.Add(comSes);
                }
                {
                    CommunicationSession comSes = new CommunicationSession();
                    comSes.antennaId = 2;
                    comSes.Zone5timeFrom = timeFrom.AddMinutes(duration / 9);
                    comSes.Zone5timeTo = timeTo.AddMinutes(-duration / 9);
                    comSes.Zone7timeFrom = timeFrom.AddMinutes(duration / 7);
                    comSes.Zone7timeTo = timeTo.AddMinutes(-duration / 7);

                    comSes.routesToReset = new List<RouteMPZ>();
                    comSes.routesToReset.Add(new RouteMPZ(RegimeTypes.VI));
                    comSes.routesToReset.Add(new RouteMPZ(RegimeTypes.ZI));
                    sessions.Add(comSes);
                }
            }
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
        public static string getSOENViewPolygon(DateTime dateTime, double rollAngle, double pitchAngle, int duration, DIOS.Common.SqlManager managerDB)
        {
            string wtk = "";
            if (duration == 0)
            {
                DBTables.DataFetcher fetcher = new DBTables.DataFetcher(managerDB);
                TrajectoryPoint? point = fetcher.GetPositionSat(dateTime);
                // @todo обработать ситуацию, когда point == null (например когда траектории в бд нет)
                if (point == null)
                {
                    return wtk;
                }
                Vector3D dirVector = LanePos.getDirectionVector((TrajectoryPoint)point, rollAngle, pitchAngle);
                Polygon viewPol = Routines.getViewPolygon((TrajectoryPoint)point, dirVector, OptimalChain.Constants.camera_angle);
                wtk = viewPol.ToWtk();             
            }
            else
            {
                DateTime timeTo = dateTime.AddMilliseconds(duration);
                DataFetcher fetcher = new DataFetcher(managerDB);
                Trajectory trajectory = fetcher.GetTrajectorySat(dateTime, timeTo);
                SatLane viewLane = new SatLane(trajectory, rollAngle, pitchAngle,  OptimalChain.Constants.camera_angle);

                if (viewLane.Sectors.Count > 0)
                {
                    List<Vector3D> leftLanePoints = new List<Vector3D>();
                    List<Vector3D> rightLanePoints = new List<Vector3D>();

                    for (int sectId = 0; sectId < viewLane.Sectors.Count; sectId++)
                    {
                        var sect = viewLane.Sectors[sectId];
                        int i = 0;
                        if (sectId > 0)
                            i = 1;
                        for (; i < sect.sectorPoints.Count; i++)
                        {
                            var pos = sect.sectorPoints[i];
                            leftLanePoints.Add(pos.LeftCartPoint);
                            rightLanePoints.Add(pos.RightCartPoint);
                        }
                    }
                    for (int i = rightLanePoints.Count - 1; i >= 0; i--)
                        leftLanePoints.Add(rightLanePoints[i]);

                    Polygon pol = new Polygon(leftLanePoints);
                    wtk = pol.ToWtk();
                }
            }
            return wtk;
        }

        /// <summary>
        /// Проверка ПНб (программа наблюдений) на непротиворечивость
        /// </summary>
        /// <param name="MPZArray">Список МПЗ</param>
        /// <param name="isIncompatible"> Флаг наличия конфликтов (True/False)</param>
        /// <param name="incompatibleRoutes">Список конфликтов (может быть пустым)</param>
        public static void checkCompatibility(List<MPZ> MPZArray, out bool isIncompatible, out List<Tuple<RouteMPZ, RouteMPZ>> incompatibleRoutes)
        {
            List<RouteMPZ> routes = new List<RouteMPZ>();
            incompatibleRoutes = new List<Tuple<RouteMPZ, RouteMPZ>>();
            foreach (var mpz in MPZArray)
            {
                routes.AddRange(mpz.Routes);
            }

            if (routes.Count < 2)
            {
                isIncompatible = true;                
            }
            else 
            {
                isIncompatible = false;
                incompatibleRoutes.Add(new Tuple<RouteMPZ, RouteMPZ>(routes[0], routes[1]));
            }
            /// @todo реализовать
        }

        /// <summary>
        /// Нахождение в ПНб противоречий с заданным маршрутом
        /// </summary>
        /// <param name="MPZArray">Список МПЗ</param>
        /// <param name="route">Маршрут, который надо проверить на совместимость с этим МПЗ</param>
        /// <param name="isIncompatible">Флаг наличия конфликтов (True/False)</param>
        /// <param name="incompatibleRoutes">. Список маршрутов, с которым конфликтует заданны маршрут: c</param>
        public static void checkCompatibility(List<MPZ> MPZArray, RouteMPZ route, out bool isIncompatible, out List<RouteMPZ> incompatibleRoutes)
        {
            List<RouteMPZ> routes = new List<RouteMPZ>();
            incompatibleRoutes = new List<RouteMPZ>();
            foreach (var mpz in MPZArray)
            {
                routes.AddRange(mpz.Routes);
            }

            if (routes.Count == 0)
            {
                isIncompatible = true;
            }
            else
            {
                isIncompatible = false;
                incompatibleRoutes.Add(routes[0]);
            }
            /// @todo реализовать
        }
        
        /// <summary>
        /// Разбиение полосы видимости КА под траекторией на полигоны освещенности.
        /// </summary>
        /// <param name="DBManager">Параметры подключения к БД</param>
        /// <param name="timeFrom">Начало временного промежутка</param>
        /// <param name="timeTo">Конец временного промежутка</param>
        /// <param name="partsLitAndNot">Список объектов: номер витка и полигоны, помеченные флагом освещенности</param>
        public static void checkIfViewLaneIsLit(DIOS.Common.SqlManager DBManager, DateTime timeFrom, DateTime timeTo, out List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot)
        {
            DataFetcher fetcher = new DataFetcher(DBManager);
            var laneParts = fetcher.GetViewLaneBrokenIntoTurns(timeFrom, timeTo);
            List<SpaceTime> sunPositions = fetcher.GetPositionSun(timeFrom, timeTo);
            partsLitAndNot = new List<Tuple<int, List<wktPolygonLit>>>();

            int sunPositionsCount = sunPositions.Count();
            int curSunPositionIndex = 0;

            foreach (var lanePart in laneParts)
            {
                var lane = lanePart.Item2;
                List<wktPolygonLit> turnPartsLitAndNot = new List<wktPolygonLit>();

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
                            turnPartsLitAndNot.Add(new wktPolygonLit
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
                            turnPartsLitAndNot.Add(new wktPolygonLit { wktPolygon = p.ToWtk(), sun = true });
                        foreach (SphericalGeom.Polygon p in LitAndNot.Item2)
                            turnPartsLitAndNot.Add(new wktPolygonLit { wktPolygon = p.ToWtk(), sun = false });
                    }
                }

                partsLitAndNot.Add(Tuple.Create(lanePart.Item1, turnPartsLitAndNot));
            }
        }

        /// <summary>
        /// Вычисление зон связи МНКПОИ за заданный промежуток времени.
        /// </summary>
        /// <param name="DBManager">Параметры подключения к БД</param>
        /// <param name="timeFrom">Начало временного промежутка</param>
        /// <param name="timeTo">Конец временного промежутка</param>
        /// <param name="zones">Зоны связи</param>
        public static void getMNKPOICommuncationZones(DIOS.Common.SqlManager DBManager, DateTime timeFrom, DateTime timeTo, out List<CommunicationZoneMNKPOI> zones)
        {
            DataFetcher fetcher = new DataFetcher(DBManager);

            System.Data.DataRow[] prePosRow = fetcher.GetDataBeforeEqualDate(MnkpoiTable.Name, MnkpoiTable.TimeFrom, timeFrom, 1);
            if (prePosRow.Length < 1) // no data
            {
                zones = null;
                return;
            }

            PositionMNKPOI prePos = MnkpoiTable.GetDataMNKPOI(prePosRow[0]);
            List<PositionMNKPOI> positions = new List<PositionMNKPOI>();

            if (prePos.TimeBeg < timeFrom)
                positions.Add(prePos);
            positions.AddRange(fetcher.GetPositionMNKPOI(timeFrom, timeTo));

            double R = Astronomy.Constants.EarthRadius;
            double h = fetcher.GetPositionSat(timeFrom, timeTo).Select(spaceTime => spaceTime.Position.Length - R).Average();

            zones = new List<CommunicationZoneMNKPOI>();
            foreach (PositionMNKPOI pos in positions)
            {
                double d = pos.Altitude / 1000; // altitude
                zones.Add(new CommunicationZoneMNKPOI
                    {
                        CentreLat = pos.Position.Latitude,
                        CentreLon = pos.Position.Longitude,
                        IdNumber = pos.Number,
                        Radius5 = ZoneRadius(R, h, d, 5),
                        Radius7 = ZoneRadius(R, h, d, 7),
                        From = timeFrom < pos.TimeBeg ? pos.TimeBeg : timeFrom,
                        To = timeTo < pos.TimeEnd ? timeTo : pos.TimeEnd
                    });
            }
            
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="R">Planet radius [km]</param>
        /// <param name="h">Orbit height [km]</param>
        /// <param name="d">Altitude [km]</param>
        /// <param name="a">Zone angle [deg]</param>
        /// <returns>радиус зоны в километрах</returns>
        private static double ZoneRadius(double R, double h, double d, double a)
        {
            double c = Math.Cos(AstronomyMath.ToRad(a));
            double s = Math.Sin(AstronomyMath.ToRad(a));
            return -(R + d) * c * s + c * Math.Sqrt((R + d) * (R + d) * s * s + (2 * R + h + d) * (h - d));
        }

        /// <summary>
        /// ошибка создания маршрута
        /// </summary>
        public enum createMPZStatus
        {
            eSuccses,
            eTotLong,            // Длительность МПЗ превышена
            eRoutesIncompatible, // Маршрута противоречат друг другу
            eIncorrectNumber     // Слишком много/мало маршрутов (если 0 или если больше 12)
        }

        /// <summary>
        /// Создание МПЗ по заданным маршрутам и доп.параметрам
        /// </summary>
        /// <param name="routes">Набор маршрутов </param>
        /// <param name="PWR_ON">признак PWR_ON</param>
        /// <param name="mpz">параметры МПЗ</param>
        /// <param name="error">ошибка создания</param>
        public static void createMPZ(List<RouteMPZ> routes, int PWR_ON, out MPZ mpz, out createMPZStatus error)
        { 
            mpz = new MPZ(routes);
            error = createMPZStatus.eSuccses;
        }

        /// <summary>
        /// Расчет сеансов связи за заданный период времени
        /// </summary>
        /// <param name="timeFrom">Начало временного отрезка</param>
        /// <param name="timeTo">Конец временного отрезка</param>
        /// <returns>Все возможные сеансы связи за это время</returns>
        public static List<CommunicationSession> createCommunicationSessions(DateTime timeFrom, DateTime timeTo, DIOS.Common.SqlManager managerDB)
        {
            DataFetcher fetcher = new DataFetcher(managerDB);
            List<CommunicationZoneMNKPOI> zones;
            getMNKPOICommuncationZones(managerDB, timeFrom, timeTo, out zones);

            Trajectory trajectory = fetcher.GetTrajectorySat(timeFrom, timeTo);

            int count = trajectory.Count;
            var points = trajectory.Points;

            List<CommunicationSession> sessions = new List<CommunicationSession>();

            //CommunicationSession 
            foreach (var zone in zones)
            {
                bool prevIn5zone = false;
                bool prevIn7zone = false;
                CommunicationSession tempSession = new CommunicationSession();
                for (int i = 0; i < count; i++)
                {
                    TrajectoryPoint point = points[i]; // @todo struct копироованиe?

                    bool in5zone, in7zone;
                    zone.isPointInZone(point.Position, out in5zone, out in7zone);

                    if (in5zone && !prevIn5zone) // текущая точка в 5ти гр. зоне, причем предыдущая не в пятиградусной зоне (или текущая точка - первая (i==0)  )
                    {
                        tempSession.Zone5timeFrom = point.Time; /// @todo интерполяция                                                                
                    }

                    if (in7zone && !prevIn7zone) // текущая точка в 7ти гр. зоне, причем предыдущая не в 7ти гр. зоне )
                    {
                        tempSession.Zone7timeFrom = point.Time; /// @todo интерполяция    
                    }

                    if (!in7zone && prevIn7zone) // вышли из семиградусной зоны (текущая точка не в семиградусной зоне, а предыдущая в семиградусной)
                    {
                        tempSession.Zone7timeTo = point.Time; /// @todo интерполяция    
                    }

                    if ((!in5zone || i == count) && prevIn5zone) // предыдущая точка в пятиградусной зоне, а текущая точка последняя или находится вне пятиградусной зоны
                    {
                        tempSession.Zone5timeTo = point.Time; /// @todo интерполяция                        
                        tempSession.antennaId = zone.IdNumber;
                        sessions.Add(tempSession);
                        tempSession = new CommunicationSession();
                    }

                    prevIn5zone = in5zone;
                    prevIn7zone = in7zone;                                        
                }
            }

            /*
            List<CommunicationSession> sessions = new List<CommunicationSession>();
            double duration = (timeTo - timeFrom).TotalMinutes;
            {
                CommunicationSession comSes = new CommunicationSession();
                comSes.id = 3;
                comSes.Zone5timeFrom = timeFrom.AddMinutes(duration / 10);
                comSes.Zone5timeTo = timeTo.AddMinutes(-duration / 10);
                comSes.Zone7timeFrom = timeFrom.AddMinutes(duration / 8);
                comSes.Zone7timeTo = timeTo.AddMinutes(-duration / 8);

                comSes.routesToReset = new List<RouteMPZ>();
                comSes.routesToReset.Add(new RouteMPZ(RegimeTypes.SI));
                comSes.routesToReset.Add(new RouteMPZ(RegimeTypes.NP));
                sessions.Add(comSes);
            }
            {
                CommunicationSession comSes = new CommunicationSession();
                comSes.id = 2;
                comSes.Zone5timeFrom = timeFrom.AddMinutes(duration / 9);
                comSes.Zone5timeTo = timeTo.AddMinutes(-duration / 9);
                comSes.Zone7timeFrom = timeFrom.AddMinutes(duration / 7);
                comSes.Zone7timeTo = timeTo.AddMinutes(-duration / 7);

                comSes.routesToReset = new List<RouteMPZ>();
                comSes.routesToReset.Add(new RouteMPZ(RegimeTypes.VI));
                comSes.routesToReset.Add(new RouteMPZ(RegimeTypes.ZI));
                sessions.Add(comSes);
            }
             */
            return sessions;
        }

        /// <summary>
        /// Создание маршрута на съемку
        /// </summary>
        /// <param name="timeFrom">Время начала съемки</param>
        /// <param name="duration">длительность съемки (милисекунды)</param>
        /// <param name="pitchAngle">тангаж</param>
        /// <param name="rollAngle">крен</param>
        /// <returns>Параметры маршрута</returns>
        public static RouteMPZ createRouteToCapture(DateTime timeFrom, int duration, double pitchAngle, double rollAngle)
        {
            return new RouteMPZ(RegimeTypes.SI);
        }

        /// <summary>
        /// Создание маршрута на удаление
        /// </summary>
        /// <param name="timeFrom">Время начала маршрута </param>
        /// <param name="MPZ_Id">номер МПЗ удаляемого маршрута</param>
        /// <param name="routeId">номер удаляемого маршрута</param>
        /// <returns>Параметры маршрута</returns>
        public static RouteMPZ createRouteToDelete(DateTime timeFrom, int MPZ_Id, int routeId)
        {
            return new RouteMPZ(RegimeTypes.SI);
        }

        /// <summary>
        /// Создание маршрута на сброс
        /// </summary>
        /// <param name="timeFrom">Время начала маршрута </param>
        /// <param name="MPZ_Id">номер МПЗ сбрасываемого маршрута</param>
        /// <param name="routeId">номер сбрасываемого маршрута</param>
        /// <param name="antennaId">идентификатор антенны</param>
        /// <returns>Параметры маршрута</returns>
        public static RouteMPZ createRouteToReset(DateTime timeFrom, int MPZ_Id, int routeId, int antennaId)
        {
            return new RouteMPZ(RegimeTypes.SI);
        }

        /// <summary>
        /// Создание маршрута на съемку со сбросом
        /// </summary>
        /// <param name="timeFrom">Время начала маршрута </param>
        /// <param name="duration">длительность съемки (милисекунды)</param>
        /// <param name="pitchAngle">тангаж</param>
        /// <param name="rollAngle">крен</param>
        /// <param name="MPZ_Id">номер МПЗ сбрасываемого маршрута</param>
        /// <param name="routeId">номер сбрасываемого маршрута</param>
        /// <param name="antennaId">идентификатор антенны</param>
        /// <returns>Параметры маршрута</returns>
        public static RouteMPZ createRouteToCaptureWithReset(DateTime timeFrom, int duration, double pitchAngle, double rollAngle, int MPZ_Id, int routeId, int antennaId)
        {
            return new RouteMPZ(RegimeTypes.SI);
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
     
    /*
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
    */

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
 
    public struct CommunicationZoneMNKPOI
    {
        public int IdNumber;
        public DateTime From;
        public DateTime To;
        public double CentreLat;
        public double CentreLon;
        public double Radius5;
        public double Radius7;


        public void isPointInZone(Point3D checkPoint, out bool in5zone, out bool in7zone )
        {
            GeoPoint point = GeoPoint.FromCartesian(checkPoint.ToVector());
            GeoPoint centre = new GeoPoint(CentreLat, CentreLon);
            double dist = GeoPoint.DistanceOverSurface(centre, point) * Astronomy.Constants.EarthRadius;
            in5zone = dist <= Radius5;
            in7zone = dist <= Radius7;            
        }

  

    }
 
}
