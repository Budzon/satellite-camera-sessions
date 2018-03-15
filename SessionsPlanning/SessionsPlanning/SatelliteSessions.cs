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
        public CommunicationSession()
        {
            routesToDrop = new List<RouteMPZ>();
        }
        /// <summary>
        ///  Id антенны
        /// </summary>
        public string nkpoiType { get; set; } // "MIGS" or "FIGS"
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
        public List<RouteMPZ> routesToDrop { get; set; }
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
        public static void isRequestFeasible(RequestParams request, DateTime timeFrom, DateTime timeTo, DIOS.Common.SqlManager managerDB, out double coverage, out List<CaptureConf> possibleConfs)
        {
            // string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "trajectory_1day.dat";     
            // Astronomy.Trajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, timeFrom, timeTo); // @todo временно      
            DataFetcher fetcher = new DataFetcher(managerDB);
            Trajectory trajectory = fetcher.GetTrajectorySat(timeFrom, timeTo);

            double maxRoll = Math.Min(OptimalChain.Constants.max_roll_angle, request.Max_SOEN_anlge);
            double viewAngle = maxRoll * 2 + OptimalChain.Constants.camera_angle; 
            SatLane viewLane = new SatLane(trajectory, 0, 0, viewAngle);
            possibleConfs = viewLane.getCaptureConfs(request);
            double summ = 0;

            /////// @todo костыль ////////////
            foreach (var conf in possibleConfs)
            {
                foreach (var order in conf.orders)
                {
                    summ += order.intersection_coeff;
                }
            }
            if (summ > 1)
                summ = 1;

            coverage = summ;
            ///////////////////////////
            /*
            List<SphericalGeom.Polygon> region = new List<SphericalGeom.Polygon> { new SphericalGeom.Polygon(request.wktPolygon) };
 
            foreach (var conf in possibleConfs)
            {
                foreach (var order in conf.Orders)
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
            */
        }

        private static void getCaptureConfArrayForTrajectory(IList<RequestParams> requests, Trajectory trajectory, List<CaptureConf> captureConfs)
        { 
            double viewAngle = OptimalChain.Constants.camera_angle; // угол обзора камеры
            double angleStep = viewAngle; // шаг равен углу обзора

            double Max_SOEN_anlge = requests[0].Max_SOEN_anlge;
            foreach (var req in requests)
            {
                if (req.Max_SOEN_anlge > Max_SOEN_anlge)
                    Max_SOEN_anlge = req.Max_SOEN_anlge;
            }

            double max_roll_angle = Math.Min(Max_SOEN_anlge, OptimalChain.Constants.max_roll_angle);
            double min_roll_angle = Math.Max(-Max_SOEN_anlge, -OptimalChain.Constants.max_roll_angle);

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
                    var pol = viewLane.getSegment(conf.dateFrom, conf.dateTo);
                    TrajectoryPoint pointFrom = trajectory.GetPoint(conf.dateFrom);
                    TrajectoryPoint pointTo = trajectory.GetPoint(conf.dateTo);

                    conf.setPolygon(pol);
                    
                    calculatePitchArrays(conf, rollAngle, pointFrom);
                }
                captureConfs.AddRange(laneCaptureConfs);
            }
            );              
        }

        public static List<CaptureConf> getCaptureConfArray(
            IList<RequestParams> requests,
            DateTime timeFrom,
            DateTime timeTo,
            DIOS.Common.SqlManager managerDB,
            List<Tuple<DateTime, DateTime>> inactivityRanges)
        {
            if (requests.Count == 0)
                return new List<CaptureConf>();

            //string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "\\" + "trajectory_1day.dat";
            //Astronomy.Trajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, timeFrom, timeTo); // @todo временно
            DataFetcher fetcher = new DataFetcher(managerDB);

            inactivityRanges.Sort(delegate(Tuple<DateTime, DateTime> span1, Tuple<DateTime, DateTime> span2) { return span1.Item1.CompareTo(span2.Item1); });

            List<Trajectory> trajSpans = new List<Trajectory>();

            DateTime firstDt = timeFrom;

            foreach (var timeSpan in inactivityRanges)
            {
                if (firstDt < timeSpan.Item1)                
                    trajSpans.Add(fetcher.GetTrajectorySat(firstDt, timeSpan.Item1));                
                firstDt = timeSpan.Item2;
            }

            trajSpans.Add(fetcher.GetTrajectorySat(firstDt, timeTo));

            List<CaptureConf> captureConfs = new List<CaptureConf>();

            foreach (var trajectory in  trajSpans)
                getCaptureConfArrayForTrajectory(requests, trajectory, captureConfs);

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
        /// <param name="routesToDrop">Перечень маршрутов на сброс</param>
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
            , List<RouteMPZ> routesToDrop
            , List<RouteMPZ> routesToDelete
            , DIOS.Common.SqlManager managerDB
            , out List<MPZ> mpzArray
            , out List<CommunicationSession> sessions)
        {
            List<CaptureConf> confsToCapture = getCaptureConfArray(requests, timeFrom, timeTo, managerDB, inactivityRanges);
            ///@todo реализовать всё, что касается параметров silentRanges, routesToReset, routesToDelete
         
            List<CaptureConf> confsToDelete = getConfsToDelete(routesToDelete, timeFrom, timeTo);
            confsToDelete.Sort(delegate(CaptureConf conf1, CaptureConf conf2) { return conf1.dateFrom.CompareTo(conf2.dateTo); }); // сортируем по времени завершения
            
            List<Tuple<DateTime, DateTime>> silentAndCaptureRanges = confsToCapture.Select(conf => new Tuple<DateTime, DateTime>(conf.dateFrom, conf.dateTo)).ToList();
            silentAndCaptureRanges.AddRange(silentRanges);

            if (confsToDelete.Count > 0) // если есть конигурации на удаление, нам с ними нельзя пересекаться.
            {
                DateTime deleteTimeTo = confsToDelete.Last().dateTo;
                if (timeFrom != deleteTimeTo)
                    silentAndCaptureRanges.Add(new Tuple<DateTime, DateTime>(timeFrom, deleteTimeTo));
            }
            silentAndCaptureRanges = compressTimeRanges(silentAndCaptureRanges);

            List<CommunicationSession> snkpoiSessions = getAllSNKPOICommunicationSessions(timeFrom, timeTo, managerDB);
            List<CommunicationSession> mnkpoiSessions = getAllMNKPOICommunicationSessions(timeFrom, timeTo, managerDB);

            List<CommunicationSession> nkpoiSessions = new List<CommunicationSession>();
            nkpoiSessions.AddRange(snkpoiSessions);
            nkpoiSessions.AddRange(mnkpoiSessions);

            List<Tuple<DateTime, DateTime>> freeRanges = new List<Tuple<DateTime, DateTime>>();            
            foreach (var session in nkpoiSessions)
            {
                var timeSpan = new Tuple<DateTime, DateTime>(session.Zone5timeFrom, session.Zone5timeTo);
                List<Tuple<DateTime, DateTime>> freeRangesForSession = getFreeTimeRanges(timeSpan, silentAndCaptureRanges);
                freeRanges.AddRange(freeRangesForSession);
            }
            freeRanges = compressTimeRanges(freeRanges);
            freeRanges = freeRanges.OrderByDescending(range => (range.Item2 - range.Item1).TotalSeconds).ToList();

            List<CaptureConf> confsToDrop = getConfsToDrop(routesToDrop, freeRanges);

            List<CaptureConf> allConfs = new List<CaptureConf>();
            allConfs.AddRange(confsToDelete);
            allConfs.AddRange(confsToDrop);
            allConfs.AddRange(confsToCapture);

            Graph graph = new Graph(allConfs);
            List<OptimalChain.MPZParams> mpz_params = graph.findOptimalChain();
            mpzArray = new List<MPZ>();
            foreach (var mpz_param in mpz_params)
            {
                mpzArray.Add(new MPZ(mpz_param));
            }

            sessions = new List<CommunicationSession>();
             
            foreach (var mpz_param in mpz_params)
            {
                List<RouteParams> dropRoutes = mpz_param.routes.Where(route => route.type == 1).ToList();
                List<CommunicationSession> sSessions = new List<CommunicationSession>();
                putRoutesInSessions(dropRoutes, snkpoiSessions, sSessions);
                List<CommunicationSession> mSessions = new List<CommunicationSession>();
                putRoutesInSessions(dropRoutes, mnkpoiSessions, mSessions);
                sessions.AddRange(sSessions);
                sessions.AddRange(mSessions);
            }
        }

        public static List<CaptureConf> getConfsToDrop(List<RouteMPZ> routesToDrop, List<Tuple<DateTime, DateTime>> freeRanges)
        {
            List<CaptureConf> res = new List<CaptureConf>();
            
            double summFreeTime = freeRanges.Sum(range => (range.Item2 - range.Item1).TotalSeconds);

            int prevRouteInd = 0;
            foreach (var range in freeRanges)
            {
                if (routesToDrop.Count == res.Count) // все конифгурации созданы
                    break;

                double curTime = (range.Item2 - range.Item1).TotalSeconds;

                int numRoutes;
                if (freeRanges.Count > routesToDrop.Count)
                    numRoutes = 1;
                else
                    numRoutes = (int)(routesToDrop.Count * curTime / summFreeTime);

                double sumDropTime = routesToDrop.Sum(route => route.Parameters.getDropTime());

                DateTime spanCentre = range.Item1.AddSeconds(curTime / 2); // середина отрезка
                DateTime dropFrom = spanCentre.AddSeconds(sumDropTime / 2); // время начала сброса (без учёта дельты)

                DateTime prevTime = dropFrom;
                for (int i = prevRouteInd; i < prevRouteInd + numRoutes; i++)
                {
                    var route = routesToDrop[i];
                    double roll = route.Parameters.ShootingConf.roll;
                    var connectedRoute = new Tuple<int, int>(route.NPZ, route.Nroute);
                    DateTime dropTimeTo = prevTime.AddSeconds(route.Parameters.getDropTime());
                    CaptureConf newConf = new CaptureConf(prevTime, dropTimeTo, roll, new List<Order>(route.Parameters.ShootingConf.orders), 1, connectedRoute);

                    DateTime dropTimeCentre = prevTime.AddSeconds(route.Parameters.getDropTime() / 2);
                    double timeDelta = Math.Min((dropTimeCentre - range.Item1).TotalSeconds, (range.Item2 - dropTimeCentre).TotalSeconds);
                    newConf.setPitchDependency(new Dictionary<double, double>(), timeDelta);
                    res.Add(newConf);
                }

                prevRouteInd = prevRouteInd + numRoutes;
            }
            return res;
        }

        public static List<CaptureConf> getConfsToDelete(List<RouteMPZ> routesToDelete, DateTime timeFrom, DateTime timeTo)
        {
            List<CaptureConf> res = new List<CaptureConf>();

            DateTime prevDeleteTime = timeFrom;
            List<CaptureConf> confsToDelete = new List<CaptureConf>();
            int deleteInd = 1;
            foreach (RouteMPZ route in routesToDelete)
            {
                DateTime confTimeTo = prevDeleteTime.AddSeconds(OptimalChain.Constants.routeDeleteTime);

                //if (confTimeTo > confsDateFrom) // началась съемка.
                //{
                    // вроде ничего от этого не меняется? @todo
                //}

                double roll = route.Parameters.ShootingConf.roll;
                var connectedRoute = new Tuple<int, int>(route.NPZ, route.Nroute);
                CaptureConf newConf = new CaptureConf(prevDeleteTime, confTimeTo, roll, new List<Order>(), 2, connectedRoute);
                res.Add(newConf);

                if (deleteInd == 12)
                {
                    prevDeleteTime = confTimeTo.AddSeconds(OptimalChain.Constants.bigDeleteInterval);
                    deleteInd = 1;
                }
                else
                {
                    prevDeleteTime = confTimeTo.AddSeconds(OptimalChain.Constants.smallDeleteInterval);
                    deleteInd++;
                }
            } 

            return res;
        }
                 
        public static List<Tuple<DateTime, DateTime>> getFreeTimeRanges(Tuple<DateTime, DateTime> timeSpan, List<Tuple<DateTime, DateTime>> forbiddenRanges)
        {
            List<Tuple<DateTime, DateTime>> compressedSilentAndCaptureRanges = compressTimeRanges(forbiddenRanges);
            return invertTimeSpans(compressedSilentAndCaptureRanges, timeSpan.Item1, timeSpan.Item2);  
        }

        public static List<Tuple<DateTime, DateTime>> compressTimeRanges(List<Tuple<DateTime, DateTime>> silentRanges)
        {
            // соеденим пересекающиеся диапазоны
            List<Tuple<DateTime, DateTime>> compressedSilentAndCaptureRanges = new List<Tuple<DateTime, DateTime>>(silentRanges);
            compressedSilentAndCaptureRanges.Sort(delegate(Tuple<DateTime, DateTime> span1, Tuple<DateTime, DateTime> span2) { return span1.Item1.CompareTo(span2.Item1); });
            List<Tuple<DateTime, DateTime>> res = new List<Tuple<DateTime, DateTime>>();

            for (int i = 0; i < compressedSilentAndCaptureRanges.Count; i++)
            {
                var curRange = compressedSilentAndCaptureRanges[i];
                for (int j = i+1; j < compressedSilentAndCaptureRanges.Count; j++)
                {
                    var comRange = compressedSilentAndCaptureRanges[j];
                    if (curRange.Item1 <= comRange.Item1 && comRange.Item1 <= curRange.Item2 ||
                        curRange.Item1 <= comRange.Item2 && comRange.Item2 <= curRange.Item2)
                    {
                        var itemFrom = curRange.Item1 < comRange.Item1 ? curRange.Item1 : comRange.Item1;
                        var itemTo = curRange.Item2 > comRange.Item2 ? curRange.Item2 : comRange.Item2;
                        curRange = new Tuple<DateTime, DateTime>(itemFrom, itemTo);
                        compressedSilentAndCaptureRanges.Remove(comRange);
                        j--;
                    }
                }
                res.Add(curRange);
            }
            return res;
        }

        public static List<Tuple<DateTime, DateTime>> invertTimeSpans(List<Tuple<DateTime, DateTime>> inputSpans, DateTime timeFrom, DateTime timeTo)
        {
            inputSpans.Sort(delegate(Tuple<DateTime, DateTime> span1, Tuple<DateTime, DateTime> span2) { return span1.Item1.CompareTo(span2.Item1); });

            List<Tuple<DateTime, DateTime>> res = new List<Tuple<DateTime, DateTime>>();

            DateTime firstDt = timeFrom;

            foreach (var timeSpan in inputSpans)
            {
                if (firstDt != timeSpan.Item1)
                    res.Add(new Tuple<DateTime, DateTime>(firstDt, timeSpan.Item1));
                firstDt = timeSpan.Item2;
            }

            if (firstDt != timeTo)
                res.Add(new Tuple<DateTime, DateTime>(firstDt, timeTo));

            return res;
        }

        private static void putRoutesInSessions(List<RouteParams> routes, List<CommunicationSession> nkpoiSessions, List<CommunicationSession> finalSessions)
        {
            foreach (var route in routes)
            {
                bool success = false;
                foreach (var session in finalSessions) // сначала пробуем добавить в уже использующиеся сессии
                {
                    if (session.Zone7timeFrom <= route.start &&
                         route.end <= session.Zone7timeTo)
                    {
                        session.routesToDrop.Add(new RouteMPZ(route));
                        routes.Remove(route);
                        success = true;
                        break;
                    }
                }
                if (!success) // берём новую сессию из nkpoiSessions
                {
                    foreach (var session in nkpoiSessions)
                    {
                        // тут пробуем впихнуть
                        if (session.Zone7timeFrom <= route.start &&
                             route.end <= session.Zone7timeTo)
                        {
                            session.routesToDrop.Add(new RouteMPZ(route));
                            nkpoiSessions.Remove(session);
                            routes.Remove(route);
                            break;
                        }
                    }
                }

            }
        }


        /// <summary>
        /// Создание ПНб по набору маршрутов
        /// </summary>
        /// <param name="routesParams"> Набор маршрутов RouteParams</param>
        /// <returns> набор МПЗ, созданный из маршутов</returns>
        public static List<MPZ> createPNbOfRoutes(List<RouteParams> routesParams)
        {
            List<RouteMPZ> routes = routesParams.Select(rparams => new RouteMPZ(rparams)).ToList();

            List<MPZ> res = new List<MPZ>();
            List<RouteMPZ> routesTemp = new List<RouteMPZ>();
            for (int i = 0; i < routes.Count; i++)
            {
                routesTemp.Add(routes[i]);
                if (routesTemp.Count == 12 || i == routes.Count - 1)
                {
                    res.Add(new MPZ(routesTemp));
                    routesTemp = new List<RouteMPZ>();
                }
            }

            return res;
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
                SatLane viewLane = new SatLane(trajectory, rollAngle, pitchAngle, OptimalChain.Constants.camera_angle);

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
                            List<Polygon> pieces = SatelliteTrajectory.TrajectoryRoutines.FormSectorFromLanePoints(lane, streakBegin, i).BreakIntoLobes();
                            foreach (Polygon piece in pieces)
                                turnPartsLitAndNot.Add(new wktPolygonLit
                                {
                                    wktPolygon = piece.ToWtk(),
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
                            foreach(Polygon piece in p.BreakIntoLobes())
                                turnPartsLitAndNot.Add(new wktPolygonLit { wktPolygon = piece.ToWtk(), sun = true });
                        foreach (SphericalGeom.Polygon p in LitAndNot.Item2)
                            foreach (Polygon piece in p.BreakIntoLobes())
                                turnPartsLitAndNot.Add(new wktPolygonLit { wktPolygon = piece.ToWtk(), sun = false });
                    }
                }

                partsLitAndNot.Add(Tuple.Create(lanePart.Item1, turnPartsLitAndNot));
            }
        }

        /// <summary>
        /// Вычисление зон связи СНКПОИ и МНКПОИ в заданный момент времени.
        /// </summary>
        /// <param name="DBManager">Параметры подключения к БД</param>
        /// <param name="time">Интересующий момент времени</param>
        /// <param name="snkpoi">Зона связи СНКПОИ</param>
        /// <param name="mnkpoi">Зона связи МНКПОИ</param>
        public static void getCommunicationZones(DIOS.Common.SqlManager DBManager, DateTime time, out CommunicationZoneSNKPOI snkpoi, out CommunicationZoneMNKPOI mnkpoi)
        {
            DataFetcher fetcher = new DataFetcher(DBManager);
            getSNKPOICommunicationZones(DBManager, out snkpoi);

            PositionMNKPOI mnkpoiPos = fetcher.GetPositionMNKPOI(time);
            if (mnkpoiPos == null)
            {
                mnkpoi = null;
            }
            else
            {
                double R = Astronomy.Constants.EarthRadius;
                double h = OptimalChain.Constants.orbit_height;
                double d = mnkpoiPos.Altitude / 1000;
                mnkpoi = new CommunicationZoneMNKPOI
                    {
                        CentreLat = mnkpoiPos.Position.Latitude,
                        CentreLon = mnkpoiPos.Position.Longitude,
                        IdNumber = mnkpoiPos.Number,
                        Radius5 = ZoneRadius(R, h, d, 5),
                        Radius7 = ZoneRadius(R, h, d, 7),
                        From = time,
                        To = time
                    };
            }
        }

        /// <summary>
        /// Вычисление зон связи МНКПОИ за заданный промежуток времени.
        /// </summary>
        /// <param name="DBManager">Параметры подключения к БД</param>
        /// <param name="timeFrom">Начало временного промежутка</param>
        /// <param name="timeTo">Конец временного промежутка</param>
        /// <param name="zones">Зоны связи</param>
        public static void getMNKPOICommunicationZones(DIOS.Common.SqlManager DBManager, DateTime timeFrom, DateTime timeTo, out List<CommunicationZoneMNKPOI> zones)
        {
            DataFetcher fetcher = new DataFetcher(DBManager);
            List<PositionMNKPOI> positions = new List<PositionMNKPOI>();

            // Check if timeFrom is covered by a work interval that started strictly before.
            System.Data.DataRow[] prePosRow = fetcher.GetDataBeforeDate(MnkpoiTable.Name, MnkpoiTable.TimeFrom, timeFrom, 1);
            if (prePosRow.Length > 0)
            {
                PositionMNKPOI prePos = MnkpoiTable.GetDataMNKPOI(prePosRow[0]);
                if (prePos.TimeEnd > timeFrom)
                    positions.Add(prePos);
            }
            // Add work intervals such that timeFrom <= timeBegin < timeTo.
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
        /// Вычисление зоны связи СНКПОИ.
        /// </summary>
        /// <param name="DBManager">Параметры подключения к БД</param>
        /// <param name="zone">Зона связи</param>
        public static void getSNKPOICommunicationZones(DIOS.Common.SqlManager DBManager, out CommunicationZoneSNKPOI zone)
        {
            DataFetcher fetcher = new DataFetcher(DBManager);

            double R = Astronomy.Constants.EarthRadius;
            double h = OptimalChain.Constants.orbit_height; // okay to take as a constant?
            double d = 0; // altitude ?

            GeoPoint snkpoi = fetcher.GetPositionGeoSNKPOI();

            zone = new CommunicationZoneSNKPOI
            {
                CentreLat = snkpoi.Latitude,
                CentreLon = snkpoi.Longitude,
                Radius5 = ZoneRadius(R, h, d, 5),
                Radius7 = ZoneRadius(R, h, d, 7)
            };
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

        public static void getSessionFromZone(CommunicationZone zone, Trajectory trajectory, List<CommunicationSession> sessions)
        {
            Vector3D centre = GeoPoint.ToCartesian(new GeoPoint(zone.CentreLat, zone.CentreLon), 1);

            bool prevIn5zone = false;
            bool prevIn7zone = false;
            CommunicationSession tempSession = new CommunicationSession();

            int count = trajectory.Count;
            var points = trajectory.Points;
            for (int i = 0; i < count; i++)
            {
                TrajectoryPoint point = points[i]; // @todo struct копироованиe?

                bool in5zone = zone.isPointInZone(point.Position, zone.Radius5);
                bool in7zone = zone.isPointInZone(point.Position, zone.Radius7);

                if (in5zone && !prevIn5zone) // текущая точка в 5ти гр. зоне, причем предыдущая не в пятиградусной зоне (или текущая точка - первая (i==0)  )
                {
                    if (i == 0)
                        tempSession.Zone5timeFrom = point.Time;
                    else
                        tempSession.Zone5timeFrom = getIntersectionTime(points[i - 1], point, centre, zone.Radius5);
                }

                if (in7zone && !prevIn7zone) // текущая точка в 7ти гр. зоне, причем предыдущая не в 7ти гр. зоне )
                {
                    if (i == 0)
                        tempSession.Zone7timeFrom = point.Time;
                    else
                        tempSession.Zone7timeFrom = getIntersectionTime(points[i - 1], point, centre, zone.Radius7);
                }

                if (!in7zone && prevIn7zone) // вышли из семиградусной зоны (текущая точка не в семиградусной зоне, а предыдущая в семиградусной)
                {
                    tempSession.Zone7timeTo = getIntersectionTime(points[i - 1], point, centre, zone.Radius7);
                }

                if ((!in5zone || i == count) && prevIn5zone) // предыдущая точка в пятиградусной зоне, а текущая точка последняя или находится вне пятиградусной зоны
                {
                    tempSession.Zone5timeTo = getIntersectionTime(points[i - 1], point, centre, zone.Radius5);
                    tempSession.routesToDrop = new List<RouteMPZ>();
                    //tempSession.antennaId = zone.IdNumber;
                    sessions.Add(tempSession);
                    tempSession = new CommunicationSession();
                }

                prevIn5zone = in5zone;
                prevIn7zone = in7zone;
            }
        }


        private static List<CommunicationSession> getAllSNKPOICommunicationSessions(DateTime timeFrom, DateTime timeTo, DIOS.Common.SqlManager managerDB)
        {
            List<CommunicationSession> sessions = new List<CommunicationSession>();
            DataFetcher fetcher = new DataFetcher(managerDB);
            CommunicationZoneSNKPOI sZone;
            getSNKPOICommunicationZones(managerDB, out sZone);
            Trajectory fullTrajectory = fetcher.GetTrajectorySat(timeFrom, timeTo);
            getSessionFromZone(sZone, fullTrajectory, sessions);
            foreach (var sess in  sessions)            
                sess.nkpoiType = "FIGS";
            
            return sessions;
        }

        private static List<CommunicationSession> getAllMNKPOICommunicationSessions(DateTime timeFrom, DateTime timeTo, DIOS.Common.SqlManager managerDB)
        {
            List<CommunicationSession> sessions = new List<CommunicationSession>();
            DataFetcher fetcher = new DataFetcher(managerDB);
            List<CommunicationZoneMNKPOI> mZones;
            getMNKPOICommunicationZones(managerDB, timeFrom, timeTo, out mZones);

            if (mZones == null)
                return new List<CommunicationSession>();
            if (mZones.Count == 0)
                return new List<CommunicationSession>();

            Trajectory fullTrajectory = fetcher.GetTrajectorySat(timeFrom, timeTo);

            foreach (var zone in mZones)
            {
                Trajectory trajectory;
                if (zone.From > timeFrom || zone.To < timeTo) // если временной промежуток зоны более строгий, чем  timeFrom - timeTo
                    trajectory = fetcher.GetTrajectorySat(zone.From > timeFrom ? zone.From : timeFrom,
                                                          zone.To < timeTo ? zone.To : timeTo); // то загружаем траекторию с заданными временными промежутками
                else
                    trajectory = fullTrajectory;

                getSessionFromZone(zone, trajectory, sessions);
            }

            foreach (var sess in sessions)            
                sess.nkpoiType = "MIGS";
            
            return sessions;
        }

        /// <summary>
        /// Расчет сеансов связи за заданный период времени
        /// </summary>
        /// <param name="timeFrom">Начало временного отрезка</param>
        /// <param name="timeTo">Конец временного отрезка</param>
        /// <returns>Все возможные сеансы связи за это время</returns>
        public static List<CommunicationSession> createCommunicationSessions(DateTime timeFrom, DateTime timeTo, DIOS.Common.SqlManager managerDB)
        {            
            List<CommunicationSession> snkpoSessions = getAllSNKPOICommunicationSessions(timeFrom, timeTo, managerDB);
            List<CommunicationSession> mnkpoSessions = getAllMNKPOICommunicationSessions(timeFrom, timeTo, managerDB);
            List<CommunicationSession> sessions = new List<CommunicationSession>();
            sessions.AddRange(snkpoSessions);
            sessions.AddRange(mnkpoSessions);
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


            double minMaxSoenAngle = conf.orders.Min(order => order.request.Max_SOEN_anlge);

            double maxAngle = conf.orders[0].request.Max_SOEN_anlge;


            foreach (var req in conf.orders)
            {
                if (req.request.Max_SOEN_anlge < maxAngle)
                    maxAngle = req.request.Max_SOEN_anlge;
            }

            double maxPitchAngle = Math.Abs(maxAngle) - Math.Abs(rollAngle);
            double timeDelta;
            if (0 == maxPitchAngle)
            {
                timeDelta = 0;
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
                timeDelta = dist / pointFrom.Velocity.Length;
            }
            int pitchStep = OptimalChain.Constants.pitchStep;
            conf.pitchArray[0] = 0;

            Dictionary<double, double> angleTimeArray = new Dictionary<double, double>();
            angleTimeArray[0] = 0;

            Vector3D dirRollPoint = LanePos.getSurfacePoint(pointFrom, rollAngle, 0);
            for (int pitch_degr = pitchStep; pitch_degr <= AstronomyMath.ToDegrees(maxPitchAngle); pitch_degr += pitchStep)
            {
                double pitch = AstronomyMath.ToRad(pitch_degr);
                Vector3D dirPitchPoint = LanePos.getSurfacePoint(pointFrom, rollAngle, pitch);
                double distOverSurf = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(dirPitchPoint), GeoPoint.FromCartesian(dirRollPoint)) * Astronomy.Constants.EarthRadius;
                double t = distOverSurf / pointFrom.Velocity.Length;
                angleTimeArray[pitch] = t;
            }

            LinearInterpolation interpolation = new LinearInterpolation(angleTimeArray.Values.ToArray(), angleTimeArray.Keys.ToArray());

            Dictionary<double, double> timeAngleArray = new Dictionary<double, double>();
            for (int t = 0; t <= (int)timeDelta; t++)
            {
                timeAngleArray[t] = interpolation.GetValue(t);
            }

            conf.setPitchDependency(timeAngleArray, timeDelta);            
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
        public string wktPolygon { get; set; }
        public bool sun { get; set; }
    }

    public struct TimeInterval
    {
        public DateTime From { get; set; }
        public DateTime To { get; set; }
    }


    public class CommunicationZone
    {
        /// <summary>
        /// In degrees.
        /// </summary>
        public virtual int IdNumber { get; set; }

        /// <summary>
        /// In degrees.
        /// </summary>
        public double CentreLat;
        /// <summary>
        /// In degrees.
        /// </summary>
        public double CentreLon;
        /// <summary>
        /// In km.
        /// </summary>
        public double Radius5;
        /// <summary>
        /// In km.
        /// </summary>
        public double Radius7;

        public bool isPointInZone(Point3D checkPoint, double zoneRadius)
        {
            double angleZone = Math.Asin(zoneRadius / (OptimalChain.Constants.orbit_height + Astronomy.Constants.EarthRadius));
            Vector3D centre = GeoPoint.ToCartesian(new GeoPoint(CentreLat, CentreLon), 1);
            double angleKA = AstronomyMath.ToRad(Vector3D.AngleBetween(checkPoint.ToVector(), centre));
            return angleKA <= angleZone;
        }
    }


    public class CommunicationZoneMNKPOI : CommunicationZone
    {
        public DateTime From;
        public DateTime To;
    }

    public class CommunicationZoneSNKPOI : CommunicationZone
    {
        public override int IdNumber { get { return -1; } }
    }

}
