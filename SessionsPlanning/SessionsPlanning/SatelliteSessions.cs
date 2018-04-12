#define _PARALLEL_

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using System.Collections.Concurrent;

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

        public Tuple<DateTime, DateTime> DropInterval { get { return Tuple.Create(Zone7timeFrom, Zone7timeTo); } }
    }

    public class Sessions
    {
        /// <summary>
        /// Генерирует 15 МПЗ по 12 роутов в каждом с всевозможными комбинациями параметров
        /// режима, сжатия, канала, типа съемки.
        /// </summary>
        /// <param name="managerDB">параметры БД</param>
        /// <param name="mpzs">список из 15 МПЗ</param>
        public static void testMpzFormation(DIOS.Common.SqlManager managerDB, out List<MPZ> mpzs)
        {
            List<RouteParams> param = new List<RouteParams>();
            OptimalChain.StaticConf conf;

            string[] chan = new string[3] { "pk", "mk", "cm" };
            int[] regime = new int[4] { 0, 1, 2, 3 }; // Zi, Vi, Si, Np
            int[] shooting = new int[3] { 0, 1, 2 }; // прост, стерео, коридор
            int[] compression = new int[5] { 0, 1, 2, 7, 10 };
            DateTime from = new DateTime(2019, 1, 5);
            DateTime to = from.AddSeconds(5);

            int k = 0;
            foreach (string ch in chan)
                foreach (int r in regime)
                    foreach (int s in shooting)
                        foreach (int c in compression)
                        {
                            conf = new StaticConf(k, from, to, 0, 0, 0, null, "", c, 0.3, r, ch, s);
                            RouteParams p = new RouteParams(conf);
                            p.albedo = 0.3;
                            p.coridorAzimuth = 0.5;
                            p.coridorLength = 40000;
                            p.Delta_T = 0;
                            p.duration = 10;
                            p.TNPos = 0;
                            p.binded_route = Tuple.Create(101, 1);
                            param.Add(p);
                            from = to.AddSeconds(70);
                            to = from.AddSeconds(5);
                            k++;
                        }
            var mpzParams = OptimalChain.MPZParams.FillMPZ(param);
            FlagsMPZ flags = new FlagsMPZ();
            mpzs = mpzParams.Select(p => new MPZ(p, managerDB, flags)).ToList();
        }

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

            List<Tuple<DateTime, DateTime>> shadowPeriods;
            List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot;
            checkIfViewLaneIsLitWithTimeSpans(managerDB, timeFrom, timeTo, out partsLitAndNot, out shadowPeriods);

            possibleConfs = getCaptureConfArray(
                new List<RequestParams>() { request },
                timeFrom,
                timeTo,
                managerDB,
                shadowPeriods,
                new List<Tuple<DateTime, DateTime>>()
                );
            //possibleConfs.Sort(delegate(CaptureConf conf1, CaptureConf conf2)
            //{
            //    double sum_cover1 = conf1.orders.Sum(conf => conf.intersection_coeff);
            //    double sum_cover1 = conf1.orders.Sum(conf => conf.intersection_coeff);
            //});

            double maxRoll = Math.Min(OptimalChain.Constants.max_roll_angle, request.Max_SOEN_anlge);
            double viewAngle = maxRoll * 2 + OptimalChain.Constants.camera_angle;

            List<Trajectory> possibleTrajParts = getLitTrajectoryParts(timeFrom, timeTo, managerDB, shadowPeriods);
            List<CaptureConf> fictiveBigConfs = new List<CaptureConf>();
            foreach (var traj in possibleTrajParts)
            {
                SatLane viewLane = new SatLane(traj, 0, viewAngle);
                List<CaptureConf> curConfs = viewLane.getCaptureConfs(request);
                fictiveBigConfs.AddRange(curConfs);
            }

            double summ = 0;

            List<SphericalGeom.Polygon> region = new List<SphericalGeom.Polygon> { new SphericalGeom.Polygon(request.wktPolygon) };
            foreach (var conf in fictiveBigConfs)
            {
                foreach (var order in conf.orders)
                {
                    var notCoveredBefore = new List<SphericalGeom.Polygon>();
                    var toBeCoveredAfter = new List<SphericalGeom.Polygon>();
                    for (int i = 0; i < region.Count; ++i)
                    {
                        try
                        {
                            var intAndSub = SphericalGeom.Polygon.IntersectAndSubtract(region[i], order.captured);
                            notCoveredBefore.AddRange(intAndSub.Item1);
                            toBeCoveredAfter.AddRange(intAndSub.Item2);
                        }
                        catch (Exception ex)
                        {
                            // Часть непокрытого региона -- слишком тонкая. Выкинем ее из рассмотрения.
                            region.RemoveAt(i);
                            i--;
                        }
                    }
                    double areaNotCoveredBefore = 0;
                    for (int j = 0; j < notCoveredBefore.Count; ++j)
                    {
                        try
                        {
                            areaNotCoveredBefore += notCoveredBefore[j].Area;
                        }
                        catch (Exception ex)
                        {
                            /// Исключение должно тут случаться только из-за того,
                            /// что очередное покрытие ещё ранее не покрытой части заказа
                            /// столь тонкое, что почти отрезок.
                            /// Define площадь "отрезка" 0.
                        }
                    }
                    summ += order.intersection_coeff * areaNotCoveredBefore / order.captured.Area;
                    notCoveredBefore.Clear();
                    region.Clear();
                    region = toBeCoveredAfter;
                }
            }

            if (summ > 1)
                coverage = 1;
            else
                coverage = summ;
        }

        private static void getCaptureConfArrayForTrajectory(
            IList<RequestParams> requests,
            Trajectory trajectory,
            List<CaptureConf> captureConfs,
            List<Tuple<DateTime, DateTime>> freeSessionPeriodsForDrop,
            List<Tuple<DateTime, DateTime>> capturePeriods)
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

            ConcurrentBag<CaptureConf> concurrentlist = new ConcurrentBag<CaptureConf>();


#if _PARALLEL_
            Parallel.For(0, num_steps + 1, index =>
            {
                double rollAngle = min_roll_angle + index * angleStep;
#else
                for (double rollAngle = min_roll_angle; rollAngle <= max_roll_angle; rollAngle += angleStep)    {        
#endif
                List<CaptureConf> laneCaptureConfs = new List<CaptureConf>(); // участки захвата для текущий линии захвата
                SatLane viewLane = new SatLane(trajectory, rollAngle, viewAngle);
                foreach (var request in requests)
                {
                    if (Math.Abs(rollAngle) > Math.Abs(request.Max_SOEN_anlge))
                        continue;
                    List<CaptureConf> confs = viewLane.getCaptureConfs(request);

                    if (confs.Count == 0)
                        continue;

                    // если сжатие заказа == 10, то для всех конифгураций, помещающихся в зону дейтвия НКПОИ мы выставляем режим "съемка со сбросом"
                    if (request.compression == OptimalChain.Constants.compressionDropCapture)
                    {
                        var confsToFropCapt = confs.Where(cc => isPeriodInPeriods(Tuple.Create(cc.dateFrom, cc.dateTo), freeSessionPeriodsForDrop)).ToList();
                        foreach (var conf in confsToFropCapt)
                            conf.confType = 3;
                    }

                    // если заказ - стерео, то пробуем его снять в стерео.
                    if (1 == request.shootingType)
                    {
                        for (int i = 0; i < confs.Count; i++)
                        {
                            TrajectoryPoint pointFrom = trajectory.GetPoint(confs[i].dateFrom);
                            confs[i].converToStereoTriplet(pointFrom, capturePeriods);
                        }
                    }

                    CaptureConf.compressCConfArray(confs);
                    CaptureConf.compressTwoCConfArrays(laneCaptureConfs, confs);
                    laneCaptureConfs.AddRange(confs);
                }

                foreach (var conf in laneCaptureConfs)
                {
                    var pol = viewLane.getSegment(conf.dateFrom, conf.dateTo);
                    TrajectoryPoint pointFrom = trajectory.GetPoint(conf.dateFrom);
                    TrajectoryPoint pointTo = trajectory.GetPoint(conf.dateTo);

                    conf.setPolygon(pol);
                    if (conf.pitchArray.Count == 0) // если уже не рассчитали (в случае стереосъемки)
                        calculatePitchArrays(conf, pointFrom);
                }
                foreach (var conf in laneCaptureConfs)
                    concurrentlist.Add(conf);
            }
#if _PARALLEL_
            );
#endif

            captureConfs.AddRange(concurrentlist.ToList());
        }


        public static List<Trajectory> getLitTrajectoryParts(DateTime timeFrom, DateTime timeTo, DIOS.Common.SqlManager managerDB, List<Tuple<DateTime, DateTime>> shadowPeriods)
        {
            DataFetcher fetcher = new DataFetcher(managerDB);
            DateTime firstDt = timeFrom;
            List<Trajectory> posiibleTrajectoryParts = new List<Trajectory>();
            foreach (var timeSpan in shadowPeriods)
            {
                if (firstDt < timeSpan.Item1)
                    posiibleTrajectoryParts.Add(fetcher.GetTrajectorySat(firstDt, timeSpan.Item1));
                firstDt = timeSpan.Item2;
            }

            if (firstDt < timeTo)
                posiibleTrajectoryParts.Add(fetcher.GetTrajectorySat(firstDt, timeTo));

            return posiibleTrajectoryParts;
        }

        public static List<CaptureConf> getCaptureConfArray(
            IList<RequestParams> requests,
            DateTime timeFrom,
            DateTime timeTo,
            DIOS.Common.SqlManager managerDB,
            List<Tuple<DateTime, DateTime>> inactivityRanges,
            List<Tuple<DateTime, DateTime>> freeSessionPeriodsForDrop)
        {
            if (requests.Count == 0)
                return new List<CaptureConf>();

            //string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "\\" + "trajectory_1day.dat";
            //Astronomy.Trajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, timeFrom, timeTo); // @todo временно
            DataFetcher fetcher = new DataFetcher(managerDB);

            inactivityRanges.Sort(delegate(Tuple<DateTime, DateTime> span1, Tuple<DateTime, DateTime> span2) { return span1.Item1.CompareTo(span2.Item1); });

            List<Trajectory> trajSpans = getLitTrajectoryParts(timeFrom, timeTo, managerDB, inactivityRanges);

            List<CaptureConf> captureConfs = new List<CaptureConf>();

            // периоды, во время которых можно проводить съемку.
            List<Tuple<DateTime, DateTime>> capturePeriods = getFreeIntervals(inactivityRanges, timeFrom, timeTo);

            foreach (var trajectory in trajSpans)
                getCaptureConfArrayForTrajectory(requests, trajectory, captureConfs, freeSessionPeriodsForDrop, capturePeriods);

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
        /// <param name="Nmax">номер,с которого мпз следует нумеровать</param>
        /// <param name="mpzArray">Набор МПЗ</param>
        /// <param name="sessions">Сеансы связи</param>
        /// <param name="flags">Флаги для настройки МПЗ</param>
        public static void getMPZArray(
              List<RequestParams> requests
            , DateTime timeFrom
            , DateTime timeTo
            , List<Tuple<DateTime, DateTime>> silentRanges
            , List<Tuple<DateTime, DateTime>> inactivityRanges
            , List<RouteMPZ> routesToDrop
            , List<RouteMPZ> routesToDelete
            , DIOS.Common.SqlManager managerDB
            , int Nmax
            , out List<MPZ> mpzArray
            , out List<CommunicationSession> sessions
            , FlagsMPZ flags = null)
        {
            //List<RouteMPZ> routesToDropCopy = new List<RouteMPZ>(routesToDrop); // локальная копия, не будем портить входной массив
            //List<RouteMPZ> routesToDeleteCopy = new List<RouteMPZ>(routesToDelete); // локальная копия, не будем портить входной массив

            List<CommunicationSession> snkpoiSessions = getAllSNKPOICommunicationSessions(timeFrom, timeTo, managerDB);
            List<CommunicationSession> mnkpoiSessions = getAllMNKPOICommunicationSessions(timeFrom, timeTo, managerDB);

            List<CommunicationSession> nkpoiSessions = new List<CommunicationSession>();
            nkpoiSessions.AddRange(snkpoiSessions);
            nkpoiSessions.AddRange(mnkpoiSessions);

            // временные периоды, во время которых можно проводить съемку со сбросом
            List<Tuple<DateTime, DateTime>> freeSessionPeriodsForDrop = getFreeTimePeriodsOfSessions(nkpoiSessions, silentRanges);

            List<Tuple<DateTime, DateTime>> shadowPeriods;
            List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot;
            checkIfViewLaneIsLitWithTimeSpans(managerDB, timeFrom, timeTo, out partsLitAndNot, out shadowPeriods);

            // учёт освещённости
            List<Tuple<DateTime, DateTime>> shadowAndInactivityPeriods = new List<Tuple<DateTime, DateTime>>();
            shadowAndInactivityPeriods.AddRange(inactivityRanges);
            shadowAndInactivityPeriods.AddRange(shadowPeriods);
            shadowAndInactivityPeriods = compressTimePeriods(shadowAndInactivityPeriods);

            // расчёт всех возможных конфигураций съемки на этот период с учётом ограничений
            List<CaptureConf> confsToCapture = getCaptureConfArray(requests, timeFrom, timeTo, managerDB, shadowAndInactivityPeriods, freeSessionPeriodsForDrop);

            // поиск оптимального набора маршрутов среди всех возможных конфигураций
            Graph captureGraph = new Graph(confsToCapture);
            List<MPZParams> captureMPZParams = captureGraph.findOptimalChain(Nmax);

            // Найдём все возможные промежутки времени для сброса (из диапазона [timeFrom - timeTo] вычитаются все inactivityRanges и диапазоны съемки)
            List<Tuple<DateTime, DateTime>> captureIntervals = captureMPZParams.Select(mpz => new Tuple<DateTime, DateTime>(mpz.start, mpz.end)).ToList();
            List<Tuple<DateTime, DateTime>> silentAndCaptureRanges = new List<Tuple<DateTime, DateTime>>();
            silentAndCaptureRanges.AddRange(silentRanges);
            silentAndCaptureRanges.AddRange(captureIntervals);
            silentAndCaptureRanges = compressTimePeriods(silentAndCaptureRanges);
            List<Tuple<DateTime, DateTime>> freeRangesForDrop = getFreeTimePeriodsOfSessions(nkpoiSessions, silentAndCaptureRanges);

            List<MPZ> captureMpz = new List<MPZ>();
            foreach (var mpz_param in captureMPZParams)
                captureMpz.Add(new MPZ(mpz_param, managerDB, flags ?? new FlagsMPZ()));

            List<RouteMPZ> allRoutesToDrop = new List<RouteMPZ>();
            allRoutesToDrop.AddRange(routesToDrop);
            allRoutesToDrop.AddRange(captureMpz.SelectMany(mpz => mpz.Routes).Where(route => route.Parameters.type == 0).ToList()); // добавим к сбросу только что отснятые маршруты

            int maxRouteDropId = routesToDrop.Select(route => route.Nroute).DefaultIfEmpty(0).Max();
            int maxRouteDeleteId = routesToDelete.Select(route => route.Nroute).DefaultIfEmpty(0).Max();
            int maxCaptureRouteId = captureMPZParams.SelectMany(mpz => mpz.routes).Select(route => route.id).DefaultIfEmpty(0).Max(); 
            int maxRoutesNumber = Math.Max(maxRouteDropId, Math.Max(maxRouteDeleteId, maxCaptureRouteId));
            
            Dictionary<Tuple<DateTime, DateTime>, List<RouteParams>> dropRoutesParamsByIntervals = getRoutesParamsInIntervals(allRoutesToDrop, freeRangesForDrop, workType: 1, startId: maxRoutesNumber);

            foreach (var intervalRoutes in dropRoutesParamsByIntervals)
            {
                List<RouteParams> routparamsList = intervalRoutes.Value;
                Tuple<DateTime, DateTime> interval = intervalRoutes.Key;

                // попробуем вместить созданные маршрты на сброс в уже созданные маршруты на съемку
                foreach (var routparams in routparamsList.ToArray())
                {
                    foreach (var capPrms in captureMPZParams)
                    {
                        if (capPrms.InsertRoute(routparams, interval.Item1, interval.Item2, null, null))
                        {
                            routparamsList.Remove(routparams); // удаляем из списка маршрутов те, которые поместились в МПЗ
                        }
                    }
                }
            }

            int maxMpzNum = captureMPZParams.Select(mpzparam => mpzparam.id).DefaultIfEmpty(Nmax).Max();

            // теперь создаем новые мпз сброса из оставшихся маршрутов
            List<MPZParams> dropMpzParams = new List<MPZParams>();
            foreach (var intervalRoutes in dropRoutesParamsByIntervals)
            {
                List<RouteParams> routparamsList = intervalRoutes.Value;
                // Tuple<DateTime, DateTime> interval = intervalRoutes.Key;
                if (routparamsList.Count > 0)
                {
                    int curMaxMpzNum = dropMpzParams.Select(mpzparam => mpzparam.id).DefaultIfEmpty(maxMpzNum).Max();
                    List<MPZParams> curMPZ = MPZParams.FillMPZ(routparamsList, curMaxMpzNum);
                    dropMpzParams.AddRange(curMPZ);
                }
            }
             
            List<Tuple<DateTime, DateTime>> dropIntervals = dropMpzParams.Select(mpzparams => new Tuple<DateTime, DateTime>(mpzparams.start, mpzparams.end)).ToList();
            List<Tuple<DateTime, DateTime>> inactivityDropCaptureIntervals = new List<Tuple<DateTime, DateTime>>();
            inactivityDropCaptureIntervals.AddRange(captureIntervals);
            inactivityDropCaptureIntervals.AddRange(dropIntervals);
            inactivityDropCaptureIntervals.AddRange(inactivityRanges);
            inactivityDropCaptureIntervals = compressTimePeriods(inactivityDropCaptureIntervals);
            List<Tuple<DateTime, DateTime>> freeRangesForDelete = getFreeIntervals(inactivityDropCaptureIntervals, timeFrom, timeTo);


            maxRouteDropId = dropMpzParams.SelectMany(mpz => mpz.routes).Select(route => route.id).DefaultIfEmpty(0).Max();
            maxCaptureRouteId = captureMPZParams.SelectMany(mpz => mpz.routes).Select(route => route.id).DefaultIfEmpty(0).Max();
     
            maxRoutesNumber = Math.Max(maxRouteDropId, Math.Max(maxRouteDeleteId, maxCaptureRouteId));
            Dictionary<Tuple<DateTime, DateTime>, List<RouteParams>> deleteRoutesParamsByIntervals = getRoutesParamsInIntervals(routesToDelete, freeRangesForDelete, workType: 2, startId: maxRoutesNumber);

            maxMpzNum = dropMpzParams.Select(mpzparam => mpzparam.id).DefaultIfEmpty(maxMpzNum).Max();

            // создаем новые мпз удаления из маршрутов на удаление
            List<MPZParams> deleteMpzParams = new List<MPZParams>();
            foreach (var intervalRoutes in deleteRoutesParamsByIntervals)
            {
                List<RouteParams> routparamsList = intervalRoutes.Value;
                //Tuple<DateTime, DateTime> interval = intervalRoutes.Key;
                int curMaxMpzNum = deleteMpzParams.Select(mpzparam => mpzparam.id).DefaultIfEmpty(maxMpzNum).Max();
                List<MPZParams> curMPZ = MPZParams.FillMPZ(routparamsList, curMaxMpzNum);
                deleteMpzParams.AddRange(curMPZ);
            }


            List<MPZParams> allMPZParams = new List<MPZParams>();
            //            allMPZParams.AddRange(captureMPZParams);
            allMPZParams.AddRange(dropMpzParams);
            allMPZParams.AddRange(deleteMpzParams);

            mpzArray = new List<MPZ>();

            foreach (var mpz_param in allMPZParams)
                mpzArray.Add(new MPZ(mpz_param, managerDB, flags ?? new FlagsMPZ()));


            mpzArray.InsertRange(0, captureMpz);

            // составим массив использованных сессий

            sessions = new List<CommunicationSession>();

            List<Tuple<DateTime, DateTime>> allDropIntervals = new List<Tuple<DateTime, DateTime>>(); // все использованные интервалы связи (на сброс и на *съемку и сброс*)
            allDropIntervals.AddRange(dropMpzParams.SelectMany(mpz => mpz.routes).Select(route => Tuple.Create(route.start, route.end)));

            var allRoutesLists = captureMPZParams.Select(mpz => mpz.routes).ToList();
            List<RouteParams> allRoutes = new List<RouteParams>();
            foreach (var r in allRoutesLists)
                allRoutes.AddRange(r);

            var droproutes = allRoutes.Where(rout => (rout.type == 1 || rout.type == 3)).ToList();

            allDropIntervals.AddRange(droproutes.Select(rout => Tuple.Create(rout.start, rout.end)));

            foreach (var interval in allDropIntervals.ToArray())
            {
                foreach (var sess in nkpoiSessions)
                {
                    if (isPeriodInPeriod(interval, sess.DropInterval)) // если этот интервал полностью в сессии, значит добавляем эту сессию в использованные
                    {
                        if (!sessions.Contains(sess)) // добавляем только если еще не добавили
                            sessions.Add(sess);
                        allDropIntervals.Remove(interval);
                        continue;
                    }
                }
            }

        }

        /// <summary>
        /// получить свободные промежутки времени из сессий связи
        /// </summary>
        /// <param name="sessions"> все сессии связи </param>
        /// <param name="occupiedPeriods"> промежутки времени, которые следует исключить из сессий связи </param>
        /// <returns> свободные промежутки времени </returns>
        public static List<Tuple<DateTime, DateTime>> getFreeTimePeriodsOfSessions(List<CommunicationSession> sessions, List<Tuple<DateTime, DateTime>> occupiedPeriods)
        {
            List<Tuple<DateTime, DateTime>> freeRangesForDrop = new List<Tuple<DateTime, DateTime>>();
            var compressedOccupiedPeriods = compressTimePeriods(occupiedPeriods);
            foreach (var session in sessions)
            {
                var timeSpan = session.DropInterval;
                //List<Tuple<DateTime, DateTime>> freeRangesForSession = getFreeTimeRanges(timeSpan, occupiedPeriods);
                List<Tuple<DateTime, DateTime>> freeRangesForSession = getFreeIntervals(compressedOccupiedPeriods, timeSpan.Item1, timeSpan.Item2);

                freeRangesForDrop.AddRange(freeRangesForSession);
            }
            freeRangesForDrop = compressTimePeriods(freeRangesForDrop);
            freeRangesForDrop = freeRangesForDrop.OrderByDescending(range => (range.Item2 - range.Item1).TotalSeconds).ToList();
            return freeRangesForDrop;
        }

        /// <summary>
        /// является ли период времнеи checkPeriod полностью принадлежащим одному из periods
        /// </summary>
        /// <param name="checkPeriod"></param>
        /// <param name="periods"></param>
        /// <returns></returns>
        public static bool isPeriodInPeriods(Tuple<DateTime, DateTime> checkPeriod, List<Tuple<DateTime, DateTime>> periods)
        {
            foreach (var period in periods)
            {
                if (isPeriodInPeriod(checkPeriod, period))
                    return true;
            }
            return false;
        }

        public static bool isPeriodInPeriod(Tuple<DateTime, DateTime> checkPeriod, Tuple<DateTime, DateTime> period)
        {
            if (period.Item1 <= checkPeriod.Item1 && checkPeriod.Item1 <= period.Item2
                    && period.Item1 <= checkPeriod.Item2 && checkPeriod.Item2 <= period.Item2)
                return true;
            else
                return false;
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="routesToDrop"></param>
        /// <param name="freeCompressedIntervals"></param>
        /// <returns> возвращает словарь *временной интервал / маршруты, помещенные в этот интервал*  </returns>
        public static Dictionary<Tuple<DateTime, DateTime>, List<RouteParams>> getRoutesParamsInIntervals(List<RouteMPZ> inpRoutes, List<Tuple<DateTime, DateTime>> freeCompressedIntervals, int workType, int startId)
        {
            Dictionary<Tuple<DateTime, DateTime>, List<RouteParams>> res = new Dictionary<Tuple<DateTime, DateTime>, List<RouteParams>>();
            List<RouteMPZ> routes = new List<RouteMPZ>(inpRoutes);

            // отсортируем свободные промежутки по продолжительности в порядке возрастания по прищнаку продолжительности
            freeCompressedIntervals.Sort(delegate(Tuple<DateTime, DateTime> span1, Tuple<DateTime, DateTime> span2)
                                     { return (span1.Item2 - span1.Item1).CompareTo(span2.Item2 - span2.Item1); });

            foreach (var interval in freeCompressedIntervals)
            {
                int id = startId + 1;
                // получим все маршруты, которые могут быть сброшены/удалены в этом промежутке
                List<RouteMPZ> curRoutes = routes.Where(rout => rout.Parameters.end < interval.Item2).ToList();

                if (curRoutes.Count == 0)
                    continue;

                curRoutes.Sort(delegate(RouteMPZ rout1, RouteMPZ rout2) { return rout1.Parameters.end.CompareTo(rout2.Parameters.end); });

                List<RouteParams> curPeriodRoutes = new List<RouteParams>();

                DateTime prevDt = curRoutes[0].Parameters.end > interval.Item1 ? curRoutes[0].Parameters.end : interval.Item1;
                foreach (var rmpz in curRoutes.ToArray())
                {
                    if (prevDt < rmpz.Parameters.end)
                        prevDt = rmpz.Parameters.end; // если текущее время раньше времени конца работы сбрасываемого/удаляемого маршрута, сдвигаем текущее время

                    double actionTime = 0;
                    if (workType == 1)
                        actionTime = rmpz.Parameters.getDropTime();
                    else if (workType == 2)
                        actionTime = OptimalChain.Constants.routeDeleteTime;

                    DateTime nextDt = prevDt.AddSeconds(actionTime);
                    if (nextDt > interval.Item2)
                        break; // если вышли за пределы текущего интервала, переходим к следующему интервалу

                    nextDt.AddMilliseconds(OptimalChain.Constants.min_Delta_time); // @todo точно ли эта дельта?

                    RouteParams curParam = new RouteParams(workType, prevDt, nextDt, Tuple.Create(rmpz.NPZ, rmpz.Nroute));
                    curParam.id = id;
                    curParam.ShootingConf = rmpz.Parameters.ShootingConf;
                    curPeriodRoutes.Add(curParam);
                    id++;
                    routes.Remove(rmpz);

                    prevDt = nextDt;
                }

                if (curPeriodRoutes.Count != 0)
                {
                    res[interval] = curPeriodRoutes;
                }

            }

            return res;
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
                    newConf.setPitchDependency(new Dictionary<double, Tuple<double, double>>(), timeDelta);
                    res.Add(newConf);
                }

                prevRouteInd = prevRouteInd + numRoutes;
            }
            return res;
        }


        public static List<Tuple<DateTime, DateTime>> compressTimePeriods(List<Tuple<DateTime, DateTime>> timePeriods)
        {
            // соеденим пересекающиеся диапазоны
            List<Tuple<DateTime, DateTime>> compressedSilentAndCaptureRanges = new List<Tuple<DateTime, DateTime>>(timePeriods);
            compressedSilentAndCaptureRanges.Sort(delegate(Tuple<DateTime, DateTime> span1, Tuple<DateTime, DateTime> span2) { return span1.Item1.CompareTo(span2.Item1); });
            List<Tuple<DateTime, DateTime>> res = new List<Tuple<DateTime, DateTime>>();

            for (int i = 0; i < compressedSilentAndCaptureRanges.Count; i++)
            {
                var curRange = compressedSilentAndCaptureRanges[i];
                for (int j = i + 1; j < compressedSilentAndCaptureRanges.Count; j++)
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

        public static List<Tuple<DateTime, DateTime>> getFreeIntervals(List<Tuple<DateTime, DateTime>> compressedOccupiedPeriods, DateTime timeFrom, DateTime timeTo)
        {
            compressedOccupiedPeriods.Sort(delegate(Tuple<DateTime, DateTime> span1, Tuple<DateTime, DateTime> span2) { return span1.Item1.CompareTo(span2.Item1); });

            List<Tuple<DateTime, DateTime>> res = new List<Tuple<DateTime, DateTime>>();

            DateTime firstDt = timeFrom;

            foreach (var timeSpan in compressedOccupiedPeriods)
            {
                if (timeSpan.Item2 < timeFrom)
                    continue;
                if (timeSpan.Item1 > timeTo)
                    break;
                if (firstDt < timeSpan.Item1)
                    res.Add(new Tuple<DateTime, DateTime>(firstDt, timeSpan.Item1));
                firstDt = timeSpan.Item2;
            }

            if (firstDt < timeTo)
                res.Add(new Tuple<DateTime, DateTime>(firstDt, timeTo));

            return res;
        }


        /// <summary>
        /// Создание ПНб по набору маршрутов
        /// </summary>
        /// <param name="routesParams"> Набор маршрутов RouteParams</param>
        /// <returns> набор МПЗ, созданный из маршутов</returns>
        /// <param name="Nmax">номер,с которого мпз следует нумеровать</param>
        /// <param name="managerDB">параметры взаимодействия с БД</param>
        /// <param name="flags">Флаги для настройки МПЗ</param>
        public static List<MPZ> createPNbOfRoutes(List<RouteParams> routesParams, int Nmax, DIOS.Common.SqlManager managerDB, FlagsMPZ flags = null)
        {
            List<MPZ> res = new List<MPZ>();
            List<MPZParams> mpzParams = MPZParams.FillMPZ(routesParams, Nmax);
            foreach (var param in mpzParams)
                res.Add(new MPZ(param, managerDB, flags ?? new FlagsMPZ()));

            return res;
        }


        /// <summary>
        /// Рассчитать коридор съемки/видимости для заданной конфигурации СОЭНc
        /// </summary>
        /// <param name="dateTime">Момент времени</param>
        /// <param name="rollAngle">Крен [рад]</param>
        /// <param name="pitchAngle">Тангаж [рад]</param>
        /// <param name="dist">Длина коридора [м] (не более 97000)</param>
        /// <param name="az">Азимут коридора [рад] </param>
        /// <param name="managerDB">Параметры подключения к БД</param>
        /// <param name="wktPoly">Коридор в формате WKT</param>
        /// <param name="duration">Длительность съемки коридора [с]</param>
        /// <returns> полигон в формате WKT</returns>
        public static void getCoridorPoly(DateTime dateTime, double rollAngle, double pitchAngle, double dist, double az, DIOS.Common.SqlManager managerDB,
            out string wktPoly, out double duration)
        {
            if (dist > 97e3)
                throw new ArgumentException("Coridor length cannot exceed 97km.");

            DataFetcher fetcher = new DataFetcher(managerDB);
            TrajectoryPoint? p0_ = fetcher.GetPositionSat(dateTime);

            double l1, l2, b1, b2, s1, s2, s3;
            TrajectoryRoutines.GetCoridorParams(
                fetcher, dateTime, az, dist,
                rollAngle, pitchAngle,
                out b1, out b2, out l1, out l2, out s1, out s2, out s3, out duration);

            SatelliteCoordinates kaPos = new SatelliteCoordinates(p0_.Value);
            kaPos.addRollPitchRot(rollAngle, pitchAngle);
            //LanePos lpBegin = new LanePos(p0_.Value, OptimalChain.Constants.camera_angle, rollAngle, pitchAngle);
            GeoPoint leftFirstPoint = GeoPoint.FromCartesian(kaPos.BotLeftViewPoint);
            GeoPoint rightFirstPoint = GeoPoint.FromCartesian(kaPos.BotRightViewPoint);
            // @todo доделать учёт kaPos.Top*ViewPoint

            GeoPoint[] leftPoints = new GeoPoint[10];
            for (int i = 0; i < leftPoints.Length; ++i)
            {
                double d = dist / leftPoints.Length * (i + 1);
                leftPoints[i] = new GeoPoint(
                    AstronomyMath.ToDegrees(AstronomyMath.ToRad(leftFirstPoint.Latitude) + b1 * d + b2 * d * d),
                    AstronomyMath.ToDegrees(AstronomyMath.ToRad(leftFirstPoint.Longitude) + l1 * d + l2 * d * d)
                );
            }
            GeoPoint[] rightPoints = new GeoPoint[10];
            for (int i = 0; i < rightPoints.Length; ++i)
            {
                double d = dist / rightPoints.Length * (rightPoints.Length - i);
                rightPoints[i] = new GeoPoint(
                    AstronomyMath.ToDegrees(AstronomyMath.ToRad(rightFirstPoint.Latitude) + b1 * d + b2 * d * d),
                    AstronomyMath.ToDegrees(AstronomyMath.ToRad(rightFirstPoint.Longitude) + l1 * d + l2 * d * d)
                );
            }

            List<GeoPoint> vertices = new List<GeoPoint>();
            vertices.Add(leftFirstPoint);
            vertices.AddRange(leftPoints);
            vertices.AddRange(rightPoints);
            vertices.Add(rightFirstPoint);

            Polygon pol = new Polygon(vertices);
            wktPoly = pol.ToWtk();
        }

        /// <summary>
        /// Рассчитать полигон съемки/видимости для заданной конфигурации СОЭН
        /// </summary>
        /// <param name="dateTime"> Момент времени DateTimec</param>
        /// <param name="rollAngle">Крен в радианах double</param>
        /// <param name="pitchAngle"> Тангаж в радианах double</param>
        /// <param name="duration">Продолжительность съемки в милисекундах</param>
        /// <param name="DBManager">Параметры подключения к БД</param>
        /// <param name="isCoridor">Флаг коридорной съемки</param>
        /// <returns> полигон в формате WKT</returns>
        public static string getSOENViewPolygon(DateTime dateTime, double rollAngle, double pitchAngle, int duration, DIOS.Common.SqlManager managerDB, bool isCoridor = false)
        {
            string wtk = "";
            DataFetcher fetcher = new DataFetcher(managerDB);

            if (!isCoridor)
            {
                if (duration == 0)
                {
                    TrajectoryPoint? point = fetcher.GetPositionSat(dateTime);

                    if (point == null)
                    {
                        return wtk;
                    }
                    SatelliteCoordinates kaPos = new SatelliteCoordinates((TrajectoryPoint)point, rollAngle, pitchAngle);                    
                    wtk = kaPos.ViewPolygon.ToWtk();
                }
                else
                {
                    DateTime timeTo = dateTime.AddMilliseconds(duration);
                    Trajectory trajectory = fetcher.GetTrajectorySat(dateTime, timeTo);
                    Polygon pol = SatLane.getRollPitchLanePolygon(trajectory, rollAngle, pitchAngle);
                    wtk = pol.ToWtk();

                    //if (viewLane.Sectors.Count > 0)
                    //{
                    //    List<Vector3D> leftLanePoints = new List<Vector3D>();
                    //    List<Vector3D> rightLanePoints = new List<Vector3D>();

                    //    for (int sectId = 0; sectId < viewLane.Sectors.Count; sectId++)
                    //    {
                    //        var sect = viewLane.Sectors[sectId];
                    //        int i = 0;
                    //        if (sectId > 0)
                    //            i = 1;
                    //        for (; i < sect.sectorPoints.Count; i++)
                    //        {
                    //            var pos = sect.sectorPoints[i];
                    //            leftLanePoints.Add(pos.LeftCartPoint);
                    //            rightLanePoints.Add(pos.RightCartPoint);
                    //        }
                    //    }
                    //    for (int i = rightLanePoints.Count - 1; i >= 0; i--)
                    //        leftLanePoints.Add(rightLanePoints[i]);

                    //    Polygon pol = new Polygon(leftLanePoints);
                    //    wtk = pol.ToWtk();
                    //}
                }
            }
            else
            {
                double dur;
                getCoridorPoly(dateTime, rollAngle, pitchAngle, 96e3, Math.PI / 6, managerDB, out wtk, out dur);
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
                            foreach (Polygon piece in p.BreakIntoLobes())
                                turnPartsLitAndNot.Add(new wktPolygonLit { wktPolygon = piece.ToWtk(), sun = true });
                        foreach (SphericalGeom.Polygon p in LitAndNot.Item2)
                            foreach (Polygon piece in p.BreakIntoLobes())
                                turnPartsLitAndNot.Add(new wktPolygonLit { wktPolygon = piece.ToWtk(), sun = false });
                    }
                }

                // If no more data on this turn, but we are on streak. Write it down
                if (streakBegin != -1)
                {
                    List<Polygon> pieces = SatelliteTrajectory.TrajectoryRoutines.FormSectorFromLanePoints(lane, streakBegin, lane.Count - 1).BreakIntoLobes();
                    foreach (Polygon piece in pieces)
                        turnPartsLitAndNot.Add(new wktPolygonLit
                        {
                            wktPolygon = piece.ToWtk(),
                            sun = onLitStreak
                        });
                }

                partsLitAndNot.Add(Tuple.Create(lanePart.Item1, turnPartsLitAndNot));
            }
        }




        /// <summary>
        /// Разбиение полосы видимости КА под траекторией на полигоны освещенности.
        /// </summary>
        /// <param name="DBManager">Параметры подключения к БД</param>
        /// <param name="timeFrom">Начало временного промежутка</param>
        /// <param name="timeTo">Конец временного промежутка</param>
        /// <param name="partsLitAndNot">Список объектов: номер витка и полигоны, помеченные флагом освещенности</param>
        public static void checkIfViewLaneIsLitWithTimeSpans(DIOS.Common.SqlManager DBManager, DateTime timeFrom, DateTime timeTo, out List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot, out List<Tuple<DateTime, DateTime>> shadowPeriods)
        {
            DataFetcher fetcher = new DataFetcher(DBManager);
            var laneParts = fetcher.GetViewLaneBrokenIntoTurns(timeFrom, timeTo);
            List<SpaceTime> sunPositions = fetcher.GetPositionSun(timeFrom, timeTo);
            partsLitAndNot = new List<Tuple<int, List<wktPolygonLit>>>();
            shadowPeriods = new List<Tuple<DateTime, DateTime>>();

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

                            if (!onLitStreak) // в тени
                            {
                                DateTime dtfrom = lane[streakBegin].Time;
                                DateTime dtto = lane[i].Time;
                                shadowPeriods.Add(Tuple.Create(dtfrom, dtto));
                            }

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
                            foreach (Polygon piece in p.BreakIntoLobes())
                                turnPartsLitAndNot.Add(new wktPolygonLit { wktPolygon = piece.ToWtk(), sun = true });
                        foreach (SphericalGeom.Polygon p in LitAndNot.Item2)
                            foreach (Polygon piece in p.BreakIntoLobes())
                                turnPartsLitAndNot.Add(new wktPolygonLit { wktPolygon = piece.ToWtk(), sun = false });
                    }
                }

                // If no more data on this turn, but we are on streak. Write it down
                if (streakBegin != -1)
                {
                    List<Polygon> pieces = SatelliteTrajectory.TrajectoryRoutines.FormSectorFromLanePoints(lane, streakBegin, lane.Count - 1).BreakIntoLobes();
                    foreach (Polygon piece in pieces)
                        turnPartsLitAndNot.Add(new wktPolygonLit
                        {
                            wktPolygon = piece.ToWtk(),
                            sun = onLitStreak
                        });
                    if (!onLitStreak) // в тени
                    {
                        DateTime dtfrom = lane[streakBegin].Time;
                        DateTime dtto = lane[lane.Count - 1].Time;
                        shadowPeriods.Add(Tuple.Create(dtfrom, dtto));
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

        // НИГДЕ НЕ ИСПОЛЬЗУЕТСЯ, НО ЮЗАЕТ УСТАРЕВШИЙ КОНСТРУКТОР МПЗ -- ЗАКОММЕНТИЛ ПОКА ЧТО
        ///// <summary>
        ///// Создание МПЗ по заданным маршрутам и доп.параметрам
        ///// </summary>
        ///// <param name="routes">Набор маршрутов </param>
        ///// <param name="PWR_ON">признак PWR_ON</param>
        ///// <param name="mpz">параметры МПЗ</param>
        ///// <param name="error">ошибка создания</param>
        //public static void createMPZ(List<RouteMPZ> routes, int PWR_ON, out MPZ mpz, out createMPZStatus error)
        //{
        //    mpz = new MPZ(routes);
        //    error = createMPZStatus.eSuccses;
        //}

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
            bool is7zoneSet = false;
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
                    is7zoneSet = true;
                }

                if (!in7zone && prevIn7zone) // вышли из семиградусной зоны (текущая точка не в семиградусной зоне, а предыдущая в семиградусной)
                {
                    tempSession.Zone7timeTo = getIntersectionTime(points[i - 1], point, centre, zone.Radius7);
                }

                if ((!in5zone || i == count) && prevIn5zone) // предыдущая точка в пятиградусной зоне, а текущая точка последняя или находится вне пятиградусной зоны
                {
                    tempSession.Zone5timeTo = getIntersectionTime(points[i - 1], point, centre, zone.Radius5);
                    tempSession.routesToDrop = new List<RouteMPZ>();
                    if (is7zoneSet)
                        sessions.Add(tempSession); // добавляем только если удалось установить и 7 зону тоже                    
                    tempSession = new CommunicationSession();
                    is7zoneSet = false;
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
            foreach (var sess in sessions)
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
        private static void calculatePitchArrays(CaptureConf conf, TrajectoryPoint pointFrom)
        {
            double pitchAngleLimit = conf.orders.Min(order => order.request.Max_SOEN_anlge);

            if (pitchAngleLimit > OptimalChain.Constants.max_pitch_angle)
                pitchAngleLimit = OptimalChain.Constants.max_pitch_angle;

            double maxPitchAngle = Math.Abs(pitchAngleLimit) - Math.Abs(conf.rollAngle);

            if (maxPitchAngle < 0) // такое возможно, если rollAngle больше (по модулю) 30 градусов (максимальны тангаж) 
                maxPitchAngle = 0;

            double timeDelta;
            if (0 == maxPitchAngle)
                timeDelta = 0;
            else
                timeDelta = getTimeDeltaFromPitch(pointFrom, conf.rollAngle, maxPitchAngle);

            conf.pitchArray[0] = Tuple.Create(0.0, 0.0);

            Dictionary<double, double> angleTimeArray = new Dictionary<double, double>();
            angleTimeArray[0] = 0;

            Vector3D dirRollPoint = LanePos.getSurfacePoint(pointFrom, conf.rollAngle, 0);
            int pitchStep = 1; // угол изменения тангажа в градусах.  
            for (int pitch_degr = pitchStep; pitch_degr <= AstronomyMath.ToDegrees(maxPitchAngle); pitch_degr += pitchStep)
            {
                double pitch = AstronomyMath.ToRad(pitch_degr);
                Vector3D dirPitchPoint = LanePos.getSurfacePoint(pointFrom, conf.rollAngle, pitch);
                double distOverSurf = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(dirPitchPoint), GeoPoint.FromCartesian(dirRollPoint)) * Astronomy.Constants.EarthRadius;
                double t = distOverSurf / pointFrom.Velocity.Length;
                angleTimeArray[pitch] = t;
            }

            LinearInterpolation pitchInterpolation = new LinearInterpolation(angleTimeArray.Values.ToArray(), angleTimeArray.Keys.ToArray());

            Dictionary<double, Tuple<double, double>> timeAngleArray = new Dictionary<double, Tuple<double, double>>();
            for (int t = 0; t <= (int)timeDelta; t++)
            {
                double pitch = pitchInterpolation.GetValue(t);
                double height = pointFrom.Position.ToVector().Length - Astronomy.Constants.EarthRadius;
                GeoPoint kaGeoPoint = GeoPoint.FromCartesian(pointFrom.Position.ToVector());
                var rollCorrection = getRollCorrection(height, pointFrom.Velocity.Length, AstronomyMath.ToRad(kaGeoPoint.Latitude), pitch);
                timeAngleArray[t] = Tuple.Create(pitch, rollCorrection);
            }

            conf.setPitchDependency(timeAngleArray, timeDelta);
        }




        public static double getTimeDeltaFromPitch(TrajectoryPoint pointFrom, double rollAngle, double pitchAngle)
        {
            Vector3D rollPoint = LanePos.getSurfacePoint(pointFrom, rollAngle, 0);
            Vector3D PitchRollPoint = LanePos.getSurfacePoint(pointFrom, rollAngle, pitchAngle);
            rollPoint.Normalize();
            PitchRollPoint.Normalize();
            // расстояние в километрах между точкой c нулевым тангажом и точкой, полученной при максимальном угле тангажа
            double dist = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(rollPoint), GeoPoint.FromCartesian(PitchRollPoint)) * Astronomy.Constants.EarthRadius;
            // время, за которое спутник преодалевает dist по поверхности земли.
            return Math.Abs(dist / pointFrom.Velocity.Length);
        }


        public static double getRollCorrection(double height, double velo, double bKa, double pitchAngle)
        {
            double wEarth = OptimalChain.Constants.earthRotSpeed;
            double I = OptimalChain.Constants.orbital_inclination;
            double R = Astronomy.Constants.EarthRadius;
            //double bm = b + Math.Sin(I) * (Math.Acos(Math.Sqrt(1 - (R + h) * (R + h) / R / R * Math.Sin(pitch) * Math.Sin(pitch))) - pitch);
            //double d = Math.Cos(bm) * w / v * pitch * Math.Sin(I);
            //double sinRoll = R * Math.Sin(d) / Math.Sqrt(R * R + (R + h) * (R + h) - 2 * R * (R + h) * Math.Cos(d));
            //return Math.Asin(sinRoll); 
            double bm = bKa + Math.Sin(I) * (Math.Acos(Math.Sqrt(1 - Math.Pow((R + height) / R * Math.Sin(pitchAngle), 2))) - pitchAngle);
            double d = Math.Cos(bm) * wEarth / velo * pitchAngle * Math.Sin(I);
            double sinRoll = R * Math.Sin(d) / Math.Sqrt(Math.Pow(R, 2) + Math.Pow(R + height, 2) - 2 * R * (R + height) * Math.Cos(d));
            return Math.Asin(sinRoll);
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
