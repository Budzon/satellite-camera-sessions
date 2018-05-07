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

            List<TimePeriod> shadowPeriods;
            List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot;
            checkIfViewLaneIsLitWithTimeSpans(managerDB, timeFrom, timeTo, out partsLitAndNot, out shadowPeriods);

            possibleConfs = getCaptureConfArray(
                new List<RequestParams>() { request },
                timeFrom,
                timeTo,
                managerDB,
                shadowPeriods,
                new List<TimePeriod>()
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

            List<SphericalGeom.Polygon> region = new List<Polygon>(request.polygons);
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
                            // @todo  по человечески нало бы...
                            // Часть непокрытого региона -- слишком тонкая/некорректная. Пропускаем....
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

         


        private static void getCaptureConfArrayForTrajectoryForCoridor(
           DIOS.Common.SqlManager managerDB,
           List<RequestParams> requests,
           Trajectory trajectory,
           List<CaptureConf> captureConfs,
           List<TimePeriod> freeSessionPeriodsForDrop,
           List<TimePeriod> capturePeriods)
        {
            if (requests.Count == 0)
                return;

            

            foreach (var req in requests)
            {
                if (req.polygons.Count != 1)
                    throw new System.ArgumentException("Coridor request can have only one polygon");

                double maxpitch = Math.Min(OptimalChain.Constants.max_pitch_angle, req.Max_SOEN_anlge);
                double maxroll = Math.Min(OptimalChain.Constants.max_roll_angle, req.Max_SOEN_anlge);

                Polygon reqpol = req.polygons.First();

                double viewAngle = Math.Min(req.Max_SOEN_anlge, OptimalChain.Constants.max_roll_angle);
                SatLane viewLane = new SatLane(trajectory, 0, viewAngle);

                List<CaptureConf> confs = viewLane.getCaptureConfs(req);

                confs = TimePeriod.compressTimePeriods<CaptureConf>(confs, 0);

                List<CoridorParams> allCoridors = new List<CoridorParams>();
                int eps = 1; // немного удлинним сегмент полосы для того, чтобы точно замести полигон закакза целиком
                foreach (var conf in confs)
                {
                    Polygon segment = viewLane.getSegment(conf.dateFrom.AddSeconds(-eps), conf.dateTo.AddSeconds(eps));

                    IList<Polygon> interpols = Polygon.Intersect(segment, reqpol);

                    foreach (var p in interpols)
                    {
                        List<GeoPoint> line = p.getCenterLine();                         
                        double deltaPitchTime = getTimeDeltaFromPitch(trajectory.GetPoint(conf.dateFrom), 0, maxpitch);
                        DateTime start = conf.dateFrom.AddSeconds(-deltaPitchTime);
                                                
                        while (start < conf.dateTo.AddSeconds(deltaPitchTime))
                        {
                            List<CoridorParams> coridorParams;
                            try
                            {
                                getPiecewiseCoridorParams(start, line, managerDB, out coridorParams, 0.5e5);
                                start = coridorParams.Max(cor => cor.EndTime);
                                allCoridors.AddRange(coridorParams);
                            }
                            catch(Exception e)
                            {
                            //    Console.WriteLine(e.Message);       
                                start = start.AddSeconds(20);
                            }
                            
                        }
                    }

                }

                allCoridors.RemoveAll(cor => cor.AbsMaxRequiredPitch > maxpitch);
                allCoridors.RemoveAll(cor => cor.AbsMaxRequiredRoll > maxroll);
                
                //allCoridors.RemoveAll(cor => SatelliteCoordinates ka = new SatelliteCoordinates(cor.AbsMaxRequiredRoll, cor.AbsMaxRequiredPitch);  Vector3D.AngleBetween(ka.ViewDir, ka.)   > req.Max_SOEN_anlge);

                foreach (var cp in allCoridors)
                { 
                    double interCoeff = cp.Coridor.Area / req.polygons.First().Area;
                    
                    var orders = new List<Order>() { new Order() { request = req, captured = cp.Coridor, intersection_coeff = interCoeff } };
                    int confType = 0;
                    if (req.compression == OptimalChain.Constants.compressionDropCapture)
                        confType = 3;
                    CaptureConf cc = new CaptureConf(cp.StartTime, cp.EndTime, cp.AbsMaxRequiredRoll, orders, confType, null, _poliCoef: cp.CoridorCoefs);
                    cc.setPolygon(cp.Coridor);
                    captureConfs.Add(cc);
                }

            }
 
        }
 

        private static void getCaptureConfArrayForTrajectoryForPlainReq(
        List<RequestParams> requests,
        Trajectory trajectory,
        List<CaptureConf> captureConfs,
        List<TimePeriod> freeSessionPeriodsForDrop,
        List<TimePeriod> capturePeriods)
        {
            if (requests.Count == 0)
                return;

            List<List<RequestParams>> breakingRequests = RequestParams.breakRequestsIntoGroups(requests); // разделим заказы на несовместимые подгруппы 

            double Max_SOEN_anlge = requests.Max(req => req.Max_SOEN_anlge); // не будем генерировать полос для угла, превышающего это значение
 
            double max_roll_angle = Math.Min(Max_SOEN_anlge, OptimalChain.Constants.max_roll_angle);
            double min_roll_angle = -max_roll_angle;

            double angleStep = OptimalChain.Constants.camera_angle; // шаг равен углу обзора
            int num_steps = (int)((max_roll_angle - min_roll_angle) / angleStep); /// @todo что делать с остатком от деления?

            ConcurrentBag<CaptureConf> concurrentlist = new ConcurrentBag<CaptureConf>();
            
#if _PARALLEL_
            Parallel.For(0, num_steps + 1, index =>
            {
                double rollAngle = min_roll_angle + index * angleStep;
#else
                for (double rollAngle = min_roll_angle; rollAngle <= max_roll_angle; rollAngle += angleStep)    {
#endif
                List<CaptureConf> laneCaptureConfs = new List<CaptureConf>(); // конфигурации захвата для текущий полосы захвата
                SatLane viewLane = new SatLane(trajectory, rollAngle, OptimalChain.Constants.camera_angle);

                foreach (var requestGroup in breakingRequests)
                {
                    List<CaptureConf> groupConfs = new List<CaptureConf>();

                    foreach (var request in requestGroup)
                    {
                        if (Math.Abs(rollAngle) > Math.Abs(request.Max_SOEN_anlge))
                            continue;
                        List<CaptureConf> confs = viewLane.getCaptureConfs(request);

                        if (confs.Count == 0)
                            continue;

                        // если сжатие заказа == 10, то для всех конифгураций, помещающихся в зону дейтвия НКПОИ мы выставляем режим "съемка со сбросом"
                        if (request.compression == OptimalChain.Constants.compressionDropCapture)
                        {
                            var confsToFropCapt = confs.Where(cc => TimePeriod.isPeriodInPeriods(new TimePeriod(cc.dateFrom, cc.dateTo), freeSessionPeriodsForDrop)).ToList();
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
                        groupConfs.AddRange(confs);
                    }

                    laneCaptureConfs.AddRange(CaptureConf.compressCConfArray(groupConfs));
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

 

        public static List<Trajectory> getLitTrajectoryParts(DateTime timeFrom, DateTime timeTo, DIOS.Common.SqlManager managerDB, List<TimePeriod> shadowPeriods)
        {
            DataFetcher fetcher = new DataFetcher(managerDB);
            DateTime firstDt = timeFrom;
            List<Trajectory> posiibleTrajectoryParts = new List<Trajectory>();
            foreach (var timeSpan in shadowPeriods)
            {
                if (firstDt < timeSpan.dateFrom)
                    posiibleTrajectoryParts.Add(fetcher.GetTrajectorySat(firstDt, timeSpan.dateFrom));
                firstDt = timeSpan.dateTo;
            }

            if (firstDt < timeTo)
                posiibleTrajectoryParts.Add(fetcher.GetTrajectorySat(firstDt, timeTo));

            return posiibleTrajectoryParts;
        }

        public static List<CaptureConf> getCaptureConfArray(
            List<RequestParams> requests,
            DateTime timeFrom,
            DateTime timeTo,
            DIOS.Common.SqlManager managerDB,
            List<TimePeriod> inactivityRanges,
            List<TimePeriod> freeSessionPeriodsForDrop)
        {
            if (requests.Count == 0)
                return new List<CaptureConf>();

            //string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "\\" + "trajectory_1day.dat";
            //Astronomy.Trajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, timeFrom, timeTo); // @todo временно
            DataFetcher fetcher = new DataFetcher(managerDB);

            inactivityRanges.Sort(delegate(TimePeriod span1, TimePeriod span2) { return span1.dateFrom.CompareTo(span2.dateFrom); });

            List<Trajectory> trajSpans = getLitTrajectoryParts(timeFrom, timeTo, managerDB, inactivityRanges);

            // периоды, во время которых можно проводить съемку.
            List<TimePeriod> capturePeriods = TimePeriod.getFreeIntervals(inactivityRanges, timeFrom, timeTo);
                         
            var requestCoridor = requests.Where(req => req.shootingType == 2).ToList();
            var requestNOTCoridor = requests.Where(req => req.shootingType != 2).ToList();

            List<CaptureConf> captureConfsPlain = new List<CaptureConf>();
            foreach (var trajectory in trajSpans)
                getCaptureConfArrayForTrajectoryForPlainReq(requestNOTCoridor, trajectory, captureConfsPlain, freeSessionPeriodsForDrop, capturePeriods);

            List<CaptureConf> captureConfsCoridor = new List<CaptureConf>();
            foreach (var trajectory in trajSpans)
                getCaptureConfArrayForTrajectoryForCoridor(managerDB, requestCoridor, trajectory, captureConfsCoridor, freeSessionPeriodsForDrop, capturePeriods);

            List<CaptureConf> captureConfs = new List<CaptureConf>(captureConfsPlain);
            captureConfs.AddRange(captureConfsCoridor);

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
        /// <param name="silentTimePeriods">Список интервалов времени , в которые нельзя сбрасывать данные в СНКПОИ и/или МНКПОИ</param>
        /// <param name="inactivityTimePeriods">Список интервалов, когда нельзя проводить съемку</param>
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

            List<TimePeriod> silentTimePeriods = silentRanges.Select(tuple => new TimePeriod(tuple.Item1, tuple.Item2)).ToList();
            List<TimePeriod> inactivityTimePeriods = inactivityRanges.Select(tuple => new TimePeriod(tuple.Item1, tuple.Item2)).ToList();

            List<CommunicationSession> snkpoiSessions = CommunicationSession.getAllSNKPOICommunicationSessions(timeFrom, timeTo, managerDB);
            List<CommunicationSession> mnkpoiSessions = CommunicationSession.getAllMNKPOICommunicationSessions(timeFrom, timeTo, managerDB);

            List<CommunicationSession> nkpoiSessions = new List<CommunicationSession>();
            nkpoiSessions.AddRange(snkpoiSessions);
            nkpoiSessions.AddRange(mnkpoiSessions);

            // временные периоды, во время которых можно проводить съемку со сбросом
            List<TimePeriod> freeSessionPeriodsForDrop = CommunicationSession.getFreeTimePeriodsOfSessions(nkpoiSessions, silentTimePeriods);

            List<TimePeriod> shadowPeriods;
            List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot;
            checkIfViewLaneIsLitWithTimeSpans(managerDB, timeFrom, timeTo, out partsLitAndNot, out shadowPeriods);

            // учёт освещённости
            List<TimePeriod> shadowAndInactivityPeriods = new List<TimePeriod>();
            shadowAndInactivityPeriods.AddRange(inactivityTimePeriods);
            shadowAndInactivityPeriods.AddRange(shadowPeriods);
            shadowAndInactivityPeriods = TimePeriod.compressTimePeriods(shadowAndInactivityPeriods);

            // расчёт всех возможных конфигураций съемки на этот период с учётом ограничений
            List<CaptureConf> confsToCapture = getCaptureConfArray(requests, timeFrom, timeTo, managerDB, shadowAndInactivityPeriods, freeSessionPeriodsForDrop);

            // поиск оптимального набора маршрутов среди всех возможных конфигураций
            Graph captureGraph = new Graph(confsToCapture);
            List<MPZParams> captureMPZParams = captureGraph.findOptimalChain(Nmax);

            // Найдём все возможные промежутки времени для сброса (из диапазона [timeFrom - timeTo] вычитаются все inactivityRanges и диапазоны съемки)
            List<TimePeriod> captureIntervals = captureMPZParams.Select(mpz => new TimePeriod(mpz.start, mpz.end)).ToList();
            List<TimePeriod> silentAndCaptureRanges = new List<TimePeriod>();
            silentAndCaptureRanges.AddRange(silentTimePeriods);
            silentAndCaptureRanges.AddRange(captureIntervals);
            silentAndCaptureRanges = TimePeriod.compressTimePeriods(silentAndCaptureRanges);
            List<TimePeriod> freeRangesForDrop = CommunicationSession.getFreeTimePeriodsOfSessions(nkpoiSessions, silentAndCaptureRanges);

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

            Dictionary<TimePeriod, List<RouteParams>> dropRoutesParamsByIntervals = TimePeriod.getRoutesParamsInIntervals(allRoutesToDrop, freeRangesForDrop, workType: 1, startId: maxRoutesNumber);

            foreach (var intervalRoutes in dropRoutesParamsByIntervals)
            {
                List<RouteParams> routparamsList = intervalRoutes.Value;
                TimePeriod interval = intervalRoutes.Key;

                // попробуем вместить созданные маршрты на сброс в уже созданные маршруты на съемку
                foreach (var routparams in routparamsList.ToArray())
                {
                    foreach (var capPrms in captureMPZParams)
                    {
                        if (capPrms.InsertRoute(routparams, interval.dateFrom, interval.dateTo, null, null))
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
                // TimePeriod interval = intervalRoutes.Key;
                if (routparamsList.Count > 0)
                {
                    int curMaxMpzNum = dropMpzParams.Select(mpzparam => mpzparam.id).DefaultIfEmpty(maxMpzNum).Max();
                    List<MPZParams> curMPZ = MPZParams.FillMPZ(routparamsList, curMaxMpzNum);
                    dropMpzParams.AddRange(curMPZ);
                }
            }

            List<TimePeriod> dropIntervals = dropMpzParams.Select(mpzparams => new TimePeriod(mpzparams.start, mpzparams.end)).ToList();
            List<TimePeriod> inactivityDropCaptureIntervals = new List<TimePeriod>();
            inactivityDropCaptureIntervals.AddRange(captureIntervals);
            inactivityDropCaptureIntervals.AddRange(dropIntervals);
            inactivityDropCaptureIntervals.AddRange(inactivityTimePeriods);
            inactivityDropCaptureIntervals = TimePeriod.compressTimePeriods(inactivityDropCaptureIntervals);
            List<TimePeriod> freeRangesForDelete = TimePeriod.getFreeIntervals(inactivityDropCaptureIntervals, timeFrom, timeTo);
            
            maxRouteDropId = dropMpzParams.SelectMany(mpz => mpz.routes).Select(route => route.id).DefaultIfEmpty(0).Max();
            maxCaptureRouteId = captureMPZParams.SelectMany(mpz => mpz.routes).Select(route => route.id).DefaultIfEmpty(0).Max();

            maxRoutesNumber = Math.Max(maxRouteDropId, Math.Max(maxRouteDeleteId, maxCaptureRouteId));
            Dictionary<TimePeriod, List<RouteParams>> deleteRoutesParamsByIntervals = TimePeriod.getRoutesParamsInIntervals(routesToDelete, freeRangesForDelete, workType: 2, startId: maxRoutesNumber);

            maxMpzNum = dropMpzParams.Select(mpzparam => mpzparam.id).DefaultIfEmpty(maxMpzNum).Max();

            // создаем новые мпз удаления из маршрутов на удаление
            List<MPZParams> deleteMpzParams = new List<MPZParams>();
            foreach (var intervalRoutes in deleteRoutesParamsByIntervals)
            {
                List<RouteParams> routparamsList = intervalRoutes.Value;
                //TimePeriod interval = intervalRoutes.Key;
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

            List<TimePeriod> allDropIntervals = new List<TimePeriod>(); // все использованные интервалы связи (на сброс и на *съемку и сброс*)
            allDropIntervals.AddRange(dropMpzParams.SelectMany(mpz => mpz.routes).Select(route => new TimePeriod(route.start, route.end)));

            var allRoutesLists = captureMPZParams.Select(mpz => mpz.routes).ToList();
            List<RouteParams> allRoutes = new List<RouteParams>();
            foreach (var r in allRoutesLists)
                allRoutes.AddRange(r);

            var droproutes = allRoutes.Where(rout => (rout.type == 1 || rout.type == 3)).ToList();

            allDropIntervals.AddRange(droproutes.Select(rout => new TimePeriod(rout.start, rout.end)));

            foreach (var interval in allDropIntervals.ToArray())
            {
                foreach (var sess in nkpoiSessions)
                {
                    if (TimePeriod.isPeriodInPeriod(interval, sess.DropInterval)) // если этот интервал полностью в сессии, значит добавляем эту сессию в использованные
                    {
                        if (!sessions.Contains(sess)) // добавляем только если еще не добавили
                            sessions.Add(sess);
                        allDropIntervals.Remove(interval);
                        continue;
                    }
                }
            }

        }


        public static List<CaptureConf> getConfsToDrop(List<RouteMPZ> routesToDrop, List<TimePeriod> freeRanges)
        {
            List<CaptureConf> res = new List<CaptureConf>();

            double summFreeTime = freeRanges.Sum(range => (range.dateTo - range.dateFrom).TotalSeconds);

            int prevRouteInd = 0;
            foreach (var range in freeRanges)
            {
                if (routesToDrop.Count == res.Count) // все конифгурации созданы
                    break;

                double curTime = (range.dateTo - range.dateFrom).TotalSeconds;

                int numRoutes;
                if (freeRanges.Count > routesToDrop.Count)
                    numRoutes = 1;
                else
                    numRoutes = (int)(routesToDrop.Count * curTime / summFreeTime);

                double sumDropTime = routesToDrop.Sum(route => route.Parameters.getDropTime());

                DateTime spanCentre = range.dateFrom.AddSeconds(curTime / 2); // середина отрезка
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
                    double timeDelta = Math.Min((dropTimeCentre - range.dateFrom).TotalSeconds, (range.dateTo - dropTimeCentre).TotalSeconds);
                    newConf.setPitchDependency(new Dictionary<double, Tuple<double, double>>(), timeDelta);
                    res.Add(newConf);
                }

                prevRouteInd = prevRouteInd + numRoutes;
            }
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

        public static Trajectory getMaxTrajectory(DIOS.Common.SqlManager managerDB, DateTime start)
        {
            DataFetcher fetcher = new DataFetcher(managerDB);
            return fetcher.GetTrajectorySat(start, start.AddSeconds(1000)); // максимальная длительность маршрута
        }

        public static void getPieciwiseCoridor(DateTime dateTime, List<GeoPoint> vertices, DIOS.Common.SqlManager managerDB, out List<string> wkts, /*out List<GeoPoint> satPos,*/ double maxSubCurveLengthMeters = 1.5e5/*, bool custom = false*/)
        {
            List<CoridorParams> coridorParams;
            getPiecewiseCoridorParams(dateTime, vertices, managerDB, out coridorParams, maxSubCurveLengthMeters);
            wkts = new List<string>();

            foreach (var cp in coridorParams)
            {
                wkts.Add(cp.Coridor.ToWtk());
            }

            //GeoPoint curr, next;
            //TrajectoryPoint tp;
            //double roll, pitch, duration, dist;
            //DateTime now = dateTime;
            //string wkt;

            //Trajectory traj = getMaxTrajectory(managerDB, dateTime);
            //CoridorParams coridorParams;
            //var curves = new Curve(vertices).BreakByCurvatureAndDistance(1.5e5);
            //if (!custom)
            //{
            //    //for (int i = 0; i < curves.Count; ++i)
            //    //{
            //    //    curr = curves[i][0];
            //    //    next = curves[i][curves[i].Count - 1];
            //    //    tp = traj.GetPoint(now);

            //    //    //Routines.GetRollPitch(tp, curr, out roll, out pitch);
            //    //    //getCoridorPoly(now, curr, next, managerDB, out wkt, out duration, out dist);
            //    //    //getCoridorPoly(now, curr, next, managerDB, out wkt, out duration, out dist);
            //    //    wkts.Add(wkt);
            //    //    satPos.Add(GeoPoint.FromCartesian(tp.Position.ToVector()));
            //    //    now = now.AddSeconds(duration); // + 12
            //    //}
            //}
            //else
            //{
            //    for (int i = 0; i < curves.Count; ++i)
            //    {
            //        tp = traj.GetPoint(now);
            //        getCustomCoridor(traj, now, curves[i], out coridorParams);
            //        wkts.Add(coridorParams.Coridor.ToWtk());
            //        satPos.Add(GeoPoint.FromCartesian(tp.Position.ToVector()));
            //        now = now.AddSeconds(coridorParams.Duration);
            //    }
            //}
        }

        public static void getPiecewiseCoridorParams(DateTime dateTime, List<GeoPoint> vertices, DIOS.Common.SqlManager managerDB, out List<CoridorParams> coridorParams, double maxSubCurveLengthMeters = 1.5e5)
        {
            DateTime now = dateTime;
            Trajectory traj = getMaxTrajectory(managerDB, dateTime);

            var curves = new Curve(vertices).BreakByCurvatureAndDistance(maxSubCurveLengthMeters);
            coridorParams = new List<CoridorParams>();
            CoridorParams oneParam;

            for (int i = 0; i < curves.Count; ++i)
            {
                getCustomCoridor(traj, now, curves[i], out oneParam);
                now = now.AddSeconds(oneParam.Duration);
                coridorParams.Add(oneParam);
            }
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
            //TrajectoryPoint? p0_ = fetcher.GetPositionSat(dateTime);
            Trajectory traj = getMaxTrajectory(managerDB, dateTime);

            double l1, l2, b1, b2, s1, s2, s3;
            TrajectoryRoutines.GetCoridorParams(
                traj, dateTime, az, dist,
                rollAngle, pitchAngle,
                out b1, out b2, out l1, out l2, out s1, out s2, out s3, out duration);

            CoridorParams coridorParams = new CoridorParams(l1, l2, b1, b2, s1, s2, s3, 0, rollAngle, pitchAngle, dateTime, dateTime.AddSeconds(duration));
            coridorParams.ComputeCoridorPolygon(traj);
            wktPoly = coridorParams.Coridor.ToWtk();
        }

        public static void getCoridorPoly(DateTime dateTime, double rollAngle, double pitchAngle, GeoPoint end, DIOS.Common.SqlManager managerDB,
            out string wktPoly, out double duration, out double dist)
        {
            DataFetcher fetcher = new DataFetcher(managerDB);
            //TrajectoryPoint? p0_ = fetcher.GetPositionSat(dateTime);
            Trajectory traj = getMaxTrajectory(managerDB, dateTime);

            double l1, l2, b1, b2, s1, s2, s3;
            TrajectoryRoutines.GetCoridorParams(
                traj, dateTime, rollAngle, pitchAngle,
                end,
                out b1, out b2, out l1, out l2, out s1, out s2, out s3, out duration, out dist);

            CoridorParams coridorParams = new CoridorParams(l1, l2, b1, b2, s1, s2, s3, 0, rollAngle, pitchAngle, dateTime, dateTime.AddSeconds(duration));
            coridorParams.ComputeCoridorPolygon(traj);
            wktPoly = coridorParams.Coridor.ToWtk();
        }

        public static void getCoridorPoly(DateTime dateTime, GeoPoint start, GeoPoint end, DIOS.Common.SqlManager managerDB,
            out string wktPoly, out double duration, out double dist)
        {
            DataFetcher fetcher = new DataFetcher(managerDB);
            //TrajectoryPoint? p0_ = fetcher.GetPositionSat(dateTime);
            Trajectory traj = getMaxTrajectory(managerDB, dateTime);

            double l1, l2, b1, b2, s1, s2, s3, roll, pitch;
            TrajectoryRoutines.GetCoridorParams(
                traj, dateTime, start, end,
                out b1, out b2, out l1, out l2, out s1, out s2, out s3, out duration, out dist, out roll, out pitch);

            CoridorParams coridorParams = new CoridorParams(l1, l2, b1, b2, s1, s2, s3, 0, roll, pitch, dateTime, dateTime.AddSeconds(duration));
            coridorParams.ComputeCoridorPolygon(traj);
            wktPoly = coridorParams.Coridor.ToWtk();
        }

        /// <summary>
        /// Assumes positive(negative) curvature of the curve
        /// </summary>
        /// <param name="dateTime"></param>
        /// <param name="curve"></param>
        /// <param name="managerDB"></param>
        /// <param name="wktPoly"></param>
        /// <param name="duration"></param>
        /// <param name="dist"></param>
        public static void getCustomCoridor(Trajectory traj, DateTime startTime, Curve curve, out CoridorParams coridorParams)
        {
            //DataFetcher fetcher = new DataFetcher(managerDB);
            //TrajectoryPoint? p0_ = fetcher.GetPositionSat(dateTime);
            TrajectoryRoutines.GetCustomCoridorParams(traj, startTime, curve, out coridorParams);
            coridorParams.ComputeCoridorPolygon(traj);
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
        public static void checkIfViewLaneIsLitWithTimeSpans(DIOS.Common.SqlManager DBManager, DateTime timeFrom, DateTime timeTo, out List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot, out List<TimePeriod> shadowPeriods)
        {
            DataFetcher fetcher = new DataFetcher(DBManager);
            var laneParts = fetcher.GetViewLaneBrokenIntoTurns(timeFrom, timeTo);
            List<SpaceTime> sunPositions = fetcher.GetPositionSun(timeFrom, timeTo);
            partsLitAndNot = new List<Tuple<int, List<wktPolygonLit>>>();
            shadowPeriods = new List<TimePeriod>();

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
                    if (i == 4)
                    {
                        int a = 2;
                        a += 3;
                    }
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
                                shadowPeriods.Add(new TimePeriod(dtfrom, dtto));
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
                        shadowPeriods.Add(new TimePeriod(dtfrom, dtto));
                    }
                }

                partsLitAndNot.Add(Tuple.Create(lanePart.Item1, turnPartsLitAndNot));
            }
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
                double velo = pointFrom.Velocity.Length / pointFrom.Position.ToVector().Length;
                GeoPoint kaGeoPoint = GeoPoint.FromCartesian(pointFrom.Position.ToVector());
                var rollCorrection = getRollCorrection(height, velo, AstronomyMath.ToRad(kaGeoPoint.Latitude), pitch);
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


        /// <summary>
        /// расчёт поправки по крену
        /// </summary>
        /// <param name="height">высота ка в км</param>
        /// <param name="velo">скорость подспутниковой точки в радианах</param>
        /// <param name="bKa">широта подспутниковой точки в радианах </param>
        /// <param name="pitchAngle">угол тангажа</param>
        /// <returns>поправка по крену</returns>
        public static double getRollCorrection(double height, double velo, double bKa, double pitch)
        {
            double wEarth = OptimalChain.Constants.earthRotSpeed;
            double I = OptimalChain.Constants.orbital_inclination;
            double R = Astronomy.Constants.EarthRadius;
            double bm = bKa + Math.Sin(I) * (Math.Acos(Math.Sqrt(1 - Math.Pow((R + height) / R * Math.Sin(pitch), 2))) - pitch);
            //Разница между двумя позициями спутника
            double b2 = Math.Acos(Math.Sqrt(1 - Math.Pow((R + height) / R * Math.Sin(pitch), 2))) - Math.Abs(pitch);
            double d = Math.Cos(bm) * wEarth / velo * b2 * Math.Sin(I);
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
    } */

    public class wktPolygonLit
    {
        public string wktPolygon { get; set; }
        public bool sun { get; set; }
    }
}
