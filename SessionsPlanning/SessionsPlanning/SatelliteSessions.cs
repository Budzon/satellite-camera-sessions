#define  _PARALLEL_

using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using System.Collections.Concurrent;
using SatelliteTrajectory;
using Astronomy;
using Common;
using OptimalChain;
using DBTables;

using SphericalGeom;
using SessionsPlanning;

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
        public static void testMpzFormation(DIOS.Common.SqlManager managerDB, DIOS.Common.SqlManager managerDbCUKS, out List<MPZ> mpzs)
        {
            List<RouteParams> param = new List<RouteParams>();
            OptimalChain.StaticConf conf;

            ShootingChannel[] chan = new ShootingChannel[3] { ShootingChannel.pk, ShootingChannel.mk, ShootingChannel.cm };
            WorkingType[] regime = new WorkingType[4] { WorkingType.Shooting, WorkingType.Downloading, WorkingType.Removal, WorkingType.ShootingSending }; // Zi, Vi, Si, Np
            ShootingType[] shooting = new ShootingType[4] { ShootingType.Normal, ShootingType.StereoTriplet, ShootingType.StereoTriplet, ShootingType.Coridor }; // прост, стерео, коридор

            int[] compression = new int[5] { 0, 1, 2, 7, 10 };
            DateTime from = new DateTime(2019, 1, 5);
            DateTime to = from.AddSeconds(5);

            int k = 0;
            foreach (ShootingChannel ch in chan)
                foreach (WorkingType r in regime)
                    foreach (ShootingType s in shooting)
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
                            p.binded_route = null; // @todo чего тут я не знаю
                            param.Add(p);
                            from = to.AddSeconds(70);
                            to = from.AddSeconds(5);
                            k++;
                        }
            var mpzParams = OptimalChain.MPZParams.FillMPZ(param);
            FlagsMPZ flags = new FlagsMPZ();
            mpzs = mpzParams.Select(p => new MPZ(p, managerDB, managerDbCUKS, flags)).ToList();
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
        public static void isRequestFeasible(
            RequestParams request,
            DateTime timeFrom,
            DateTime timeTo,
            string conStringCUKS,
            string conStringCUP,
            out double coverage,
            out List<CaptureConf> possibleConfs)
        {
            DIOS.Common.SqlManager managerDbCUP = new DIOS.Common.SqlManager(conStringCUP);
            DIOS.Common.SqlManager managerDbCUKS = new DIOS.Common.SqlManager(conStringCUKS);
            DataFetcher fetcher = new DataFetcher(managerDbCUP);

            List<TimePeriod> shadowPeriods;// = new List<TimePeriod>();
            List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot;// = new List<Tuple<int,List<wktPolygonLit>>>();  
            checkIfViewLaneIsLitWithTimeSpans(managerDbCUP, timeFrom, timeTo, out partsLitAndNot, out shadowPeriods);
            possibleConfs = getCaptureConfArray(
                new List<RequestParams>() { request },
                timeFrom,
                timeTo,
                managerDbCUP,
                managerDbCUKS,
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

            List<Trajectory> possibleTrajParts = getLitTrajectoryParts(timeFrom, timeTo, managerDbCUP, shadowPeriods);
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
                            // @todo  по человечески надо бы...
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
           Trajectory sunTrajectory,
           List<CaptureConf> captureConfs,
           List<TimePeriod> freeSessionPeriodsForDrop,
           List<TimePeriod> capturePeriods,
           List<CloudinessData> meteoList)
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

                double maxRoll = Math.Min(OptimalChain.Constants.max_roll_angle, req.Max_SOEN_anlge);
                double viewAngle = maxRoll * 2 + OptimalChain.Constants.camera_angle;

                SatLane viewLane = new SatLane(trajectory, 0, viewAngle);

                List<CaptureConf> confs = viewLane.getCaptureConfs(req);

                confs = TimePeriod.compressTimePeriods<CaptureConf>(confs, 0);

                List<CoridorParams> allCoridors = new List<CoridorParams>();

                foreach (var conf in confs)
                {
                    int startEps = 1, endEps = 1; // если возможно, то немного удлинним сегмент полосы для того, чтобы точно замести полигон заказа целиком 
                    if (conf.dateFrom.AddSeconds(-startEps) < viewLane.FromDt)
                        startEps = 0;
                    if (conf.dateTo.AddSeconds(endEps) > viewLane.ToDt)
                        endEps = 0;

                    Polygon segment = viewLane.getSegment(conf.dateFrom.AddSeconds(-startEps), conf.dateTo.AddSeconds(endEps));

                    List<Polygon> interpols = Polygon.Intersect(segment, reqpol);

                    foreach (var p in interpols)
                    {
                        List<GeoPoint> line = p.getCenterLine();
                         
                        if (line.Count < 2)
                            continue;
                        double deltaPitchTime = CaptureConf.getTimeDeltaFromPitch(trajectory.GetPoint(conf.dateFrom), 0, maxpitch);
                        DateTime start = conf.dateFrom.AddSeconds(-deltaPitchTime);

                        while (start < conf.dateTo.AddSeconds(deltaPitchTime))
                        {
                            List<CoridorParams> coridorParams;
                            try
                            {
                                getPiecewiseCoridorParams(start, line, managerDB, out coridorParams);
                                start = coridorParams.Max(cor => cor.EndTime);
                                //  coridorParams.RemoveAll(cor => cor.AbsMaxRequiredPitch > maxpitch);
                                //  coridorParams.RemoveAll(cor => cor.AbsMaxRequiredRoll > maxroll);
                                if (coridorParams.Count != 0)
                                {
                                    allCoridors.AddRange(coridorParams);
                                    break;
                                }
                                else
                                {
                                    start = start.AddSeconds(20);
                                }
                            }
                            catch (Curve.NotEnougPointsException e)
                            {
                                // Console.WriteLine(e.Message);
                                start = start.AddSeconds(20);
                            }
                        }
                    }
                }
                 
                //allCoridors.RemoveAll(cor => SatelliteCoordinates ka = new SatelliteCoordinates(cor.AbsMaxRequiredRoll, cor.AbsMaxRequiredPitch);  Vector3D.AngleBetween(ka.ViewDir, ka.)   > req.Max_SOEN_anlge);

                foreach (var cp in allCoridors)
                {
                    double interCoeff = cp.Coridor.Area / req.polygons.First().Area;

                    var orders = new List<Order>() { new Order() { request = req, captured = cp.Coridor, intersection_coeff = interCoeff } };
                    WorkingType confType = WorkingType.Shooting;
                    if (req.compression == OptimalChain.Constants.compressionDropCapture)
                        confType = WorkingType.ShootingSending;

                    CaptureConf cc = new CaptureConf(cp.StartTime, cp.EndTime, cp.AbsMaxRequiredRoll, orders, confType, null, _poliCoef: cp.CoridorCoefs);

                    // проверим, не превышена ли облачность в месте съемки
                    bool isCloudiness = false;
                    foreach (CloudinessData cloud in meteoList)
                    {
                        if (CloudinessData.isCapConfInCloud(cc, cloud))
                        {
                            isCloudiness = true;
                            break;
                        }
                    }

                    if (isCloudiness)
                        continue;

                    // зададим угол солнца
                    if (cc.dateFrom == cc.dateTo)
                    {
                        TrajectoryPoint pointFrom = trajectory.GetPoint(cc.dateFrom);
                        cc.setSun(pointFrom.Position.ToVector(), sunTrajectory.GetPosition(cc.dateFrom).ToVector());
                    }
                    else
                    {
                        DateTime midTime;
                        midTime = cc.dateFrom.AddSeconds((cc.dateTo - cc.dateFrom).TotalSeconds);
                        cc.setSun(trajectory.GetPosition(midTime).ToVector(), sunTrajectory.GetPosition(midTime).ToVector());
                    }

                    cc.setPolygon(cp.Coridor);
                    captureConfs.Add(cc);
                }

            }

        }


        private static void getCaptureConfArrayForTrajectoryForPlainReq(
            List<RequestParams> requests,
            Trajectory trajectory,
            Trajectory sunTrajectory,
            List<CaptureConf> captureConfs,
            List<TimePeriod> freeSessionPeriodsForDrop,
            List<TimePeriod> capturePeriods,
            List<CloudinessData> meteoList)
        {
            if (requests.Count == 0)
                return;

            List<List<RequestParams>> breakingRequests = RequestParams.breakRequestsIntoGroups(requests); // разделим заказы на несовместимые подгруппы 

            double Max_SOEN_anlge = requests.Max(req => req.Max_SOEN_anlge); // не будем генерировать полос для угла, превышающего это значение

            double max_roll_angle = Math.Min(Max_SOEN_anlge, OptimalChain.Constants.max_roll_angle);
            double min_roll_angle = -max_roll_angle;

            double angleStep = OptimalChain.Constants.camera_angle * (1 - OptimalChain.Constants.stripOverlap); // шаг равен углу обзора
            int num_steps = (int)((max_roll_angle - min_roll_angle) / angleStep); /// @todo что делать с остатком от деления?

            ConcurrentBag<CaptureConf> concurrentlist = new ConcurrentBag<CaptureConf>();

#if _PARALLEL_
            Parallel.For(0, num_steps + 1, index =>
            {
                double rollAngle = min_roll_angle + index * angleStep;
#else
            for (double rollAngle = min_roll_angle; rollAngle <= max_roll_angle; rollAngle += angleStep)
            {
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

                        //отбросим конфигурации, попавшие в облачные участки                        
                        for (int i = 0; i < confs.Count; i++)
                        {
                            foreach (CloudinessData cloud in meteoList)
                            {
                                if (!CloudinessData.isCapConfInCloud(confs[i], cloud))
                                    continue;

                                confs.RemoveAt(i);
                                i--;
                            }
                        }

                        // если сжатие заказа == compressionDropCapture, то для всех конифгураций, помещающихся в зону дейтвия НКПОИ мы выставляем режим "съемка со сбросом"
                        if (request.compression == OptimalChain.Constants.compressionDropCapture)
                        {
                            var confsToFropCapt = confs.Where(cc => TimePeriod.isPeriodInPeriods(new TimePeriod(cc.dateFrom, cc.dateTo), freeSessionPeriodsForDrop)).ToList();
                            foreach (var conf in confsToFropCapt)
                                conf.confType = WorkingType.ShootingSending;
                        }

                        // если заказ - стерео, то пробуем его снять в стерео.
                        if (ShootingType.StereoTriplet == request.shootingType || ShootingType.Stereo == request.shootingType)
                        {
                            for (int i = 0; i < confs.Count; i++)
                            {
                                TrajectoryPoint pointFrom = trajectory.GetPoint(confs[i].dateFrom);
                                confs[i].converToStereo(pointFrom, capturePeriods, request.shootingType);
                            }
                        }
                        groupConfs.AddRange(confs);
                    }

                    laneCaptureConfs.AddRange(CaptureConf.compressCConfArray(groupConfs));
                }

                foreach (var conf in laneCaptureConfs)
                {
                    TrajectoryPoint pointFrom = trajectory.GetPoint(conf.dateFrom);
                    Polygon pol;
                    if (conf.dateFrom == conf.dateTo)
                        pol = new SatelliteCoordinates(pointFrom, rollAngle, 0).ViewPolygon;
                    else
                        pol = viewLane.getSegment(conf.dateFrom, conf.dateTo);

                    conf.setPolygon(pol);
                    if (conf.pitchArray.Count == 0) // если уже не рассчитали (в случае стереосъемки)
                        conf.calculatePitchArrays(pointFrom);

                    if (conf.dateFrom == conf.dateTo)
                    {
                        conf.setSun(pointFrom.Position.ToVector(), sunTrajectory.GetPosition(conf.dateFrom).ToVector());
                    }
                    else
                    {
                        DateTime midTime;
                        midTime = conf.dateFrom.AddSeconds((conf.dateTo - conf.dateFrom).TotalSeconds);
                        conf.setSun(trajectory.GetPosition(midTime).ToVector(), sunTrajectory.GetPosition(midTime).ToVector());
                    }
                }
                foreach (var conf in laneCaptureConfs)
                    concurrentlist.Add(conf);
            }
#if _PARALLEL_
);
#endif

            captureConfs.AddRange(concurrentlist.ToList());
        }

        public static List<Trajectory> getLitTrajectoryParts(
           DateTime timeFrom,
           DateTime timeTo,
           DIOS.Common.SqlManager managerDB,
           List<TimePeriod> shadowPeriods)
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
            DIOS.Common.SqlManager managerDbCUP,
            DIOS.Common.SqlManager managerDbCUKS,
            List<TimePeriod> inactivityRanges,
            List<TimePeriod> freeSessionPeriodsForDownoad)
        {
            if (requests.Count == 0)
                return new List<CaptureConf>();

            inactivityRanges.Sort(delegate(TimePeriod span1, TimePeriod span2) { return span1.dateFrom.CompareTo(span2.dateFrom); });

            Trajectory sunTrajectory = new DataFetcher(managerDbCUP).GetTrajectorySun(timeFrom, timeTo);
            List<Trajectory> trajSpans = getLitTrajectoryParts(timeFrom, timeTo, managerDbCUP, inactivityRanges);

            // периоды, во время которых можно проводить съемку.
            List<TimePeriod> capturePeriods = TimePeriod.getFreeIntervals(inactivityRanges, timeFrom, timeTo);

            var requestCoridor = requests.Where(req => req.shootingType == ShootingType.Coridor).ToList();
            var requestNOTCoridor = requests.Where(req => req.shootingType != ShootingType.Coridor).ToList();

            List<CloudinessData> meteoList = new DataFetcher(managerDbCUKS).GetMeteoData(timeFrom, timeTo); // облачность

            List<CaptureConf> captureConfsPlain = new List<CaptureConf>();
            foreach (var trajectory in trajSpans)
                getCaptureConfArrayForTrajectoryForPlainReq(requestNOTCoridor, trajectory, sunTrajectory, captureConfsPlain, freeSessionPeriodsForDownoad, capturePeriods, meteoList);

            List<CaptureConf> captureConfsCoridor = new List<CaptureConf>();
            foreach (var trajectory in trajSpans)
                getCaptureConfArrayForTrajectoryForCoridor(managerDbCUP, requestCoridor, trajectory, sunTrajectory, captureConfsCoridor, freeSessionPeriodsForDownoad, capturePeriods, meteoList
                    );

            List<CaptureConf> captureConfs = captureConfsPlain.Concat(captureConfsCoridor).ToList();

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
        /// <param name="routesToDownload">Перечень маршрутов на сброс</param>
        /// <param name="routesToDelete">Перечень маршрутов на удаление</param>
        /// <param name="managerDB">параметры взаимодействия с БД</param>
        /// <param name="Nmax">номер,с которого мпз следует нумеровать</param>
        /// <param name="mpzArray">Набор МПЗ</param>
        /// <param name="sessions">Сеансы связи</param>
        /// <param name="enabledStations">список включённых станций</param>
        /// <param name="flags">Флаги для настройки МПЗ</param>
        public static void getMPZArray(
              List<RequestParams> requests
            , DateTime timeFrom
            , DateTime timeTo
            , List<Tuple<DateTime, DateTime>> silentRanges
            , List<Tuple<DateTime, DateTime>> inactivityRanges
            , List<RouteMPZ> routesToDownload
            , List<RouteMPZ> routesToDelete
            , string conStringCUP
            , string conStringCUKS
            , int Nmax
            , out List<MPZ> mpzArray
            , out List<CommunicationSession> sessions
            , List<SessionsPlanning.CommunicationSessionStation> enabledStations
            , FlagsMPZ flags = null
            )
        {
            DIOS.Common.SqlManager ManagerDbCUP = new DIOS.Common.SqlManager(conStringCUP);
            DIOS.Common.SqlManager ManagerDbCUKS = new DIOS.Common.SqlManager(conStringCUKS);

            List<TimePeriod> silentTimePeriods = silentRanges.Select(tuple => new TimePeriod(tuple.Item1, tuple.Item2)).ToList();
            List<TimePeriod> inactivityTimePeriods = inactivityRanges.Select(tuple => new TimePeriod(tuple.Item1, tuple.Item2)).ToList();

            Dictionary<CommunicationSessionStation, List<CommunicationSession>> nkpoiSessions
                = CommunicationSession.createCommunicationSessions(timeFrom, timeTo, conStringCUP, enabledStations);

            List<TimePeriod> shadowPeriods;// = new List<TimePeriod>();
            List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot;
            checkIfViewLaneIsLitWithTimeSpans(ManagerDbCUP, timeFrom, timeTo, out partsLitAndNot, out shadowPeriods);

            /*
            var strings = partsLitAndNot.SelectMany(pair => pair.Item2.Where(part => part.sun).Select(part => new Polygon(part.wktPolygon).ToWebglearthString()));
            int i = 0;
            foreach (var s in strings)
            {
                Console.WriteLine("pol"+i.ToString() +" = " + s);
                Console.WriteLine("pol" + i.ToString() + ".addTo(earth);");
                Console.WriteLine();
                i++;
            }
            */
            
            List<TimePeriod> shadowAndInactivityPeriods = new List<TimePeriod>();
            shadowAndInactivityPeriods.AddRange(inactivityTimePeriods);
            shadowAndInactivityPeriods.AddRange(shadowPeriods);
            shadowAndInactivityPeriods = TimePeriod.compressTimePeriods(shadowAndInactivityPeriods);

            // временные периоды, во время которых можно проводить съемку со сбросом
            List<TimePeriod> freeIntervalsForDownload
                = CommunicationSession.getFreeTimePeriodsOfSessions(nkpoiSessions.SelectMany(list => list.Value).ToList(), silentTimePeriods);

            // расчёт всех возможных конфигураций съемки на этот период с учётом ограничений
            List<CaptureConf> confsToCapture
                = getCaptureConfArray(
                requests,
                timeFrom,
                timeTo,
                ManagerDbCUP,
                ManagerDbCUKS,
                shadowAndInactivityPeriods,
                freeIntervalsForDownload);
            
#if DEBUG
#warning this is only for debug
            List<Polygon> pols = confsToCapture.Select(cc => new Polygon(cc.wktPolygon)).ToList();
            Console.WriteLine(Polygon.getMultipolFromPolygons(pols));
#endif

            // поиск оптимального набора маршрутов среди всех возможных конфигураций
            List<MPZParams> captureMPZParams = new Graph(confsToCapture).findOptimalChain(Nmax);

            // все параметры МПЗ на съемку, пришедшие из графа
            //List<MPZ> captureMpz = new Graph(confsToCapture).findOptimalChain(Nmax).Select(mpz_param => new MPZ(mpz_param, ManagerDbCUP, ManagerDbCUKS, flags ?? new FlagsMPZ())).ToList();

            // составим массив маршрутов на сброс (скачивание)
            List<RouteParams> allRoutesToDownload = new List<RouteParams>();
            allRoutesToDownload.AddRange(routesToDownload.Select(r => r.Parameters));
            allRoutesToDownload.AddRange(captureMPZParams.SelectMany(mpz => mpz.routes)
                .Where(route => route.type == WorkingType.Shooting)
                .ToList()); // добавим к сбросу только что отснятые маршруты

            // временные промежутки, занятые съемкой           
            List<TimePeriod> captureIntervals = captureMPZParams.Select(mpz => new TimePeriod(mpz.start, mpz.end)).ToList();

            // временные интервалы каждой антенны
            Dictionary<CommunicationSessionStation, List<TimePeriod>> nkpoiSessionsIntervals
             = nkpoiSessions.Select(pair => new KeyValuePair<CommunicationSessionStation, List<TimePeriod>>
            (pair.Key, pair.Value.Select(sess => sess.DropInterval).ToList()))
            .ToDictionary(pair => pair.Key, pair => pair.Value);

            // вырежем из интервалов для сброса занятые съемкой интервалы и интервалы silentTimePeriods            
            foreach (var station in nkpoiSessionsIntervals.Keys.ToList())
            {
                nkpoiSessionsIntervals[station] = TimePeriod.compressTimePeriods<TimePeriod>(nkpoiSessionsIntervals[station]);
                nkpoiSessionsIntervals[station].erase(silentTimePeriods); // вырежем из этих периодов все запрещённые к сбросу периоды времени
                nkpoiSessionsIntervals[station].erase(captureIntervals);  // вырежем из этих периодов все занятые съемкой периоды времени
            }

            List<CommunicationSessionStation> sortedStationList = new DataFetcher(ManagerDbCUKS).getSortedStations();

            int maxMpzNum = routesToDelete.Select(mpz => mpz.NPZ).DefaultIfEmpty(Nmax).Max();
            maxMpzNum = Math.Max(maxMpzNum, captureMPZParams.Select(mpz => mpz.id).DefaultIfEmpty(0).Max());

             List<MPZParams> downloadMpzParams = new List<MPZParams>();

            // распределим маршруты на сброс по доступным интервалам
            foreach (CommunicationSessionStation station in sortedStationList)
            {
                if (!nkpoiSessionsIntervals.ContainsKey(station))
                    continue;
                List<TimePeriod> freeStationIntervals = nkpoiSessionsIntervals[station];

                // сначала постараемся уместить маршруты на сброс в имеющиеся МПЗ
                foreach (var capPrms in captureMPZParams)
                {
                    // если антенна для этого МПЗ уже определена и отличается от текущей, значит пропускаем МПЗ
                    if (capPrms.Station.HasValue)
                        if (capPrms.Station.Value != station)
                            continue;

                    foreach (var routeToDownload in allRoutesToDownload.ToList())
                    {
                        RouteParams routeParams = new RouteParams(
                            WorkingType.Downloading,
                            routeToDownload.getDropTime(station).TotalSeconds,
                            routeToDownload);

                        foreach (TimePeriod period in freeStationIntervals.ToList())
                        {
                            if (!capPrms.InsertRoute(routeParams, TimePeriod.Max(routeToDownload.end, period.dateFrom), period.dateTo, null, null))
                                continue;
                            allRoutesToDownload.Remove(routeToDownload);
                            freeStationIntervals.erase(period);
                            if (!capPrms.Station.HasValue)
                                capPrms.Station = station;
                            break;
                        }
                    }
                }

                // Оставшиеся маршруты попытаемся вместить в оставшиеся сессии этой станции
                List<RouteParams> downloadRoutes
                    = TimePeriod.getRoutesParamsInIntervals(allRoutesToDownload, freeStationIntervals, workType: WorkingType.Downloading, station: station);//, startId: maxRoutesNumber);

                freeStationIntervals.erase(downloadRoutes.Select(route => new TimePeriod(route.start, route.end)).ToList());

                maxMpzNum = Math.Max(maxMpzNum, downloadMpzParams.Select(mpzparam => mpzparam.id).DefaultIfEmpty(0).Max());
                List<MPZParams> curMPZ = MPZParams.FillMPZ(downloadRoutes, maxMpzNum);
                curMPZ.ForEach(mpz => mpz.Station = station);
                downloadMpzParams.AddRange(curMPZ);
            }

            // наёдем еще свободные временные промежутки, в которые можно произвести удаление
             List<TimePeriod> downloadIntervals = downloadMpzParams.Select(mpzparams => new TimePeriod(mpzparams.start, mpzparams.end)).ToList();
            List<TimePeriod> inactivityDownCaptIntervals = new List<TimePeriod>();
            inactivityDownCaptIntervals.AddRange(captureIntervals);
            inactivityDownCaptIntervals.AddRange(downloadIntervals);
            inactivityDownCaptIntervals.AddRange(inactivityTimePeriods);
            inactivityDownCaptIntervals = TimePeriod.compressTimePeriods(inactivityDownCaptIntervals);
            List<TimePeriod> freeRangesForDelete = TimePeriod.getFreeIntervals(inactivityDownCaptIntervals, timeFrom, timeTo);

            List<RouteParams> deleteRoutesParams
                = TimePeriod.getRoutesParamsInIntervals(routesToDelete.Select(rout => rout.Parameters).ToList(), freeRangesForDelete, workType: WorkingType.Removal);//, startId: maxRoutesNumber);

            maxMpzNum = Math.Max(maxMpzNum, downloadMpzParams.Select(mpzparam => mpzparam.id).DefaultIfEmpty(0).Max());

            // создаем новые мпз удаления из маршрутов на удаление
            List<MPZParams> deleteMpzParams = new List<MPZParams>();

            deleteMpzParams.AddRange(MPZParams.FillMPZ(deleteRoutesParams, maxMpzNum));

            mpzArray = new List<MPZ>();
            mpzArray.AddRange(captureMPZParams.Select(mpz_param => new MPZ(mpz_param, ManagerDbCUP, ManagerDbCUKS, flags ?? new FlagsMPZ())));
            mpzArray.AddRange(downloadMpzParams.Select(mpz_param => new MPZ(mpz_param, ManagerDbCUP, ManagerDbCUKS, flags ?? new FlagsMPZ())));
            mpzArray.AddRange(deleteMpzParams.Select(mpz_param => new MPZ(mpz_param, ManagerDbCUP, ManagerDbCUKS, flags ?? new FlagsMPZ())));

            // составим массив использованных сессий
            sessions = new List<CommunicationSession>();

            // получим все интервалы сброса
            var downloadingIntervals = mpzArray
                .SelectMany(mpz => mpz.Routes)
                .Select(route => route.Parameters)
                .Where(route_par => route_par.type == WorkingType.Downloading || route_par.type == WorkingType.ShootingSending)
                .Select(route => new TimePeriod(route.start, route.end)).ToList();

            foreach (var interval in downloadingIntervals.ToArray())
            {
                foreach (var sess in nkpoiSessions.SelectMany(group => group.Value))
                {
                    if (!TimePeriod.isPeriodInPeriod(interval, sess.DropInterval)) // если этот интервал не в сессии, значит не подходит
                        continue;
                    if (!sessions.Contains(sess)) // добавляем только если еще не добавили
                        sessions.Add(sess);
                    downloadingIntervals.Remove(interval);
                    break;
                }
            }
        }

        /// <summary>
        /// Создание ПНб по набору маршрутов
        /// </summary>
        /// <param name="routesParams"> Набор маршрутов RouteParams</param>
        /// <returns> набор МПЗ, созданный из маршутов</returns>
        /// <param name="Nmax">номер,с которого мпз следует нумеровать</param>
        /// <param name="managerDB">параметры взаимодействия с БД</param>
        /// <param name="flags">Флаги для настройки МПЗ</param>
        public static List<MPZ> createPNbOfRoutes(List<RouteParams> routesParams, int Nmax, string conStringCUP, string conStringCUKS, FlagsMPZ flags = null)
        {
            List<MPZ> res = new List<MPZ>();
            List<MPZParams> mpzParams = MPZParams.FillMPZ(routesParams, Nmax);
            DIOS.Common.SqlManager managerDbCUP = new DIOS.Common.SqlManager(conStringCUP);
            DIOS.Common.SqlManager ManagerDbCUKS = new DIOS.Common.SqlManager(conStringCUKS);
            return mpzParams.Select(param => new MPZ(param, managerDbCUP, ManagerDbCUKS, flags ?? new FlagsMPZ())).ToList();
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

        public static void getPiecewiseCoridorParams(DateTime dateTime, List<GeoPoint> vertices, DIOS.Common.SqlManager managerDB, out List<CoridorParams> coridorParams, double maxSubCurveLengthMeters = 3.5e5)
        {
            DateTime now = dateTime;
            Trajectory traj = getMaxTrajectory(managerDB, dateTime);

            var curves = new Curve(vertices).BreakByCurvatureAndDistance(maxSubCurveLengthMeters);
            coridorParams = new List<CoridorParams>();
            CoridorParams oneParam;

            for (int i = 0; i < curves.Count; ++i)
            {
                getCustomCoridor(traj, now, curves[i], out oneParam);
                now = now.AddSeconds(oneParam.Duration + 20);
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
        public static void getCoridorPoly(DateTime dateTime, double rollAngle, double pitchAngle, double dist, double az, string connectStr,
            out string wktPoly, out double duration)
        {
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(connectStr);
            if (dist > 97e3)
                throw new ArgumentException("Coridor length cannot exceed 97km.");

            CoridorParams coridorParams = getCoridorParams(dateTime, rollAngle, pitchAngle, dist, az, connectStr, out duration);
            coridorParams.ComputeCoridorPolygon(getMaxTrajectory(managerDB, dateTime));
            wktPoly = coridorParams.Coridor.ToWtk();
        }

        public static CoridorParams getCoridorParams(DateTime dateTime, double rollAngle, double pitchAngle, double dist, double az, string connectStr, out double duration)
        {
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(connectStr);

            DataFetcher fetcher = new DataFetcher(managerDB);
            //TrajectoryPoint? p0_ = fetcher.GetPositionSat(dateTime);
            Trajectory traj = getMaxTrajectory(managerDB, dateTime);

            double l1, l2, b1, b2, s1, s2, s3;
            TrajectoryRoutines.GetCoridorParams(
                traj, dateTime, az, dist,
                rollAngle, pitchAngle,
                out b1, out b2, out l1, out l2, out s1, out s2, out s3, out duration);

            CoridorParams coridorParams = new CoridorParams(l1, l2, b1, b2, s1, s2, s3, 0, rollAngle, pitchAngle, dateTime, dateTime.AddSeconds(duration));
            return coridorParams;
        }


        public static void getCoridorPoly(DateTime dateTime, double rollAngle, double pitchAngle, GeoPoint end, string connectStr,
            out string wktPoly, out double duration, out double dist)
        {
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(connectStr);
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

        public static void getCoridorPoly(DateTime dateTime, GeoPoint start, GeoPoint end, string connectStr,
            out string wktPoly, out double duration, out double dist)
        {
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(connectStr);
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
        public static string getSOENViewPolygon(DateTime dateTime, double rollAngle, double pitchAngle, int duration, string connectStr, bool isCoridor = false)
        {
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(connectStr);
            string wtk = "";
            DataFetcher fetcher = new DataFetcher(managerDB);

            if (!isCoridor)
            {
                if (duration == 0)
                {
                    TrajectoryPoint? point = fetcher.GetSingleSatPoint(dateTime);

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
                getCoridorPoly(dateTime, rollAngle, pitchAngle, 96e3, Math.PI / 6, connectStr, out wtk, out dur);
            }

            return wtk;
        }

        

        ///\<summary>
        ///Разбиение полосы видимости КА под траекторией на полигоны освещенности.
        ///</summary>
        ///<param name="DBManager">Параметры подключения к БД</param>
        ///<param name="timeFrom">Начало временного промежутка</param>
        ///<param name="timeTo">Конец временного промежутка</param>
        ///<param name="partsLitAndNot">Список объектов: номер витка и полигоны, помеченные флагом освещенности</param>
        public static void checkIfViewLaneIsLit(string connectStr, DateTime timeFrom, DateTime timeTo, out List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot)
        {
            DIOS.Common.SqlManager DBManager = new DIOS.Common.SqlManager(connectStr);
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

                    var LitAndNot = SphericalGeom.Polygon.IntersectAndSubtract(sector, Polygon.Hemisphere(sun));
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

        
        ///<summary>
        ///Разбиение полосы видимости КА под траекторией на полигоны освещенности.
        ///</summary>
        ///<param name="DBManager">Параметры подключения к БД</param>
        ///<param name="timeFrom">Начало временного промежутка</param>
        ///<param name="timeTo">Конец временного промежутка</param>
        ///<param name="partsLitAndNot">Список объектов: номер витка и полигоны, помеченные флагом освещенности</param>
        public static void checkIfViewLaneIsLitWithTimeSpans(
            DIOS.Common.SqlManager DBManager,
            DateTime timeFrom,
            DateTime timeTo,
            out List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot,
            out List<TimePeriod> shadowPeriods)
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

                    // can ignore scaling here as the distances are enormous both in kms and in units of Earth radius
                    Vector3D sun = sunPositions[curSunPositionIndex].Position;
                    SphericalGeom.Polygon sector = SatelliteTrajectory.TrajectoryRoutines.FormSectorFromLanePoints(lane, i, i + 1);
                    
                    var LitAndNot = SphericalGeom.Polygon.IntersectAndSubtract(sector, Polygon.Hemisphere(sun));
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
            TimePeriod.compressTimePeriods(shadowPeriods);
        }

        /// <summary>
        /// Проверка маршрута на совместимость с ПНБ
        /// </summary>
        /// <param name="routeParams">параметры маршрута</param>
        /// <param name="pnb">ПНБ</param>
        /// <param name="conflicts">список найденных конфликтов</param>
        public static void checkPNBRouteCompatible(RouteParams routeParams, List<MPZ> pnb, out List<Tuple<int, int>> conflicts)
        {
            conflicts = new List<Tuple<int, int>>();
            foreach (var mpz in pnb)
            {
                if (!mpz.Parameters.isCompatibleWithMPZ(routeParams))
                {
                    conflicts.Add(Tuple.Create(mpz.Parameters.id, -1));
                    continue;
                }
                foreach (var route in mpz.Routes)
                {
                    if (!route.Parameters.isCompatible(routeParams))
                        conflicts.Add(Tuple.Create(mpz.Parameters.id, route.Parameters.NRoute));
                }
            }
        }

        /// <summary>
        /// Добавление мрашрута в ПНБ
        /// </summary>
        /// <param name="routeParams">параметры добавляемого маршрута</param>
        /// <param name="PNB">ПНБ</param>
        /// <param name="connString">строка подключения к БД ЦУП</param>
        /// <param name="modRouteParams">изменённые параметры маршрута</param>
        /// <param name="newMPZ">новй МПЗ (null, если не создался)</param>
        /// <param name="sessionStart">время начала сессии (null, если не задано)</param>
        /// <param name="sessionEnd">время конца сессии (null, если не задано)</param>
        public static void addRouteToPNB(RouteParams routeParams,
            List<MPZ> PNB,
            string connStringCup,
            string connStringCuks,
            out RouteParams modRouteParams,
            out MPZ newMPZ,
            DateTime? sessionStart = null,
            DateTime? sessionEnd = null)
        {
            foreach (var mpz in PNB)
            {
                if (!mpz.Parameters.isCompatibleWithMPZ(routeParams))
                {
                    throw new ArgumentException("Route has conflict with FTA #" + mpz.Parameters.id.ToString());
                }
                foreach (var mpzRoute in mpz.Routes)
                {
                    if (!mpzRoute.Parameters.isCompatible(routeParams))
                        throw new ArgumentException("Route has conflict with Route #" + mpz.Parameters.id.ToString() + "." + mpzRoute.Parameters.NRoute.ToString());
                }
            }

            modRouteParams = new RouteParams(routeParams);
            newMPZ = null;

            if (!sessionStart.HasValue)
                sessionStart = DateTime.MinValue;
            if (!sessionEnd.HasValue)
                sessionEnd = DateTime.MaxValue;

            foreach (var mpz in PNB)
            {
                var copyMpzParams = new MPZParams(mpz.Parameters);
                if (copyMpzParams.InsertRoute(modRouteParams, sessionStart.Value, sessionEnd.Value))
                    return;
            }

            List<MPZParams> tmpList = MPZParams.FillMPZ(new List<RouteParams>() { modRouteParams });

            if (tmpList.Count > 0)
                newMPZ = new MPZ(tmpList.First(), new DIOS.Common.SqlManager(connStringCup), new DIOS.Common.SqlManager(connStringCuks), new FlagsMPZ());
        }

        /// <summary>
        /// Удаление маршрута из ПНБ
        /// </summary>
        /// <param name="PNB">ПНБ</param>
        /// <param name="MPZId">номер мпз, из которого удаляем маршрут</param>
        /// <param name="routeId">номер удаляемого маршрута</param>
        public static void deleteRouteFromPNB(List<MPZ> PNB, int MPZId, int routeId)
        {
            var mpz = PNB.FirstOrDefault(m => m.Header.NPZ == MPZId);

            if (mpz == null)
                throw new ArgumentException("Mpz id is incorrect");

            var route = mpz.Routes.FirstOrDefault(r => r.Nroute == routeId);

            if (route == null)
                throw new ArgumentException("Route id is incorrect");

            mpz.Routes.Remove(route);
        }




        /// <summary>
        /// Создание параметров маршрута для обычной съемки
        /// </summary>
        /// <param name="connectStr">строка для подключения к БД</param>
        /// <param name="dtFrom">время начала съемки</param>
        /// <param name="duration">длительность съемки в секундах</param>
        /// <param name="channel">канал</param>
        /// <param name="shType">тип съемки</param>
        /// <param name="wType">тип работы</param>
        /// <param name="roll">крен</param>
        /// <param name="pitch">тангаж</param>
        /// <returns>параметры маршрута</returns>
        public static RouteParams createNormalCaptureRoute(
            string connectStr,
            DateTime dtFrom,
            double duration,
            ShootingChannel channel,
            ShootingType shType,
            WorkingType wType,
            double roll,
            double pitch)
        {
            if (!(wType == WorkingType.ShootingSending || wType == WorkingType.Shooting))
                throw new ArgumentException("Unsupported working type");

            DIOS.Common.SqlManager managerDb = new DIOS.Common.SqlManager(connectStr);
            DataFetcher fetcher = new DataFetcher(managerDb);

            DateTime dtTo = dtFrom.AddMilliseconds(duration);
            Trajectory trajectory = fetcher.GetTrajectorySat(dtFrom, dtTo);
            Polygon pol = SatLane.getRollPitchLanePolygon(trajectory, roll, pitch);

            OptimalChain.StaticConf stConf = new StaticConf(
                i: 0,
                d1: dtFrom,
                d2: dtTo,
                t: pitch,
                r: roll,
                s: pol.Area,
                o: new List<Order>(),
                polygon: pol.ToWtk(),
                comp: 0,
                alb: 0,
                T: wType,
                channel: channel,
                stype: shType);

            RouteParams route = new RouteParams(stConf);
            return route;
        }

        /// <summary>
        /// создание параметров маршрута коридорной съемки
        /// </summary>
        /// <param name="connectStr">строка подключения к БД</param>
        /// <param name="from">время начала съемки</param>
        /// <param name="channel">канал</param>
        /// <param name="shType">тип съемки</param>
        /// <param name="wType">тип работы</param>
        /// <param name="coridorAzimuth">азимут</param>
        /// <param name="coridorLength">длина коридора</param>
        /// <param name="roll">крен</param>
        /// <param name="pitch">тангаж</param>
        /// <returns>параметры маршрута</returns>
        public static RouteParams createCorridorCaptureRoute(string connectStr,
            DateTime from,
            ShootingChannel channel,
            ShootingType shType,
            WorkingType wType,
            double coridorAzimuth,
            double coridorLength,
            double roll,
            double pitch)
        {
            if (!(wType == WorkingType.ShootingSending || wType == WorkingType.Shooting))
                throw new ArgumentException("Unsupported working type");
            double duration;
            CoridorParams corrParams = getCoridorParams(from, roll, pitch, coridorLength, coridorAzimuth, connectStr, out duration);

            OptimalChain.StaticConf stConf = new StaticConf(
                    i: 0,
                    d1: corrParams.StartTime,
                    d2: corrParams.EndTime,
                    t: 0,
                    r: 0,
                    s: corrParams.Coridor.Area,
                    o: new List<Order>(),
                    polygon: corrParams.Coridor.ToWtk(),
                    comp: 0,
                    alb: 0,
                    T: wType,
                    channel: channel,
                    stype: shType,
                    _poliCoef: corrParams.CoridorCoefs);

            return new RouteParams(stConf);
        }

        /// <summary>
        /// Создание параметров маршрута коридорной съемки
        /// </summary>
        /// <param name="connectStr">строка подключения к БД</param>
        /// <param name="from">время начала съемки</param>
        /// <param name="channel">канал</param>
        /// <param name="shType">тип съемки</param>
        /// <param name="wType">тип работы</param>
        /// <param name="wktPolygon">полигон заказа, снимаемый коридорами</param>
        /// <returns>массив параметров маршрута</returns>
        public static List<RouteParams> createCorridorCaptureRoute(
            string connectStr,
            DateTime from,
            ShootingChannel channel,
            ShootingType shType,
            WorkingType wType,
            string wktPolygon)
        {
            if (!(wType == WorkingType.ShootingSending || wType == WorkingType.Shooting))
                throw new ArgumentException("Unsupported working type");

            DIOS.Common.SqlManager managerDb = new DIOS.Common.SqlManager(connectStr);
            List<RouteParams> res = new List<RouteParams>();
            List<CoridorParams> coridorParams;
            var line = new Polygon(wktPolygon).getCenterLine();
            
            if (line.Count < 2)
                return res;

            getPiecewiseCoridorParams(from, line, managerDb, out coridorParams);

            foreach (CoridorParams corrParams in coridorParams)
            {
                DateTime toDt = corrParams.EndTime;
                OptimalChain.StaticConf stConf = new StaticConf(
                   i: 0,
                   d1: corrParams.StartTime,
                   d2: corrParams.EndTime,
                   t: 0,
                   r: 0,
                   s: corrParams.Coridor.Area,
                   o: new List<Order>(),
                   polygon: corrParams.Coridor.ToWtk(),
                   comp: 0,
                   alb: 0,
                   T: wType,
                   channel: channel,
                   stype: shType,
                   _poliCoef: corrParams.CoridorCoefs);

                res.Add(new RouteParams(stConf));
            }

            return res;
        }

        /// <summary>
        /// Создание памаетром маршрута на удаление или сброс (скачивание)
        /// </summary>
        /// <param name="fromDt">время начала съемки </param>
        /// <param name="channel">канал</param>
        /// <param name="shType">тип съемки</param>
        /// <param name="wType">тип работы</param>
        /// <param name="mpzId">номер МПЗ</param>
        /// <param name="routeId">номер удаляемого (или сбрасываемого) маршрута</param>
        /// <returns>маршрут на удаление или сброс</returns>
        public static RouteParams createServiceRoute(
            DateTime fromDt,
            ShootingChannel channel, /// @toso зачем эти параметры
            ShootingType shType, /// @toso зачем эти параметры
            WorkingType wType,
            RouteParams routeToAction)
        {
            double actionTime = 0;
            if (wType == WorkingType.Downloading)
                actionTime = routeToAction.getDropTime(CommunicationSessionStation.FIGS_Main).TotalSeconds;
            else if (wType == WorkingType.Removal)
                actionTime = OptimalChain.Constants.routeDeleteTime;
            else
                throw new ArgumentException("Unsupported working type");

            DateTime toDt = fromDt.AddSeconds(actionTime + OptimalChain.Constants.min_Delta_time);

            RouteParams curParam = new RouteParams(wType, fromDt, toDt, routeToAction);
            return curParam;
        }

    }

    public class wktPolygonLit
    {
        public string wktPolygon { get; set; }
        public bool sun { get; set; }
    }



}




