using System;
using System;
using System.Collections.Generic;
using System.Linq;
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
using System.Diagnostics;
using System.Reflection.Emit;
using DBTables;
using SatelliteSessions;
using SphericalGeom;

namespace SessionsPlanning
{
    public static class TestSessionsSequenses
    {
        /// <summary>
        /// Кадровая съемка. 14 кадров на витке 
        /// </summary>
        /// <param name="fromDt">время начала витка</param>
        /// <param name="managerDB"></param>
        /// <param name="mpzArray"></param>
        public static void get14PlainFrames1Turn(DateTime fromDt, DIOS.Common.SqlManager managerDB, DIOS.Common.SqlManager managerDbCUKS, out List<MPZ> mpzArray)
        {
            ShootingTestTemplate test = ShootingTestFactory.getShootingTest(ShootingType.Normal);
            test.getFramesForOneTurn(fromDt, managerDB, managerDbCUKS, out mpzArray, 14);
            checkRoutesCount(mpzArray, 14);
        }

        /// <summary>
        /// Кадровая съемка. 45 кадров на 5-ти витках по 9 кадров
        /// </summary>
        /// <param name="fromDt">дата начала первого витка</param>
        public static void get45PlainFrames4Turn(DateTime fromDt, DIOS.Common.SqlManager managerDB, DIOS.Common.SqlManager managerDbCUKS, out List<MPZ> mpzArray)
        {
            ShootingTestTemplate test = ShootingTestFactory.getShootingTest(ShootingType.Normal);
            test.getFrames(fromDt, managerDB, managerDbCUKS, out mpzArray, 9, 5);
            checkRoutesCount(mpzArray, 45);
        }

        /// <summary>
        /// Стереосъемка. 8 стереотриплетов  на одном витке
        /// </summary>
        public static void get8StereoTriplets1Turn(DateTime fromDt, DIOS.Common.SqlManager managerDB, DIOS.Common.SqlManager managerDbCUKS, out List<MPZ> mpzArray)
        {
            ShootingTestTemplate test = ShootingTestFactory.getShootingTest(ShootingType.StereoTriplet);
            test.getFramesForOneTurn(fromDt, managerDB, managerDbCUKS, out mpzArray, 8);
            checkRoutesCount(mpzArray, 24);
        }

        /// <summary>
        /// Стереосъемка. 8 стереопар на одном витке
        /// </summary>
        public static void get8StereoPairs1Turn(DateTime fromDt, DIOS.Common.SqlManager managerDB, DIOS.Common.SqlManager managerDbCUKS, out List<MPZ> mpzArray)
        {
            ShootingTestTemplate test = ShootingTestFactory.getShootingTest(ShootingType.Stereo);
            test.getFramesForOneTurn(fromDt, managerDB, managerDbCUKS, out mpzArray, 8);
            checkRoutesCount(mpzArray, 16);
        }

        /// <summary>
        /// Стереосъемка. 20 стереотриплетов на 5-ти витках по 4 стереотриплетов
        /// </summary>
        public static void get20StereoTriplets5Turn(DateTime fromDt, DIOS.Common.SqlManager managerDB, DIOS.Common.SqlManager managerDbCUKS, out List<MPZ> mpzArray)
        {
            ShootingTestTemplate test = ShootingTestFactory.getShootingTest(ShootingType.StereoTriplet);
            test.getFrames(fromDt, managerDB, managerDbCUKS, out mpzArray, 5, 4);
            checkRoutesCount(mpzArray, 60);
        }

        /// <summary>
        ///  Стереосъемка. 20 стереопар на 5-ти витках по 4 стереопар
        /// </summary>
        public static void get20StereoPairs5Turn(DateTime fromDt, DIOS.Common.SqlManager managerDB, DIOS.Common.SqlManager managerDbCUKS, out List<MPZ> mpzArray)
        {
            ShootingTestTemplate test = ShootingTestFactory.getShootingTest(ShootingType.Stereo);
            test.getFrames(fromDt, managerDB, managerDbCUKS, out mpzArray, 5, 4);
            checkRoutesCount(mpzArray, 40);
        }

        /// <summary>
        /// Коридорная съемка. 8 коридоров на одном витке
        /// </summary>
        public static void get8Coridors1Turn(DateTime fromDt, DIOS.Common.SqlManager managerDB, DIOS.Common.SqlManager managerDbCUKS, out List<MPZ> mpzArray)
        {
            ShootingTestTemplate test = ShootingTestFactory.getShootingTest(ShootingType.Coridor);
            test.getFramesForOneTurn(fromDt, managerDB, managerDbCUKS, out mpzArray, 8);
            checkRoutesCount(mpzArray, 8);
        }

        /// <summary>
        /// Коридорная съемка. 20 коридоров на 5-ти витках по 4 коридора
        /// </summary>
        public static void get20Coridors5Turn(DateTime fromDt, DIOS.Common.SqlManager managerDB, DIOS.Common.SqlManager managerDbCUKS, out List<MPZ> mpzArray)
        {
            ShootingTestTemplate test = ShootingTestFactory.getShootingTest(ShootingType.Coridor);
            test.getFrames(fromDt, managerDB, managerDbCUKS, out mpzArray, 5, 4);
            checkRoutesCount(mpzArray, 20);
        }

        /// <summary>
        /// Площадная съемка. 8 площадных съемок на одном витке
        /// </summary>
        public static void get8AreaShooting1Turn(DateTime fromDt, DIOS.Common.SqlManager managerDB, DIOS.Common.SqlManager managerDbCUKS, out List<MPZ> mpzArray)
        {
            ShootingTestTemplate test = ShootingTestFactory.getShootingTest(ShootingType.Area);
            test.getFramesForOneTurn(fromDt, managerDB, managerDbCUKS, out mpzArray, 8);
            checkRoutesCount(mpzArray, 32);
        }

        /// <summary>
        /// Площадная съемка. 20 площадных съемок на 5-ти витках по 4 коридора
        /// </summary>
        public static void get20AreaShooting5Turn(DateTime fromDt, DIOS.Common.SqlManager managerDB, DIOS.Common.SqlManager managerDbCUKS, out List<MPZ> mpzArray)
        {
            ShootingTestTemplate test = ShootingTestFactory.getShootingTest(ShootingType.Area);
            test.getFrames(fromDt, managerDB, managerDbCUKS, out mpzArray, 5, 4);
            checkRoutesCount(mpzArray, 80);
        }


        /// <summary>
        /// Кадровая съемка. 14 кадров на витке 
        /// </summary>
        /// <param name="fromDt">время начала витка</param>
        /// <param name="managerDB"></param>
        /// <param name="mpzArray"></param>
        public static void getStrip4150km1Turn(DateTime fromDt, DIOS.Common.SqlManager managerDB, DIOS.Common.SqlManager managerDbCUKS, out List<MPZ> mpzArray)
        {
            ShootingTestTemplate test = ShootingTestFactory.getShootingTest(ShootingType.Normal, true);
            test.getFramesForOneTurn(fromDt, managerDB, managerDbCUKS, out mpzArray, 4150);
            checkRoutesCount(mpzArray, 1);
        }

        /// <summary>
        /// Кадровая съемка. 45 кадров на 5-ти витках по 9 кадров
        /// </summary>
        /// <param name="fromDt">дата начала первого витка</param>
        public static void getStrip12050km5Turn(DateTime fromDt, DIOS.Common.SqlManager managerDB, DIOS.Common.SqlManager managerDbCUKS, out List<MPZ> mpzArray)
        {
            ShootingTestTemplate test = ShootingTestFactory.getShootingTest(ShootingType.Normal, true);
            test.getFrames(fromDt, managerDB, managerDbCUKS, out mpzArray, 5, 2410);
            checkRoutesCount(mpzArray, 5);
        }

        private static void checkRoutesCount(List<MPZ> mpzArray, int count)
        {
            if (mpzArray.Sum(mpz => mpz.Routes.Count) != count)
                throw new TestSequensesException("It is impossible to compose a test sequence for a given time");
        }
    }

    public class TestSequensesException : Exception
    {
        public TestSequensesException()
        {
        }

        public TestSequensesException(string message)
            : base(message)
        {
        }
    }

    internal class ShootingTestFactory
    {
        public static ShootingTestTemplate getShootingTest(ShootingType type, bool alongStrip = false)
        {
            if (type == ShootingType.Normal)
                if (alongStrip)
                    return new StripShootingTest(type);
                else
                    return new PlainShootingTest(type);
            else if (type == ShootingType.Stereo || type == ShootingType.StereoTriplet)
                return new StereoShootingTest(type);
            else if (type == ShootingType.Coridor)
                return new CoridorShootingTest(type);
            else if (type == ShootingType.Area)
                return new AreaShootingTest(type);
            else
                throw new ArgumentException();
        }
    }

    internal abstract class ShootingTestTemplate
    {
        protected ShootingType type;

        public ShootingTestTemplate(ShootingType _type)
        {
            this.type = _type;
        }

        public void getFrames(DateTime fromDt,
            DIOS.Common.SqlManager managerDB,
            DIOS.Common.SqlManager managerDbCUKS,
            out List<MPZ> mpzArray,
            int numTurns, int numFramesOnTurn)
        {
            mpzArray = new List<MPZ>();
            for (int i = 0; i < numTurns; i++)
            {
                List<MPZ> curMpz;
                getFramesForOneTurn(fromDt, managerDB, managerDbCUKS, out curMpz, numFramesOnTurn);
                mpzArray.AddRange(curMpz);
                double turnDuration = 99.2;
                fromDt = fromDt.AddMinutes(turnDuration);

                double minPause = (double)OptimalChain.Constants.CountMinPause(WorkingType.Shooting, type,
                ShootingChannel.pk, WorkingType.Shooting, type, ShootingChannel.pk) / 1000000;
 
                double dt = minPause + OptimalChain.Constants.minDeltaT / 1000;
                dt += OptimalChain.Constants.MPZ_starting_Time / 1000 + OptimalChain.Constants.MPZ_init_Time / 1000;
                fromDt = fromDt.AddSeconds(dt);
            }
        }

        public void getFramesForOneTurn(
            DateTime fromDt,
            DIOS.Common.SqlManager managerDB,
            DIOS.Common.SqlManager managerDbCUKS,
            out List<MPZ> mpzArray,
            int numFrames)
        {
            fromDt = fromDt.AddMilliseconds(OptimalChain.Constants.MPZ_starting_Time); 
            double turnDuration = 99.2;

            DateTime toDt = fromDt.AddMinutes(turnDuration);

            List<TimePeriod> shadowPeriods = new List<TimePeriod>();
            List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot;
          //  Sessions.checkIfViewLaneIsLitWithTimeSpans(managerDB, fromDt, toDt, out partsLitAndNot,
           //     out shadowPeriods);

            List<TimePeriod> freeIntervals = TimePeriod.getFreeIntervals(shadowPeriods, fromDt, toDt);

            List<StaticConf> sconfs = new List<StaticConf>();

            int i = 0;
            foreach (var period in freeIntervals)
            {
                sconfs.AddRange(getSConfs(period, managerDB, numFrames, ref i));
                if (i >= numFrames)
                    break;
            }       

            List<RouteParams> routesParams = sconfs.Select(conf => new RouteParams(conf)).ToList();
            List<MPZParams> mpzParams = MPZParams.FillMPZ(routesParams, 0);
            mpzArray = mpzParams.Select(param => new MPZ(param, managerDB, managerDbCUKS, new FlagsMPZ())).ToList();
        }



        protected abstract List<StaticConf> getSConfs(TimePeriod period, DIOS.Common.SqlManager managerDB,
            int numFrames, ref int count);
    }



    internal class PlainShootingTest : ShootingTestTemplate
    {
        public PlainShootingTest(ShootingType _type) : base(_type)
        {
        }

        protected override List<StaticConf> getSConfs(TimePeriod period, DIOS.Common.SqlManager managerDB,
            int numFrames, ref int count)
        {
            DBTables.DataFetcher fetcher = new DBTables.DataFetcher(managerDB);
 
            double minPause = (double)OptimalChain.Constants.CountMinPause(WorkingType.Shooting, type,
                ShootingChannel.pk, WorkingType.Shooting, type, ShootingChannel.pk) / 1000000;
 
            List<StaticConf> sconfs = new List<StaticConf>();

            DateTime curDtFrom = period.dateFrom;
            while (period.isDtInPeriod(curDtFrom) && count < numFrames)
            {                
                var kaPoint = fetcher.GetSingleSatPoint(curDtFrom).Value;
                SatelliteCoordinates kaPos = new SatelliteCoordinates(kaPoint);
                Polygon framePol = kaPos.ViewPolygon;
                DateTime captureTo = curDtFrom.AddMilliseconds(OptimalChain.Constants.min_shooting_time);
                Order order = new Order();
                order.captured = framePol;
                order.intersection_coeff = 1;
                order.request = new RequestParams(0, 1, DateTime.MinValue, DateTime.MaxValue, 100, 1, 100, 100,
                    framePol.ToWtk(), _shootingType: ShootingType.Normal, _requestChannel: ShootingChannel.pk);
                CaptureConf conf = new CaptureConf(curDtFrom, captureTo, 0, new List<Order>() {order},
                    WorkingType.Shooting, null);
 
                conf.setPolygon(new SatelliteCoordinates(kaPoint).ViewPolygon);
                conf.calculatePitchArrays(kaPoint);
                sconfs.Add(conf.CreateStaticConf(0, 1));
                count++;
                
                double dt = minPause + OptimalChain.Constants.minDeltaT / 1000;
                if (sconfs.Count % 12 == 0)
                    dt += OptimalChain.Constants.MPZ_starting_Time / 1000 + OptimalChain.Constants.MPZ_init_Time / 1000;
                
                curDtFrom = captureTo.AddSeconds(dt);
            }

            return sconfs;
        }
    }
    
    internal class StripShootingTest : ShootingTestTemplate
    {
        public StripShootingTest(ShootingType _type)
            : base(_type)
        {
        }

        protected override List<StaticConf> getSConfs(TimePeriod period, DIOS.Common.SqlManager managerDB,
            int numFrames, ref int count)
        {
            DBTables.DataFetcher fetcher = new DBTables.DataFetcher(managerDB);
 
            double minPause = (double)OptimalChain.Constants.CountMinPause(WorkingType.Shooting, type,
                ShootingChannel.pk, WorkingType.Shooting, type, ShootingChannel.pk) / 1000000;
 
            List<StaticConf> sconfs = new List<StaticConf>();

            double velo = 6740; // [м/с] примерная скорость движения подспутниковой точки
            double lenght = numFrames * 1000; // [м] длина полигона для коридорной съёмки
            double t = lenght / velo;
            DateTime curDtFrom = period.dateFrom;
            DateTime curDtTo = curDtFrom.AddSeconds(t);

            while (period.isDtInPeriod(curDtTo) && count < numFrames)
            {
                var traj = fetcher.GetTrajectorySat(curDtFrom, curDtTo);
                Polygon corPol = SatLane.getRollPitchLanePolygon(traj, 0, 0);

                var req = new RequestParams(0, 1, DateTime.MinValue, DateTime.MaxValue, 100, 1, 100, 100,
                    corPol.ToWtk(), _shootingType: ShootingType.Coridor, _requestChannel: ShootingChannel.pk);
                Order order = new Order();
                order.captured = corPol;
                order.intersection_coeff = 1;
                order.request = new RequestParams(0, 1, DateTime.MinValue, DateTime.MaxValue, 100, 1, 100, 100,
                    corPol.ToWtk(), _shootingType: ShootingType.Normal, _requestChannel: ShootingChannel.pk);
                CaptureConf conf = new CaptureConf(curDtFrom, curDtTo, 0, new List<Order>() { order },
                    WorkingType.Shooting, null);
 
                conf.setPolygon(corPol);
                sconfs.Add(conf.DefaultStaticConf());

                count += (int)((curDtTo - curDtFrom).TotalSeconds * velo / 1000 + 0.5); // отснятые километры

                curDtFrom = curDtTo.AddSeconds(minPause);
                curDtTo = curDtFrom.AddSeconds(t);
            }

            return sconfs;
        }
    }

    internal class StereoShootingTest : ShootingTestTemplate
    {
        public StereoShootingTest(ShootingType _type) : base(_type)
        {
        }

        protected override List<StaticConf> getSConfs(TimePeriod period, DIOS.Common.SqlManager managerDB,
            int numFrames, ref int count)
        {
            DBTables.DataFetcher fetcher = new DBTables.DataFetcher(managerDB);

            double minPause = (double)OptimalChain.Constants.CountMinPause(WorkingType.Shooting, type,
                ShootingChannel.pk, WorkingType.Shooting, type, ShootingChannel.pk) / 1000000;
 
            List<StaticConf> sconfs = new List<StaticConf>();
            
            double pitchAngle = OptimalChain.Constants.stereoPitchAngle;
            double deflectTimeDelta =
                CaptureConf.getTimeDeltaFromPitch(fetcher.GetSingleSatPoint(period.dateFrom).Value, 0, pitchAngle);
            DateTime curDt = period.dateFrom.AddSeconds(deflectTimeDelta); 
            while (period.isDtInPeriod(curDt.AddSeconds(deflectTimeDelta)) && count < numFrames)
            {                
                var kaPoint = fetcher.GetSingleSatPoint(curDt).Value;
                SatelliteCoordinates kaPos = new SatelliteCoordinates(kaPoint);
                Polygon framePol = kaPos.ViewPolygon;
                DateTime captureTo = curDt.AddMilliseconds(OptimalChain.Constants.min_shooting_time);
                Order order = new Order();
                order.captured = framePol;
                order.intersection_coeff = 1;
                order.request = new RequestParams(0, 1, DateTime.MinValue, DateTime.MaxValue, 100, 1, 100, 100,
                    framePol.ToWtk(), _requestChannel: ShootingChannel.pk);
                CaptureConf conf = new CaptureConf(curDt, captureTo, 0, new List<Order>() {order},
                    WorkingType.Shooting, null);
 
                conf.setPolygon(framePol);

                double reconfigureMin = 38;  // секунда на поворот от -stereoPitchAngle до stereoPitchAngle
                curDt = captureTo.AddSeconds(deflectTimeDelta + deflectTimeDelta + reconfigureMin);

                if (!conf.converToStereo(kaPoint, new List<TimePeriod>() { period }, type))                                    
                    continue;

                foreach (var p in conf.pitchArray)
                    sconfs.Add(conf.CreateStaticConf(p.Key, 1));
                count++;
                
                double dt = minPause + OptimalChain.Constants.minDeltaT / 1000;
                if (sconfs.Count > 0 && sconfs.Count % 12 == 0)
                     dt += OptimalChain.Constants.MPZ_starting_Time / 1000 + OptimalChain.Constants.MPZ_init_Time / 1000;
                curDt = curDt.AddSeconds(dt);            
            }
            return sconfs;
        }
    }

    internal class CoridorShootingTest : ShootingTestTemplate
    {
        public CoridorShootingTest(ShootingType _type) : base(_type)
        {
        }

        protected override List<StaticConf> getSConfs(TimePeriod period, DIOS.Common.SqlManager managerDB,
            int numFrames, ref int count)
        {
            DBTables.DataFetcher fetcher = new DBTables.DataFetcher(managerDB);

            double minPause = (double)OptimalChain.Constants.CountMinPause(WorkingType.Shooting, type,
                ShootingChannel.pk, WorkingType.Shooting, type, ShootingChannel.pk) / 1000000;
 
            List<StaticConf> sconfs = new List<StaticConf>();

            double velo = 6740; // [м/с] примерная скорость движения подспутниковой точки
            double lenght = 90000; // [м] длина полигона для коридорной съёмки
            double t = lenght / velo;
            DateTime curDtFrom = period.dateFrom;
            DateTime curDtTo = curDtFrom.AddSeconds(t);
            while (period.isDtInPeriod(curDtTo) && count < numFrames)
            {
                var traj = fetcher.GetTrajectorySat(curDtFrom, curDtTo);
                Polygon corPol = SatLane.getRollPitchLanePolygon(traj, 0, 0);

                List<GeoPoint> line = traj.Points.Select(point => GeoPoint.FromCartesian(point.Position.ToVector()))
                    .ToList();
                List<CoridorParams> coridorParams;
                Sessions.getPiecewiseCoridorParams(curDtFrom, line, managerDB, out coridorParams);
                
                var req = new RequestParams(0, 1, DateTime.MinValue, DateTime.MaxValue, 100, 1, 100, 100,
                    corPol.ToWtk(), _shootingType: ShootingType.Coridor, _requestChannel: ShootingChannel.pk);
                foreach (var cp in coridorParams)
                {
                    double interCoeff = cp.Coridor.Area / req.polygons.First().Area;
                    var orders = new List<Order>()
                    {
                        new Order() {request = req, captured = cp.Coridor, intersection_coeff = interCoeff}
                    };

                    WorkingType confType = WorkingType.Shooting;
                    if (req.compression == OptimalChain.Constants.compressionDropCapture)
                        confType = WorkingType.ShootingSending;
 
                    CaptureConf cc = new CaptureConf(cp.StartTime, cp.EndTime, cp.AbsMaxRequiredRoll, orders,
                        confType, null, _poliCoef: cp.CoridorCoefs);
                    cc.setPolygon(cp.Coridor);
                    //cc.calculatePitchArrays(
                    sconfs.Add(cc.DefaultStaticConf());
                }
                
                count++;

                double dt = minPause + OptimalChain.Constants.minDeltaT / 1000;
                if (sconfs.Count % 12 == 0)
                    dt += OptimalChain.Constants.MPZ_starting_Time / 1000 + OptimalChain.Constants.MPZ_init_Time / 1000;

                curDtFrom = coridorParams.Max(cor => cor.EndTime);
                curDtFrom = curDtFrom.AddSeconds(dt);
                curDtTo = curDtFrom.AddSeconds(t);

            }

            return sconfs;
        }
    }

    internal class AreaShootingTest : ShootingTestTemplate
    {
        public AreaShootingTest(ShootingType _type) : base(_type)
        {
        }

        protected override List<StaticConf> getSConfs(TimePeriod period, DIOS.Common.SqlManager managerDB,
            int numFrames, ref int count)
        {
            DBTables.DataFetcher fetcher = new DBTables.DataFetcher(managerDB);
 
            double minPause = (double)OptimalChain.Constants.CountMinPause(WorkingType.Shooting, type,
                ShootingChannel.pk, WorkingType.Shooting, type, ShootingChannel.pk) / 1000000;
 
            List<StaticConf> sconfs = new List<StaticConf>();

            double pitchAngle = OptimalChain.Constants.stereoPitchAngle;
            double deflectTimeDelta =
                CaptureConf.getTimeDeltaFromPitch(fetcher.GetSingleSatPoint(period.dateFrom).Value, 0,
                    pitchAngle);
            DateTime curDt = period.dateFrom.AddSeconds(deflectTimeDelta);
            while (period.isDtInPeriod(curDt) && count < numFrames)
            {
                double cam_angle = OptimalChain.Constants.camera_angle;
                var kaPoint = fetcher.GetSingleSatPoint(curDt).Value;
                SatelliteCoordinates kaPosLeft = new SatelliteCoordinates(kaPoint, -1.5 * cam_angle, 0);
                SatelliteCoordinates kaPosRight = new SatelliteCoordinates(kaPoint, 1.5 * cam_angle, 0);
                Polygon framePol = new Polygon(new List<Vector3D>()
                    {
                        kaPosLeft.BotLeftViewPoint, kaPosLeft.TopLeftViewPoint,
                        kaPosRight.TopRightViewPoint, kaPosRight.BotRightViewPoint
                    });
                
                DateTime captureTo = curDt.AddMilliseconds(OptimalChain.Constants.min_shooting_time);
                Order order = new Order();
                order.captured = framePol;
                order.intersection_coeff = 1;
                order.request = new RequestParams(0, 1, DateTime.MinValue, DateTime.MaxValue, 100, 1, 100, 100,
                    framePol.ToWtk(), _shootingType: ShootingType.Normal, _requestChannel: ShootingChannel.pk);

                
                CaptureConf conf1 = new CaptureConf(curDt, captureTo, -1.5 * cam_angle, new List<Order>() { order },
                    WorkingType.Shooting, null);                
                CaptureConf conf2 = new CaptureConf(curDt, captureTo, -0.5 * cam_angle, new List<Order>() { order },
                    WorkingType.Shooting, null);
                CaptureConf conf3 = new CaptureConf(curDt, captureTo, 0.5 * cam_angle, new List<Order>() { order },
                    WorkingType.Shooting, null);
                CaptureConf conf4 = new CaptureConf(curDt, captureTo, 1.5 * cam_angle, new List<Order>() { order },
                    WorkingType.Shooting, null);
 

                conf1.setPolygon(new SatelliteCoordinates(kaPoint, -1.5 * cam_angle, 0).ViewPolygon);
                conf2.setPolygon(new SatelliteCoordinates(kaPoint, -0.5 * cam_angle, 0).ViewPolygon);
                conf3.setPolygon(new SatelliteCoordinates(kaPoint,  0.5 * cam_angle, 0).ViewPolygon);
                conf4.setPolygon(new SatelliteCoordinates(kaPoint,  1.5 * cam_angle, 0).ViewPolygon);
                 
                conf1.calculatePitchArrays(kaPoint);
                conf2.calculatePitchArrays(kaPoint);
                conf3.calculatePitchArrays(kaPoint);
                conf4.calculatePitchArrays(kaPoint);

                double shooting_time = (double)OptimalChain.Constants.CountMinPause(WorkingType.Shooting, type,
                ShootingChannel.pk, WorkingType.Shooting, type, ShootingChannel.pk) / 1000000;

                int deltTime = (int)conf1.timeDelta;
                sconfs.Add(conf1.CreateStaticConf(deltTime, -1));

                double min_pause = StaticConf.reConfigureMilisecinds(-1.5 * cam_angle, 0, -0.5 * cam_angle, 0) / 1000;
                deltTime -= (int)(shooting_time + min_pause);
                sconfs.Add(conf2.CreateStaticConf(Math.Abs(deltTime), -1 * Math.Sign(deltTime)));

                min_pause = StaticConf.reConfigureMilisecinds(-0.5 * cam_angle, 0, 0.5 * cam_angle, 0) / 1000;
                deltTime -= (int)(shooting_time + min_pause);
                sconfs.Add(conf3.CreateStaticConf(Math.Abs(deltTime), -1 * Math.Sign(deltTime)));

                min_pause = StaticConf.reConfigureMilisecinds(0.5 * cam_angle, 0, 1.5 * cam_angle, 0) / 1000;
                deltTime -= (int)(shooting_time + min_pause);
                sconfs.Add(conf4.CreateStaticConf(Math.Abs(deltTime), -1 * Math.Sign(deltTime)));

                double dt = minPause + StaticConf.reConfigureMilisecinds(1.5 * cam_angle, 0, -1.5 * cam_angle, 0) / 1000;
                double reconfigureMin = 38;  // секунд на поворот от -pitchAngle до pitchAngle
                dt += 2* deflectTimeDelta + reconfigureMin;

                if (sconfs.Count % 12 == 0)
                    dt += OptimalChain.Constants.MPZ_starting_Time / 1000 + OptimalChain.Constants.MPZ_init_Time / 1000;

                curDt = curDt.AddSeconds(dt);
                
                count++;                
            }

            return sconfs;
        }
    }
}