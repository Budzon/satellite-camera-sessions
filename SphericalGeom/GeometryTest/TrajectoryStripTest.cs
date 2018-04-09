using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.Windows.Media.Media3D;

using Microsoft.VisualStudio.TestTools.UnitTesting;


using Common;
using SphericalGeom;
using SatelliteRequests;
using SatelliteTrajectory;
using SatelliteSessions;
using DataParsers;
using Astronomy;

using OptimalChain;
namespace TrajectoryTest
{
    [TestClass]
    public class UnitTest1
    {

        [TestMethod]
        public void TestGetDistToPoint()
        {
            var lp1 = GeoPoint.ToCartesian(new GeoPoint(-1, 2), 1);
            var cp1 = GeoPoint.ToCartesian(new GeoPoint(0, 2), 1);
            var rp1 = GeoPoint.ToCartesian(new GeoPoint(1, 2), 1);

            var lp2 = GeoPoint.ToCartesian(new GeoPoint(-1, -2), 1);
            var cp2 = GeoPoint.ToCartesian(new GeoPoint(0, -2), 1);
            var rp2 = GeoPoint.ToCartesian(new GeoPoint(1, -2), 1);

            var mp = new GeoPoint(0, 0);

            DateTime dt1 = new DateTime();
            DateTime dt2 = new DateTime();

            LanePos lanePos1 = new LanePos(lp1, cp1, rp1, dt1);
            LanePos lanePos2 = new LanePos(lp2, cp2, rp2, dt2);

            double dist = AstronomyMath.ToDegrees(GeoPoint.DistanceOverSurface(lanePos1.GeoKAPoint, lanePos2.GeoKAPoint));
            double dist2 = AstronomyMath.ToDegrees(lanePos1.getDistToPoint(lanePos2.GeoKAPoint));

            Assert.IsTrue(Math.Abs(dist - dist2) < 0.000000000001); 

            var mdist1 = AstronomyMath.ToDegrees(lanePos1.getDistToPoint(mp));
            var mdist2 = AstronomyMath.ToDegrees(lanePos2.getDistToPoint(mp));

            Console.WriteLine("Math.Abs(dist - mdist2 - mdist1) = {0} ", Math.Abs(dist - mdist2 - mdist1));

            Assert.IsTrue(Math.Abs(mdist2 - mdist1) < 0.000000000001);
            Assert.IsTrue(Math.Abs(dist - mdist2 - mdist1) < 0.000000000001);
             
        }


        //DateTime getInterpolTime(DateTime dt1, DateTime dt2, double dist, double fullDist)
        //{
        //    double fullTime13 = Math.Abs((dt1 - dt2).TotalMilliseconds);
        //    double diffMiliSecs = fullTime13 * dist / fullDist;
        //    var newTime = dt1.AddMilliseconds(diffMiliSecs);
        //    return newTime;
        //}
 
  
        //[TestMethod]
        //public void TestGetPointTime()
        //{
        //    /// Суть теста:
        //    /// Создаем те же полосы, которые используются в проде (то есть от -45 до 45 с шагом 1)
        //    /// НО пропускаем каждую вторую точку, то есть ухудшаем точность. То есть в проде 
        //    /// точность будет ВЫШЕ, чем насчитает этот тест.
        //    /// Для каждой такой полосы создаем полосу для тестирования, которая "сдвинута" относительно
        //    /// тестируемой полосы на угол difAngle ( причем по замыслу  difAngle принадлежит (0, 2*viewAngle) )
        //    /// Из второй полосы нам нужны только правые точки, так как они находятся аккурат в тестируемой полосе.
        //    /// далее мы для каждой правой точки из вспомогательной полосы считаем время функцией getPointTime,
        //    /// сверяя с заданным временем этой точки. 
        //    /// 
        //    /// ТЕСТ МОЖЕТ ЗАНИМАТЬ СЕКУНД 10-40, ЭТО НОРМАЛЬНО.
            
        //    DateTime begDt = new DateTime(2000, 1, 1);
        //    DateTime endDt = new DateTime(2020, 1, 1);

        //    string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "\\" + "trajectory_5hours.dat"; // "trajectory_full.dat";
        //    Trajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, begDt, endDt); // @todo временно

        //    double maxError = 0; // сюда будет записана максимальная найденная ошибка
        //    int countErrors = 0; // сюда будет записано колво ошибок, превышающих errorLimit
        //    int errorLimit = 3;  // максимально допустмая ошибка (в мс), при превышении которой сгенерируется исключение
 
        //    double viewAngle = AstronomyMath.ToRad(1);
        //    double difAngle = viewAngle * 0.9; // угол, на который мы сдвигаем тестовую полосу.  (0, 2*viewAngle)
        //    for (double rollAngle = AstronomyMath.ToRad(-45);
        //                rollAngle < AstronomyMath.ToRad(45) - viewAngle;
        //                rollAngle += viewAngle)
        //    {
        //        //  основная (тестируемая) полоса 
        //        SatLane lane = new SatLane(trajectory, rollAngle, viewAngle);

        //        // вспомогательная полоса, отсюда нужны только правые точки
        //        var controlLane = new SatLane(trajectory, rollAngle + viewAngle, viewAngle);

        //        var clpssd = controlLane.Sectors[0].sectorPoints[0].TopLeftViewPoint;

        //        var clpssd2 = controlLane.Sectors[0].sectorPoints[0].TopLeftViewPoint;

        //        foreach (var control_sector in controlLane.Sectors)
        //        {
        //            foreach (var ctrl_point in control_sector.sectorPoints)
        //            {
        //                var point = ctrl_point.RightCartPoint;

        //                DateTime calc_time = new DateTime();
        //                foreach (var sector in lane.Sectors)
        //                {
        //                    if (sector.fromDT <= ctrl_point.Time && ctrl_point.Time <= sector.toDT)
        //                    {
        //                        calc_time = sector.getPointTime(point);
        //                    }
        //                }

        //                double diff_msecs = (ctrl_point.Time - calc_time).TotalMilliseconds;
        //                //  Console.WriteLine("diff_msecs = {0}", diff_msecs);
        //                if (maxError < diff_msecs)
        //                    maxError = diff_msecs;
        //                if (Math.Abs(diff_msecs) >= errorLimit)
        //                {
        //                    countErrors++;
        //                }
        //            }
        //        }

        //    }

        //    Console.WriteLine("максимальная ошибка = {0} мс", maxError);
        //    Console.WriteLine("{0} ошибок, превышающих {1} мс", countErrors, errorLimit);

        //    Assert.IsTrue(countErrors == 0);
            
        //    /*

        //    double viewAngle = AstronomyMath.ToRad(10);  
        //    double rollAngle = AstronomyMath.ToRad(0);
            
        //    // максимально детализированная полоса
        //    SatLane viewLaneFull = trajectory.getCaptureLane(rollAngle, viewAngle, readStep: 1, polygonStep: 1);

        //    // полоса, в которой точек меньше в 16 раз
        //    SatLane viewLane = trajectory.getCaptureLane(rollAngle, viewAngle, readStep: 1, polygonStep: 15);

        //    double maxError = 0;
        //    int countErrors = 0;
        //    int errorLimit = 3;
 
        //    foreach (var orig_sector in viewLaneFull.Sectors)
        //    {
        //        foreach (var orig_point in orig_sector.sectorPoints)
        //        {
        //            var point = orig_point.MiddleCartPoint;

        //            DateTime calc_time = new DateTime();
        //            foreach (var sector in viewLane.Sectors)
        //            {
        //                if (sector.fromDT <= orig_point.time && orig_point.time <= sector.toDT)
        //                {
        //                    calc_time = sector.getPointTime(point);
        //                }
        //            }

        //            var diff_msecs = (orig_point.time - calc_time).TotalMilliseconds;
        //            if (maxError < diff_msecs)
        //                maxError = diff_msecs;
        //            if (Math.Abs(diff_msecs) >= errorLimit)
        //            {
        //                countErrors++;
        //            }
        //        }
        //    }

        //    Console.WriteLine("максимальная ошибка = {0}", maxError);
        //    Console.WriteLine("{0} ошибок, превышающих {1}", countErrors, errorLimit);
            
        //    if (countErrors > 0)
        //        throw new System.Exception("TEST_getPointTime falied!");
        //    */

        //    // Console.WriteLine("viewLaneFull.size = {0}, viewLane.size = {1}", viewLaneFull.lanePoints.Count, viewLane.lanePoints.Count);
 
        //    /*
           
        //    var lan1 = viewLaneFull.Sectors[1].sectorPoints[69];
        //    var lan2 = viewLaneFull.Sectors[1].sectorPoints[70];
        //    var lan3 = viewLaneFull.Sectors[1].sectorPoints[71];

        //    var distLeft12 = Math.Abs(lan1.getDistToPoint(lan2.LeftGeoPoint));
        //    var distLeft23 = Math.Abs(lan2.getDistToPoint(lan3.LeftGeoPoint));
        //    var distLeft13 = Math.Abs(lan1.getDistToPoint(lan3.LeftGeoPoint)); 
        //    Console.WriteLine("leftError = {0} %", Math.Abs(distLeft13 - (distLeft12 + distLeft23)) / distLeft13 * 100);
             
        //    //var newTime = getInterpolTime(lan1.time, lan3.time, distLeft12, distLeft13);
        //    //double errorTime = (lan2.time - newTime).TotalMilliseconds;
        //    //Console.WriteLine("errorTime2 = {0} ms", errorTime);

        //    var distRight12 = Math.Abs(lan1.getDistToPoint(lan2.RightGeoPoint));
        //    var distRight23 = Math.Abs(lan2.getDistToPoint(lan3.RightGeoPoint));
        //    var distRight13 = Math.Abs(lan1.getDistToPoint(lan3.RightGeoPoint));

        //    Console.WriteLine("RightError = {0} %", Math.Abs(distRight13 - (distRight12 + distRight23)) / distRight13 * 100);

        //    Console.WriteLine("LeftRightError = {0} %", Math.Abs(distRight12 - distLeft12) / distRight12 * 100);


        //    var spdLeft = GeoPoint.DistanceOverSurface(lan1.LeftGeoPoint, lan2.LeftGeoPoint);
        //    var spdRight = GeoPoint.DistanceOverSurface(lan1.RightGeoPoint, lan2.RightGeoPoint);
        //    Console.WriteLine("LeftRightErrorOverSurface = {0} %", Math.Abs(spdRight - spdLeft) / spdRight * 100);

        //    var distMid12 = Math.Abs(lan1.getDistToPoint(lan2.MiddleGeoPoint));
        //    var distMid23 = Math.Abs(lan2.getDistToPoint(lan3.MiddleGeoPoint));
        //    var distMid13 = Math.Abs(lan1.getDistToPoint(lan3.MiddleGeoPoint));
        //    Console.WriteLine("midError = {0} %", Math.Abs(distMid13 - (distMid12 + distMid23)) / distMid13 * 100);

        //    var newTime = getInterpolTime(lan1.time, lan3.time, distMid12, distMid13);
        //    double errorTime = (lan2.time - newTime).TotalMilliseconds;
        //    Console.WriteLine("MidErrorTime2 = {0} ms", errorTime);

        //    var selfMiddleDist = Math.Abs(lan1.getDistToPoint(lan1.MiddleGeoPoint));
        //    Console.WriteLine("selfMiddleDist in degr = {0}", AstronomyMath.ToDegrees(selfMiddleDist));
            

        //    //var selfMidTime = getInterpolTime(lan1.time, lan2.time, selfMiddleDist, distMid12);
        //    double fullDist = Math.Abs(lan1.getDistToPoint(lan1.MiddleGeoPoint) + lan2.getDistToPoint(lan1.MiddleGeoPoint));

        //    double fullTime13 = Math.Abs((lan1.time - lan2.time).TotalMilliseconds);
        //    double diffMiliSecs = fullTime13 * selfMiddleDist / distMid12;
        //    var selfMidTime = lan1.time.AddMilliseconds(diffMiliSecs);
        //    double errorSelfMidTime = (lan1.time - selfMidTime).TotalMilliseconds;
        //    Console.WriteLine();
        //    Console.WriteLine("errorSelfMidTime = {0} ms", errorSelfMidTime);
        //    Console.WriteLine("fullDist = {0} ", fullDist);
        //    Console.WriteLine("selfMiddleDist = {0} ", selfMiddleDist);
        //    Console.WriteLine("selfMiddleDist / fullDist = {0} ", selfMiddleDist / fullDist);
        //    Console.WriteLine();
             

        //    var selfRightDist = Math.Abs(lan1.getDistToPoint(lan1.RightGeoPoint));
        //    Console.WriteLine("selfRightDist in degr = {0}", AstronomyMath.ToDegrees(selfRightDist));

        //    var spdRL = GeoPoint.DistanceOverSurface(lan1.LeftGeoPoint, lan1.RightGeoPoint);
        //    var spdRM = GeoPoint.DistanceOverSurface(lan1.RightGeoPoint, lan1.MiddleGeoPoint);
        //    var spdML = GeoPoint.DistanceOverSurface(lan1.MiddleGeoPoint, lan1.LeftGeoPoint);
        //    Console.WriteLine("Error_Lane_OverSurface = {0} %", Math.Abs(spdRL - (spdRM + spdML)) / spdRL * 100);

        //    return;
        //    */
            
        //}


    }
}
