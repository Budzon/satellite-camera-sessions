using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Windows.Media.Media3D;
using SphericalGeom;
using Common;

using SatelliteTrajectory;
using Astronomy;
using DBTables;

namespace GeometryTest
{
    [TestClass]
    public class RoutinesTest
    {
        private TestContext testContextInstance;
        public TestContext TestContext
        {
            get { return testContextInstance; }
            set { testContextInstance = value; }
        }

        //[TestMethod]
        //public void TestCoridor()
        //{
        //    GeoPoint p = new GeoPoint(0, 0);
        //    Vector3D v = GeoPoint.ToCartesian(p, 1);
        //    GeoPoint p1 = new GeoPoint(-0.5, 0.7);
        //    double b1, b2, l1, l2;
        //    double distMet = 97e3;

        //    bool[] flags = new bool[2] { true, false };
        //    foreach (bool flag in flags)
        //    {
        //        Routines.getGeodesicLineEndPoints(p, p1, out b1, out b2, out l1, out l2);

        //        GeoPoint pl = new GeoPoint(0, -5);
        //        GeoPoint pr = new GeoPoint(0, 5);
        //        Vector3D vl = GeoPoint.ToCartesian(pl, 1);
        //        Vector3D vr = GeoPoint.ToCartesian(pr, 1);

        //        LanePos lpBegin = new LanePos(vl, v, vr, DateTime.Now);

        //        GeoPoint[] leftPoints = new GeoPoint[10];
        //        for (int i = 0; i < leftPoints.Length; ++i)
        //        {
        //            double d = distMet / leftPoints.Length * (i + 1);
        //            leftPoints[i] = new GeoPoint(
        //                AstronomyMath.ToDegrees(AstronomyMath.ToRad(lpBegin.LeftGeoPoint.Latitude) + b1 * d + b2 * d * d),
        //                AstronomyMath.ToDegrees(AstronomyMath.ToRad(lpBegin.LeftGeoPoint.Longitude) + l1 * d + l2 * d * d)
        //            );
        //        }
        //        GeoPoint[] rightPoints = new GeoPoint[10];
        //        for (int i = 0; i < rightPoints.Length; ++i)
        //        {
        //            double d = distMet / rightPoints.Length * (rightPoints.Length - i);
        //            rightPoints[i] = new GeoPoint(
        //                AstronomyMath.ToDegrees(AstronomyMath.ToRad(lpBegin.RightGeoPoint.Latitude) + b1 * d + b2 * d * d),
        //                AstronomyMath.ToDegrees(AstronomyMath.ToRad(lpBegin.RightGeoPoint.Longitude) + l1 * d + l2 * d * d)
        //            );
        //        }

        //        List<GeoPoint> vertices = new List<GeoPoint>();
        //        vertices.Add(lpBegin.LeftGeoPoint);
        //        vertices.AddRange(leftPoints);
        //        vertices.AddRange(rightPoints);
        //        vertices.Add(lpBegin.RightGeoPoint);

        //        Polygon pol = new Polygon(vertices);
        //        string wtk = pol.ToWtk();
        //    }
        //}

        [TestMethod]
        public void TestDivide()
        {
            foreach (var div in Curve.Divide(0, 13, 3))
            {
                foreach (int i in div)
                    Console.Write(i + " ");
                Console.WriteLine("");
            }
        }

        [TestMethod]
        public void TestRollPitch()
        {
            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);
            DBTables.DataFetcher fetcher = new DBTables.DataFetcher(manager);

            double[] rolls = new double[91];
            for (int i = 0; i < rolls.Length; ++i)
                rolls[i] = i - 45;
            double[] pitches = new double[91];
            for (int i = 0; i < pitches.Length; ++i)
                pitches[i] = i - 45;

            DateTime dt1 = new DateTime(2019, 1, 1, 10, 55, 30);
            //TrajectoryPoint tp = fetcher.GetSinglePoint<SatTableFacade>(dt1).Value;

            double roll, pitch;
            double roll1, pitch1;
            bool ok = true, equal;
            for (int s = 0; s < 50; s += 1)
            {
                TrajectoryPoint tp = fetcher.GetSingleSatPoint(dt1).Value;
                for (int i = 0; i < rolls.Length; ++i)
                {
                    roll = AstronomyMath.ToRad(rolls[i]);
                    for (int j = 0; j < pitches.Length; ++j)
                    {
                        pitch = AstronomyMath.ToRad(pitches[j]);
                        //GeoPoint q = Routines.IntersectOpticalAxisAndEarth(tp, roll, pitch);
                        SatelliteCoordinates satCoord = new SatelliteCoordinates(tp, roll, pitch);
                        GeoPoint q = GeoPoint.FromCartesian(satCoord.MidViewPoint);
                        Routines.GetRollPitch(tp, q, out roll1, out pitch1);
                        equal = Comparison.IsZero(Math.Pow(roll - roll1, 1)) && Comparison.IsZero(Math.Pow(pitch - pitch1, 1));
                        ok = ok && equal;
                        //if (!equal)
                        //{
                        //    Console.WriteLine(i + " " + j + " : " + (roll - roll1) + " " + (pitch - pitch1));
                        //}
                    }
                }
                dt1 = dt1.AddSeconds(10);
            }

            Assert.IsTrue(ok);
        }

        [TestMethod]
        public void TestSolveSLE2x3_XY()
        {
            double b1 = 1, b2 = 1;
            Vector3D a1 = new Vector3D(1, 0, 0);
            Vector3D a2 = new Vector3D(0, 2, 0);
            Vector3D solution = Routines.SolveSLE2x3(a1, a2, b1, b2);
            Vector3D actual = new Vector3D(1, 0.5, 0);

            Assert.IsTrue(Comparison.IsZero(solution - actual));
        }

        [TestMethod]
        public void TestSolveSLE2x3_XZ()
        {
            double b1 = 1, b2 = 1;
            Vector3D a1 = new Vector3D(1, 0, 0);
            Vector3D a2 = new Vector3D(0, 0, 2);
            Vector3D solution = Routines.SolveSLE2x3(a1, a2, b1, b2);
            Vector3D actual = new Vector3D(1, 0, 0.5);

            Assert.IsTrue(Comparison.IsZero(solution - actual));
        }

        [TestMethod]
        public void TestSolveSLE2x3_YZ()
        {
            double b1 = 1, b2 = 1;
            Vector3D a1 = new Vector3D(0, 1, 0);
            Vector3D a2 = new Vector3D(0, 0, 2);
            Vector3D solution = Routines.SolveSLE2x3(a1, a2, b1, b2);
            Vector3D actual = new Vector3D(0, 1, 0.5);

            Assert.IsTrue(Comparison.IsZero(solution - actual));
        }

        [TestMethod]
        public void TestSolveSLE2x3_deficient()
        {
            double b1 = 1, b2 = 1;
            Vector3D a1 = new Vector3D(0, 1, 0);
            Vector3D a2 = new Vector3D(0, 2, 0);

            bool deficient = false;
            try
            {
                Routines.SolveSLE2x3(a1, a2, b1, b2);
            }
            catch
            {
                deficient = true;
            }

            Assert.IsTrue(deficient);
        }

        [TestMethod]
        public void TestSolveQuadraticEquation()
        {
            // Two roots
            var res = Routines.SolveQuadraticEquation(1, 0, -1);
            bool twoRoots = (res.Count == 2 && res[0] == -1 && res[1] == 1);

            // One root
            res = Routines.SolveQuadraticEquation(1, -2, 1);
            bool oneRoot = (res.Count == 1 && res[0] == 1);

            // No roots
            res = Routines.SolveQuadraticEquation(1, 0, 1);
            bool noRoots = res.Count == 0;

            Assert.IsTrue(twoRoots && oneRoot && noRoots);
        }

        [TestMethod]
        public void TestFindMaxMin()
        {
            bool sameMaxInside = Math.Abs(Routines.FindMax(Math.Sin, 0, Math.PI) - 1) < 1e-3;
            bool sameMaxBorder = Math.Abs(Routines.FindMax(Math.Cos, 0, Math.PI) - 1) < 1e-3;
            bool sameMinInside = Math.Abs(Routines.FindMin(Math.Sin, -Math.PI, 0) + 1) < 1e-3;
            bool sameMinBorder = Math.Abs(Routines.FindMin(Math.Cos, 0, Math.PI) + 1) < 1e-3;
            Assert.IsTrue(sameMaxBorder && sameMaxInside && sameMinBorder && sameMinInside);
        }

        [TestMethod]
        public void TestIntersectLineUnitSphere_TwoPoints()
        {
            var intersectionTwoPoints = Routines.IntersectLineUnitSphere(
                new Vector3D(1, 0, 0), new Vector3D(-1, 0, 1));
            Vector3D v1 = new Vector3D(1, 0, 0);
            Vector3D v2 = new Vector3D(0, 0, 1);
            bool hasV1 = false;
            bool hasV2 = false;
            int count = 0;

            foreach (Vector3D v in intersectionTwoPoints)
            {
                hasV1 |= Comparison.IsZero(v - v1);
                hasV2 |= Comparison.IsZero(v - v2);
                count += 1;
            }

            Assert.IsTrue(hasV1 && hasV2 && (count == 2));
        }

        [TestMethod]
        public void TestIntersectLineUnitSphere_OnePoint()
        {
            var intersectionOnePoint = Routines.IntersectLineUnitSphere(
                new Vector3D(1, 0, 0), new Vector3D(0, 0, 1));
            Vector3D v1 = new Vector3D(1, 0, 0);
            bool hasV1 = false;
            int count = 0;

            foreach (Vector3D v in intersectionOnePoint)
            {
                hasV1 |= Comparison.IsZero(v - v1);
                count += 1;
            }

            Assert.IsTrue(hasV1 && (count == 1));
        }

        [TestMethod]
        public void TestIntersectLineUnitSphere_ZeroPoints()
        {
            var intersectionZeroPoints = Routines.IntersectLineUnitSphere(
                new Vector3D(2, 0, 0), new Vector3D(0, 0, 1));
            int count = 0;

            foreach (Vector3D v in intersectionZeroPoints)
                count += 1;

            Assert.IsTrue(count == 0);
        }

        [TestMethod]
        public void TestSliceIntoSquaresRect()
        {
            GeoRect rect = new GeoRect(-10, 10, -10, 10);

            // No crop
            List<GeoRect> squares = Routines.SliceIntoSquares(rect, 10);
            bool noCrop = (squares.Count == 4)
                && (squares[0].LowerLeft == new GeoPoint(-10, -10))
                && (squares[0].UpperRight == new GeoPoint(0, 0))
                && (squares[1].LowerLeft == new GeoPoint(-10, 0))
                && (squares[1].UpperRight == new GeoPoint(0, 10))
                && (squares[2].LowerLeft == new GeoPoint(0, -10))
                && (squares[2].UpperRight == new GeoPoint(10, 0))
                && (squares[3].LowerLeft == new GeoPoint(0, 0))
                && (squares[3].UpperRight == new GeoPoint(10, 10));

            // Crop
            squares = Routines.SliceIntoSquares(rect, 15);
            bool crop = (squares.Count == 4)
                && (squares[0].LowerLeft == new GeoPoint(-10, -10))
                && (squares[0].UpperRight == new GeoPoint(5, 5))
                && (squares[1].LowerLeft == new GeoPoint(5, -10))
                && (squares[1].UpperRight == new GeoPoint(10, 5))
                && (squares[2].LowerLeft == new GeoPoint(-10, 5))
                && (squares[2].UpperRight == new GeoPoint(5, 10))
                && (squares[3].LowerLeft == new GeoPoint(5, 5))
                && (squares[3].UpperRight == new GeoPoint(10, 10));

            Assert.IsTrue(noCrop && crop);
        }

        [TestMethod]
        public void TestSliceIntoSquaresPoly()
        {
            double t = 1 / Math.Sqrt(2);
            Vector3D a = new Vector3D(t, 0, t);
            Vector3D b = new Vector3D(0, -t, t);
            Vector3D c = new Vector3D(-t, 0, t);
            Vector3D d = new Vector3D(0, t, t);
            Polygon p = new Polygon(new List<Vector3D> { a, b, c, d });

            List<Polygon> squares = Routines.SliceIntoSquares(p,
                a, Astronomy.AstronomyMath.ToDegrees(Math.Acos(Math.Sqrt(2.0 / 3))), 35.3);
            Assert.IsTrue(squares.Count == 4);
        }
    }
}
