using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Windows.Media.Media3D;
using SphericalGeom;
using Common;

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
            bool sameMinInside = Math.Abs(Routines.FindMin(Math.Sin, -Math.PI, 0) + 1)< 1e-3;
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
                a, Astronomy.AstronomyMath.ToDegrees(Math.Acos(Math.Sqrt(2.0/3))), 35.3);
            Assert.IsTrue(squares.Count == 4);
        }
    }
}
