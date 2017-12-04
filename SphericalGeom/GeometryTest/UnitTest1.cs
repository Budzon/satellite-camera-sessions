using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Collections.Generic;
using System.Linq;
using SphericalGeom;

namespace GeometryTest
{
    public static class CompareVec
    {
        public static bool AreEqual(vector3 a, vector3 b)
        {
            return Comparison.IsZero((a - b).Length());
        }

        public static bool IsZero(vector3 a)
        {
            return Comparison.IsZero(a.Length());
        }
    }

    [TestClass]
    public class TestReferenceFrame
    {
        [TestMethod]
        public void TestConstructor()
        {
            vector3 oy = new vector3(1, 0, 0);
            ReferenceFrame actual = new ReferenceFrame(oy);
            ReferenceFrame target = new ReferenceFrame(
                new vector3(0, -1, 0),
                new vector3(1, 0, 0),
                new vector3(0, 0, 1));

            Assert.IsTrue(CompareVec.AreEqual(actual.OX, target.OX) && CompareVec.AreEqual(actual.OY, target.OY) && CompareVec.AreEqual(actual.OZ, target.OZ));
        }

        [TestMethod]
        public void TestMatVec()
        {
            ReferenceFrame frame = new ReferenceFrame(new vector3(1, 0, 0));
            vector3 target = new vector3(2, -1, 3);
            vector3 actual = frame.ToBaseFrame(1, 2, 3);

            Assert.IsTrue(CompareVec.AreEqual(target, actual));
        }
    }

    [TestClass]
    public class TestCamera
    {
        private TestContext testContextInstance;

        /// <summary>
        ///  Gets or sets the test context which provides
        ///  information about and functionality for the current test run.
        ///</summary>
        public TestContext TestContext
        {
            get { return testContextInstance; }
            set { testContextInstance = value; }
        }

        [TestMethod]
        public void TestCanBeSeenFromPosition()
        {
            Camera camera = new Camera();
            var privateCamera = new PrivateObject(camera);

            vector3 a = new vector3(new direction3(Math.PI / 4, 0), 1);
            vector3 b = new vector3(new direction3(0, Math.PI / 2.5), 1);

            bool aSeen = (bool)(privateCamera.Invoke("CanBeSeenFromPosition", a));
            bool bSeen = (bool)(privateCamera.Invoke("CanBeSeenFromPosition", b));

            Assert.IsTrue(aSeen && !bSeen);
        }

        [TestMethod]
        public void TestGetPositiveNormalsToSightBoundaries()
        {
            Camera camera = new Camera();
            var privateCamera = new PrivateObject(camera);
            ReferenceFrame frame = new ReferenceFrame(
                new vector3(0, 1, 0),
                new vector3(-1, 0, 0),
                new vector3(0, 0, 1)
                );

            vector3 n1 = new vector3(new direction3(Math.PI / 3, Math.PI), 1);
            vector3 n3 = new vector3(new direction3(-Math.PI / 3, Math.PI), 1);
            vector3 n2 = new vector3(new direction3(0, 2 * Math.PI / 3), 1);
            vector3 n4 = new vector3(new direction3(0, -2 * Math.PI / 3), 1);

            List<vector3> target = new List<vector3>() { n1, n2, n3, n4 };
            List<vector3> actual = privateCamera.Invoke("GetPositiveNormalsToSightBoundaries", frame,0) as List<vector3>;

            Assert.IsTrue(target.Zip(actual, (a, b) => a - b).All(vec => CompareVec.IsZero(vec)));
        }

        [TestMethod]
        public void TestRegionInPositiveHalfspace()
        {
            Camera camera = new Camera(new vector3(2, 0, 0.5), Math.PI / 6, Math.PI / 6);
            var privateCamera = new PrivateObject(camera);
            vector3 normal = new vector3(0, 0, 1);
            vector3 a = new vector3(0, 0, 1);
            vector3 b = new vector3(0, 0, 0.2);

            bool res = (bool)(privateCamera.Invoke("RegionInPositiveHalfspace", normal, new List<vector3>() { a, b }));
            Assert.IsFalse(res);
        }
    }

    [TestClass]
    public class TestSphericalGeometryRoutines
    {
        private TestContext testContextInstance;
        public TestContext TestContext
        {
            get { return testContextInstance; }
            set { testContextInstance = value; }
        }

        //[TestMethod]
        //public void TestIntersectSmallArcGreatArc()
        //{
        //    double pi = Math.PI;
        //    direction3 smallA = new direction3(pi/6, pi/3);
        //    direction3 smallB = new direction3(pi/6, -pi/3);
        //    direction3 smallCenter = new direction3(pi/2, 0);
        //    direction3 greatA = new direction3(pi/6, pi/3);
        //    direction3 greatB = new direction3(pi/6, -pi/3);
            
        //    vector3 target1 = new vector3(greatA, 1);
        //    vector3 target2 = new vector3(greatB, 1);
        //    List<vector3> actual = SphericalGeometryRoutines.IntersectSmallArcGreatArc(
        //        new vector3(smallA, 1), new vector3(smallB, 1), new vector3(smallCenter, 0.5),
        //        new vector3(greatA, 1), new vector3(greatB, 1));
            
        //    Assert.IsTrue(actual.Count == 2 
        //        && CompareVec.AreEqual(target1, actual[0])
        //        && CompareVec.AreEqual(target2, actual[1]));
        //}

        [TestMethod]
        public void TestSolveSLE2x3()
        {
            vector3 a1 = new vector3(1, 0, 1);
            vector3 a2 = new vector3(0, 2, 0);
            double b1 = 1;
            double b2 = 1;
            vector3 target = new vector3(1, 0.5, 0);
            vector3 actual = SphericalGeometryRoutines.SolveSLE2x3(a1, a2, b1, b2);

            Assert.IsTrue(CompareVec.AreEqual(target, actual));
        }

        [TestMethod]
        public void TestSolveQuadraticEquation()
        {
            var res = SphericalGeometryRoutines.SolveQuadraticEquation(1, 0, -1);
            Assert.IsTrue(res.Count == 2 && res[0] == -1 && res[1] == 1);
        }
    }

    [TestClass]
    public class TestPolygon
    {
        private TestContext testContextInstance;
        public TestContext TestContext
        {
            get { return testContextInstance; }
            set { testContextInstance = value; }
        }

        [TestMethod]
        public void TestContains()
        {
            double pi = Math.PI;
            direction3 a = new direction3(pi/60, 0);
            direction3 b = new direction3(-pi/60, pi/100);
            direction3 c = new direction3(-pi/60, -pi/100);
            direction3 p = new direction3(pi/59, 0);

            List<vector3> points = new List<vector3>{ new vector3(a, 1), new vector3(b, 1), new vector3(c, 1) };

            var poly = new Polygon(points, new vector3(0, 0, 0));
            Assert.IsTrue(!poly.Contains(new vector3(p, 1)));
        }

        [TestMethod]
        public void TestIntersect()
        {
            double pi = Math.PI;
            vector3 a = new vector3(new direction3(-pi / 10, -pi / 10), 1);
            vector3 b = new vector3(new direction3( pi / 10, -pi / 10), 1);
            vector3 c = new vector3(new direction3( pi / 10,  pi / 10), 1);
            vector3 d = new vector3(new direction3(-pi / 10,  pi / 10), 1);
            var p = new Polygon(new List<vector3> { a, b, c, d }, new vector3(0, 0, 0));

            vector3 A = new vector3(new direction3(-pi / 6, -pi / 8), 1);
            vector3 B = new vector3(new direction3( pi / 5,       0), 1);
            vector3 C = new vector3(new direction3(-pi / 6,  pi / 8), 1);
            var q = new Polygon(new List<vector3> { A, B, C }, new vector3(0, 0, 0));

            var r = Polygon.IntersectAndSubtract(p, q);
            Assert.IsTrue(true);
        }
    }
}
