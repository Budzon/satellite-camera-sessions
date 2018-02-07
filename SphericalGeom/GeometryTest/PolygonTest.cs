using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Windows.Media.Media3D;
using SphericalGeom;
using Common;

namespace GeometryTest
{
    [TestClass]
    public class PolygonTest
    {
        private TestContext testContextInstance;
        public TestContext TestContext
        {
            get { return testContextInstance; }
            set { testContextInstance = value; }
        }

        [TestMethod]
        public void TestHemishphere()
        {
            Polygon hs = Polygon.Hemisphere(new Vector3D(1, 0, 0));

            Assert.IsTrue(hs.Contains(new Vector3D(1, 0, 0))
                && !hs.Contains(new Vector3D(-1, 0, 0)));
        }

        [TestMethod]
        public void TestContains_Yes()
        {
            // 1km on the equator corresponds to 1e-2 degrees
            Vector3D a = GeoPoint.ToCartesian(new GeoPoint(1e-5, 0), 1);
            Vector3D b = GeoPoint.ToCartesian(new GeoPoint(0, 1e-5), 1);
            Vector3D c = GeoPoint.ToCartesian(new GeoPoint(-1e-5, 0), 1);
            Vector3D d = GeoPoint.ToCartesian(new GeoPoint(0, -1e-5), 1);
            Polygon p = new Polygon(new List<Vector3D> { a, b, c, d});

            Assert.IsTrue(p.Contains(new Vector3D(1, 0, 0)));
        }

        [TestMethod]
        public void TestContains_NoPrecision()
        {
            // 1km on the equator corresponds to 1e-2 degrees
            Vector3D a = GeoPoint.ToCartesian(new GeoPoint(1e-5, 0), 1);
            Vector3D b = GeoPoint.ToCartesian(new GeoPoint(0, 1e-5), 1);
            Vector3D c = GeoPoint.ToCartesian(new GeoPoint(-1e-5, 0), 1);
            Vector3D d = GeoPoint.ToCartesian(new GeoPoint(0, -1e-5), 1);
            Polygon p = new Polygon(new List<Vector3D> { a, b, c, d });

            Assert.IsFalse(p.Contains(GeoPoint.ToCartesian(new GeoPoint(1.0001e-5, 0), 1)));
        }

        [TestMethod]
        public void TestContains_Vertex()
        {
            Vector3D a = GeoPoint.ToCartesian(new GeoPoint(1e-5, 0), 1);
            Vector3D b = GeoPoint.ToCartesian(new GeoPoint(0, 1e-5), 1);
            Vector3D c = GeoPoint.ToCartesian(new GeoPoint(-1e-5, 0), 1);
            Vector3D d = GeoPoint.ToCartesian(new GeoPoint(0, -1e-5), 1);
            Polygon p = new Polygon(new List<Vector3D> { a, b, c, d });

            Assert.IsTrue(p.Contains(a));
        }

        [TestMethod]
        public void TestContains_PointOnEdge()
        {
            Vector3D a = GeoPoint.ToCartesian(new GeoPoint(1e-5, 1e-5), 1);
            Vector3D b = GeoPoint.ToCartesian(new GeoPoint(1e-5, -1e-5), 1);
            Vector3D c = GeoPoint.ToCartesian(new GeoPoint(-1e-5, -1e-5), 1);
            Vector3D d = GeoPoint.ToCartesian(new GeoPoint(-1e-5, 1e-5), 1);
            Polygon p = new Polygon(new List<Vector3D> { a, b, c, d });

            Assert.IsTrue(p.Contains(GeoPoint.ToCartesian(new GeoPoint(0, 1e-5), 1)));
        }

        [TestMethod]
        public void TestIsCounterclockwise()
        {
            Vector3D a = GeoPoint.ToCartesian(new GeoPoint(5, 0), 1);
            Vector3D b = GeoPoint.ToCartesian(new GeoPoint(0, 5), 1);
            Vector3D c = GeoPoint.ToCartesian(new GeoPoint(-5, 0), 1);
            Vector3D d = GeoPoint.ToCartesian(new GeoPoint(0, -5), 1);
            Polygon p = new Polygon(new List<Vector3D> { a, b, c, d });
            Polygon q = new Polygon(new List<Vector3D> { c, b, a, d });

            Assert.IsTrue(!p.IsCounterclockwise && q.IsCounterclockwise);
        }

        [TestMethod]
        public void TestToWTK_Band()
        {
            List<Vector3D> verts = new List<Vector3D>
            {
                new Vector3D(-0.852034459185435, 0.120130037485442, -0.509515509532664 ),
                new Vector3D(-0.972504632989056, -0.156405344939802, -0.172545955875771 ),
                new Vector3D(-0.549501530291284, -0.609902051020679, 0.571023253789464 ),
                new Vector3D(-0.455742357633653, -0.633482054296441, 0.625299440542594 ),
                new Vector3D(-0.469679375963483, -0.56027574121427, 0.68227001810233 ),
                new Vector3D(-0.564279557617797, -0.536416675779779, 0.627571295392033 ),
                new Vector3D(-0.989735755317796, -0.0808817092973809, -0.11781885989368 ),
                new Vector3D(-0.869933396998213, 0.194610736332144, -0.453147377893121 ),
            };
            Polygon lanepol = new Polygon(verts);
            try
            {
                Polygon pp = new Polygon(lanepol.ToWtk());
                Assert.IsTrue(true);
            }
            catch
            {
                Assert.IsTrue(false);
            }
        }

        [TestMethod]
        public void TestMiddle_Empty()
        {
            Polygon p = new Polygon();
            bool noMiddle = false;
            try
            {
                var v = p.Middle;
            }
            catch
            {
                noMiddle = true;
            }

            Assert.IsTrue(noMiddle);
        }

        [TestMethod]
        public void TestMiddle()
        {
            Vector3D a = GeoPoint.ToCartesian(new GeoPoint(5, 0), 1);
            Vector3D b = GeoPoint.ToCartesian(new GeoPoint(0, 5), 1);
            Vector3D c = GeoPoint.ToCartesian(new GeoPoint(-5, 0), 1);
            Vector3D d = GeoPoint.ToCartesian(new GeoPoint(0, -5), 1);
            Polygon p = new Polygon(new List<Vector3D> { a, b, c, d });

            Vector3D mid = p.Middle;
            mid.Normalize();

            Assert.IsTrue(Comparison.IsZero(mid - new Vector3D(1, 0, 0)));
        }

        [TestMethod]
        public void TestArea_Geodesic()
        {
            Vector3D a = new Vector3D(0, 0, 1);
            Vector3D b = new Vector3D(0, 1, 0);
            Vector3D c = new Vector3D(1, 0, 0);
            Polygon p = new Polygon(new List<Vector3D> { a, b, c });

            Assert.IsTrue(Comparison.IsEqual(p.Area, Math.PI / 2));
        }

        [TestMethod]
        public void TestArea_Cap()
        {
            double s = 0.5, t = Math.Sqrt(3) / 2;
            Vector3D a = new Vector3D(s, t, 0);
            Vector3D b = new Vector3D(s, 0, t);
            Vector3D c = new Vector3D(s, -t, 0);
            Vector3D d = new Vector3D(s, 0, -t);
            Polygon p = new Polygon(new List<Vector3D> { a, b, c, d },
                new Vector3D(s, 0, 0));
            
            Assert.IsTrue(Comparison.IsEqual(p.Area, Math.PI));
        }

        [TestMethod]
        public void TestArea_GeoRect()
        {
            Vector3D a = GeoPoint.ToCartesian(new GeoPoint(30, 30), 1);
            Vector3D b = GeoPoint.ToCartesian(new GeoPoint(-30, 30), 1);
            Vector3D c = GeoPoint.ToCartesian(new GeoPoint(-30, -30), 1);
            Vector3D d = GeoPoint.ToCartesian(new GeoPoint(30, -30), 1);
            Polygon p = new Polygon(new List<Vector3D> { a, b, c, d },
                new List<Vector3D>
                {
                    new Vector3D(0, 0, 0),
                    new Vector3D(0, 0, -0.5),
                    new Vector3D(0, 0, 0),
                    new Vector3D(0, 0, 0.5)
                });

            Assert.IsTrue(Comparison.IsEqual(p.Area, Math.PI / 3));
        }

        [TestMethod]
        public void TestBoundingBox()
        {
            Vector3D a = GeoPoint.ToCartesian(new GeoPoint(30, 30), 1);
            Vector3D b = GeoPoint.ToCartesian(new GeoPoint(-30, 30), 1);
            Vector3D c = GeoPoint.ToCartesian(new GeoPoint(-30, -30), 1);
            Vector3D d = GeoPoint.ToCartesian(new GeoPoint(30, -30), 1);
            Polygon p = new Polygon(new List<Vector3D> { a, b, c, d },
                new List<Vector3D>
                {
                    new Vector3D(0, 0, 0),
                    new Vector3D(0, 0, -0.5),
                    new Vector3D(0, 0, 0),
                    new Vector3D(0, 0, 0.5)
                });

            GeoRect bbox = p.BoundingBox();
            Assert.IsTrue(Comparison.IsEqual(bbox.LeftLongitude, -30 * (1+1e-3))
                && Comparison.IsEqual(bbox.BottomLatitude, -30 * (1+1e-3))
                && Comparison.IsEqual(bbox.RightLongitude, 30 * (1+1e-3))
                && Comparison.IsEqual(bbox.TopLatitude, 30 * (1+1e-3)));
        }
       
        //[TestMethod]
        //public void TestIntersect()
        //{
        //    double pi = Math.PI;
        //    vector3 a = new vector3(new direction3(-pi / 10, -pi / 10), 1);
        //    vector3 b = new vector3(new direction3(pi / 10, -pi / 10), 1);
        //    vector3 c = new vector3(new direction3(pi / 10, pi / 10), 1);
        //    vector3 d = new vector3(new direction3(-pi / 10, pi / 10), 1);
        //    var p = new Polygon(new List<vector3> { a, b, c, d }, new vector3(0, 0, 0));

        //    vector3 A = new vector3(new direction3(-pi / 6, -pi / 8), 1);
        //    vector3 B = new vector3(new direction3(pi / 5, 0), 1);
        //    vector3 C = new vector3(new direction3(-pi / 6, pi / 8), 1);
        //    var q = new Polygon(new List<vector3> { A, B, C }, new vector3(0, 0, 0));

        //    var r = Polygon.IntersectAndSubtract(p, q);
        //    Assert.IsTrue(true);
        //}
    }
}
