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

        //[TestMethod]
        //public void TestHemishphere()
        //{
        //    Polygon hs = Polygon.Hemisphere(new Vector3D(1, 0, 0));

        //    Assert.IsTrue(hs.Contains(new Vector3D(1, 0, 0))
        //        && !hs.Contains(new Vector3D(-1, 0, 0)));
        //}

        [TestMethod]
        public void JustTest()
        {
            List<Vector3D> verts = new List<Vector3D>
            {
                new Vector3D(-3510.84901982254, 2359.31802630575, 4764.55887787735),
                new Vector3D(-3517.73398705391,2368.94874544631,4754.69169639569),
                new Vector3D(-3521.30591868274,2385.89468552929,4743.56141170461),
                new Vector3D(-3525.18541989088,2401.81304896715,4732.63409986446),
                new Vector3D(-3529.38476882757,2416.70609497005,4721.91047078432),
                new Vector3D(-3533.91529957537,2430.57598758813,4711.39116546134),
                new Vector3D(-3538.78741484062,2443.42477427707,4701.07675910686),
                new Vector3D(-3544.01059900414,2455.25436653388,4690.96776366696),
                new Vector3D(-3549.5934314627,2466.06652257568,4681.06462971281),
                new Vector3D(-3549.00195957781,2466.47868069662,4681.29594221322),
                new Vector3D(-3555.37879273055,2475.31231086993,4671.78491519651),
                new Vector3D(-3561.8679154135,2484.64470242974,4661.87744860755),
                new Vector3D(-3550.48918474364,2492.4162229667,4666.4067129335),
                new Vector3D(-3544.13405538794,2483.63138821884,4675.91195542577),
                new Vector3D(-3531.71119162079,2464.40028199154,4695.44577213926),
                new Vector3D(-3526.03737466925,2453.54841804236,4705.38247996456),
                new Vector3D(-3520.71867916329,2441.68710128458,4715.52412480428),
                new Vector3D(-3515.74654217448,2428.81488901234,4725.87009725195),
                new Vector3D(-3511.11140327418,2414.93027607774,4736.41976238182),
                new Vector3D(-3506.80269328301,2400.0317093898,4747.17245887555),
                new Vector3D(-3502.8088228672,2384.11760385366,4758.12749844293),
                new Vector3D(-3499.11717106332,2367.18635986284,4769.28416545388)
            };

            Polygon p = new Polygon(verts);
            //p.SetAreaOrientation();
            Console.WriteLine(p.Area + " " + p.IsCounterclockwise);
            Console.WriteLine(p.ToWtk());
            Console.WriteLine(verts[0]);
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

            //p.SetAreaOrientation();
            //q.SetAreaOrientation();

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
                //Polygon pp = new Polygon(lanepol.ToWtk());
                lanepol.ToWtk();
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
