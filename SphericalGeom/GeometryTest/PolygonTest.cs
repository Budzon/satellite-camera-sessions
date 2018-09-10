using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Windows.Media.Media3D;
using SphericalGeom;
using Common;

namespace GeometryTest
{
    /*
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
        public void TestHoles()
        {
            string outPol = "POLYGON((35.760498046875 56.662264768247184,34.573974609375 54.68018309710001,42.242431640625 54.47642158429295,42.71484375 56.67433841615883,35.760498046875 56.662264768247184))";

            List<string> holes = new List<string>() {
                "POLYGON((36.5185546875 56.1333069123757,36.683349609375 56.492827145026666,36.92504882812501 56.365250136856105,37.430419921875 56.492827145026666,37.177734375 56.035225783698735,37.02392578125 56.06590296330043,37.06787109375001 56.26776108757582,36.815185546875 56.243349924105246,36.705322265625 56.07203547180089,36.5185546875 56.1333069123757))" 
                ,"POLYGON((37.89184570312501 56.42605447604976,37.628173828125 56.004524201154,37.84790039062501 55.973798205076605,38.04565429687501 56.365250136856105,38.56201171875 56.3409012041991,38.583984375 56.4199784113855,37.89184570312501 56.42605447604976))" 
                ,"POLYGON((38.91357421875 56.365250136856105,39.166259765625 56.12718415613108,38.792724609375 55.930740775711854,39.02343750000001 55.85681658243854,39.72656249999999 56.32872090717996,39.517822265625 56.32872090717996,39.29809570312501 56.243349924105246,38.91357421875 56.365250136856105))" 
                ,"POLYGON((40.92407226562501 56.32262930069561,40.836181640625 56.188367864753076,41.24267578125 56.19448087726974,41.275634765625 56.31044317134601,40.92407226562501 56.32262930069561))" 
                ,"POLYGON((36.35375976562501 55.652798033189555,36.309814453125 55.10980079314379,37.6611328125 55.36038057233307,37.6611328125 55.578344672182055,36.35375976562501 55.652798033189555))" 
                ,"POLYGON((41.49536132812499 56.79486226140054,43.01147460937501 55.96765007530669,44.176025390625 56.54737205307899,41.49536132812499 56.79486226140054))" 
                ,"POLYGON((35.419921875 55.63419794625841,35.343017578125 55.460170838618154,33.85986328124999 55.578344672182055,34.266357421875 55.986091533808406,35.419921875 55.63419794625841))" 
                ,"POLYGON((35.859375 56.07203547180089,35.66162109375 55.92458580482949,35.958251953125 55.83831352210822,36.49658203125 55.94919982336745,35.859375 56.07203547180089))" 
                ,"POLYGON((35.892333984375 56.517079019323745,35.79345703125 56.27996083172846,36.2109375 56.237244700410315,36.287841796875 56.492827145026666,35.892333984375 56.517079019323745))" 
                ,"POLYGON((35.2001953125 55.05320258537114,35.826416015625 54.977613670696286,35.716552734375 54.8386636129751,35.079345703125 54.85131525968609,35.2001953125 55.05320258537114))" 
                ,"POLYGON((38.95751953125001 55.05320258537114,39.17724609375 55.36038057233307,39.61669921875001 55.19768334019969,39.462890625 54.90188218738501,38.95751953125001 55.05320258537114))" 
                ,"POLYGON((40.83618164062499 55.64039895668736,40.616455078125 55.17886766328198,41.17675781249999 55.09723033442452,40.83618164062499 55.64039895668736))" 
                ,"POLYGON((39.517822265625 55.64659898563684,39.61669921874999 55.42901345240739,40.2099609375 55.410307210052196,40.53955078125 55.813629071199585,40.067138671875 55.94919982336745,39.75952148437499 55.677584411089526,39.517822265625 55.64659898563684))" 
                ,"POLYGON((38.265380859375 55.66519318443602,37.979736328125 55.522411831398216,38.48510742187499 55.4165436085801,39.056396484374986 55.547280698640805,38.265380859375 55.66519318443602))" 
                ,"POLYGON((38.12255859374999 54.79435160392052,37.716064453125 54.832336301970344,37.96875 55.09094362227856,38.46313476562499 55.01542594056298,38.12255859374999 54.79435160392052))" 
                ,"POLYGON((36.60644531249999 54.807017138462555,36.96899414062499 54.75633118164467,36.97998046875 54.457266680933856,36.397705078125 54.49556752187411,36.60644531249999 54.807017138462555))" 
                ,"POLYGON((42.76977539062499 55.03431871502809,41.80297851562499 54.876606654108684,42.07763671875 54.68653423452969,42.56103515625 54.699233528481386,42.76977539062499 55.03431871502809))" 
                ,"POLYGON((42.69287109374999 55.64039895668736,41.84692382812499 55.770393581620056,41.77001953125 55.441479359140686,43.121337890625 55.34788906283774,42.69287109374999 55.64039895668736))" 
                ,"POLYGON((40.24291992187499 56.72259433299854,40.18798828124999 56.53525774684846,40.3857421875 56.4806953901963,40.462646484375 56.72259433299854,40.24291992187499 56.72259433299854))" 
                ,"POLYGON((39.803466796875 54.88924640307587,40.67138671874999 54.85763959554899,40.58349609374999 54.80068486732233,39.83642578125 54.79435160392052,39.803466796875 54.88924640307587))" 
                ,"POLYGON((38.583984375 54.844989932187616,39.17724609375 54.82600799909497,39.17724609375 54.72462019492448,38.638916015625 54.74999097022689,38.583984375 54.844989932187616))" 
                ,"POLYGON((37.37548828125 54.69288437829769,37.76000976562499 54.64205540129177,37.705078125 54.48918653875083,37.265625 54.48918653875083,37.37548828125 54.69288437829769))" 
                ,"POLYGON((37.03491210937499 55.103516058019665,37.13378906249999 54.99652425983251,37.705078125 54.990221720048964,37.73803710937499 55.13492985052767,37.03491210937499 55.103516058019665))" 
                ,"POLYGON((41.660156249999986 56.145549500679095,40.83618164062499 55.986091533808406,40.91308593749999 55.795105452236925,41.69311523437499 56.004524201154,41.660156249999986 56.145549500679095))" 
                ,"POLYGON((38.869628906249986 56.752722872057376,38.67187499999999 56.53525774684846,39.37499999999999 56.517079019323745,39.517822265624986 56.81290751870026,38.869628906249986 56.752722872057376))" 
                ,"POLYGON((41.17675781249999 54.85131525968609,40.96801757812499 54.69288437829769,41.28662109374999 54.64205540129177,41.31958007812499 54.80068486732236,41.17675781249999 54.85131525968609))" 
                ,"POLYGON((35.58471679687499 55.36038057233307,35.93627929687499 55.65899609942838,36.123046875 55.59697126798508,35.848388671875 55.178867663282006,35.628662109375 55.22275708802212,35.58471679687499 55.36038057233307))" 
                ,"POLYGON((38.353271484375 56.01680776320322,37.96875 55.91227293006361,38.18847656249999 55.83831352210822,38.353271484375 56.01680776320322))" 
                ,"POLYGON((37.02392578125 55.95535088453653,38.089599609375 55.90611502601163,38.309326171875 55.832143877813024,37.144775390625 55.825973254619015,37.02392578125 55.95535088453653))" 
                ,"POLYGON((38.287353515625 55.97379820507658,39.19921875 55.72092280778696,38.968505859375 55.71473455012688,38.287353515625 55.97379820507658))" 
                ,"POLYGON((40.07809037249999 55.31661025761744,39.83639136257812 55.128616289259156,40.49557048054688 55.128616289259156,40.5175431178125 55.541032161833186,40.07809037249999 55.31661025761744))" 
            };
            SqlServerTypes.Utilities.LoadNativeAssemblies(AppDomain.CurrentDomain.BaseDirectory);
            List<Polygon> pols = new List<Polygon> { new Polygon(outPol) };
            foreach(var hole in holes.Select(s => new Polygon(s)))
            {
                List<Polygon> tmp = new List<Polygon>();
                foreach (Polygon p in pols)
                    tmp.AddRange(Polygon.IntersectAndSubtract(p, hole).Item2);
                pols = tmp;
            }

            Console.WriteLine("GEOMETRYCOLLECTION(");
            foreach (Polygon p in pols)
                Console.WriteLine(p.ToWtk() + ",");
            Console.WriteLine(")");
        }

        [TestMethod]
        public void TestPointInside()
        {
            bool ok = true;
            SqlServerTypes.Utilities.LoadNativeAssemblies(AppDomain.CurrentDomain.BaseDirectory);
            for (int i = 0; i < 10000; ++i)
            {
                Polygon p = GeneralTest.getRandomPolygon(new System.Random(), 4, 10, 30, 60);
                //Polygon p = new Polygon("POLYGON((15.117187500000007 26.431228064506428,2.9882812500000018 0.7031073524364899,27.246093750000007 -7.362466865535737,10.195312500000005 7.013667927566644,26.54296875000001 18.979025953255288,21.269531250000004 24.686952411999158,13.359375000000004 17.14079039331665,15.117187500000007 26.431228064506428))");
                Vector3D v = p.PointInside();
                //GeoPoint gp = GeoPoint.FromCartesian(v);
                //Console.WriteLine("GEOMETRYCOLLECTION(");
                //Console.WriteLine(p.ToWtk() + ",");
                //Console.WriteLine(String.Format("POINT({0} {1}))", gp.Longitude, gp.Latitude).Replace(',','.'));
                ok &= p.Contains(v);
            }
            Assert.IsTrue(ok);
        }

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
    */
}
