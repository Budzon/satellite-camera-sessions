using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Windows.Media.Media3D;
using SphericalGeom;
using Common;

namespace GeometryTest
{
    [TestClass]
    public class ArcTest
    {
        private TestContext testContextInstance;
        public TestContext TestContext
        {
            get { return testContextInstance; }
            set { testContextInstance = value; }
        }

        [TestMethod]
        public void TestConstructorApexInOrOut()
        {
            Arc arc1 = new Arc(
                GeoPoint.ToCartesian(new GeoPoint(30, -10), 1),
                GeoPoint.ToCartesian(new GeoPoint(30, 10), 1),
                new Vector3D(0, 0, 0.5));
            Arc arc2 = new Arc(
                GeoPoint.ToCartesian(new GeoPoint(30, -10), 1),
                GeoPoint.ToCartesian(new GeoPoint(30, 10), 1),
                new Vector3D(100, -200, 0.5));

            bool sameAxis = Comparison.IsZero(arc1.Axis - arc2.Axis);
            bool sameCenter = Comparison.IsZero(arc1.Center - arc2.Center);
            bool sameTangentA = Comparison.IsZero(arc1.TangentA - arc2.TangentA);
            bool sameTangentB = Comparison.IsZero(arc1.TangentB - arc2.TangentB);
            bool sameCentralAngle = Comparison.IsEqual(arc1.CentralAngle, arc2.CentralAngle);
            bool sameCentralAngleWithOrientation = Comparison.IsEqual(arc1.CentralAngleWithOrientation, arc2.CentralAngleWithOrientation);
            bool sameRadius = Comparison.IsEqual(arc1.Radius, arc2.Radius);
            bool sameCounterclockwise = (arc1.Counterclockwise == arc2.Counterclockwise);

            Assert.IsTrue(sameAxis && sameCenter
                       && sameTangentA && sameTangentB
                       && sameCentralAngle && sameCentralAngleWithOrientation
                       && sameRadius
                       && sameCounterclockwise);
        }

        [TestMethod]
        public void TestProperties()
        {
            Arc arc = new Arc(
                GeoPoint.ToCartesian(new GeoPoint(60, 0), 1),
                GeoPoint.ToCartesian(new GeoPoint(0, 60), 1),
                GeoPoint.ToCartesian(new GeoPoint(0, -60), 1));

            Vector3D center = new Vector3D(0.5, 0, 0);
            bool sameCenter = Comparison.IsZero(arc.Center - center);

            Vector3D tangentA = new Vector3D(0, 1, 0);
            bool sameTangentA = Comparison.IsZero(arc.TangentA - tangentA);

            Vector3D tangentB = new Vector3D(0, 0, -1);
            bool sameTangentB = Comparison.IsZero(arc.TangentB - tangentB);

            bool sameCentralAngle = Comparison.IsEqual(arc.CentralAngle, Math.PI / 2);
            bool sameCentralAngleWithOrientation = Comparison.IsEqual(arc.CentralAngleWithOrientation, 3 * Math.PI / 2);
            bool sameRadius = Comparison.IsEqual(arc.Radius, Math.Sqrt(3) / 2);
            bool sameCounterclockwise = (arc.Counterclockwise == false);

            Assert.IsTrue(sameCenter
                       && sameTangentA && sameTangentB
                       && sameCentralAngle && sameCentralAngleWithOrientation
                       && sameRadius
                       && sameCounterclockwise);
        }

        [TestMethod]
        public void TestExtremeLatLon()
        {
            Arc arc1 = new Arc(
                GeoPoint.ToCartesian(new GeoPoint(30, -60), 1),
                GeoPoint.ToCartesian(new GeoPoint(30, 60), 1));
            bool sameMaxLat = Math.Abs(arc1.MaxLatitudeDeg() - 
                                       180 / Math.PI * Math.Asin(2 / Math.Sqrt(7))) < 1e-3;

            double y = Math.Sqrt(3) / 2, ang = Math.PI * 2 / 3;
            Arc arc2 = new Arc(
                new Vector3D(0.5, Math.Cos(ang) * y, -Math.Sin(ang) * y),
                new Vector3D(0.5, Math.Cos(ang) * y, Math.Sin(ang) * y),
                new Vector3D(0.5, 0, 0));
            bool sameMinLon = Math.Abs(arc2.MinLongitudeDeg() + 60) < 1e-3;

            Assert.IsTrue(sameMaxLat && sameMinLon);
        }

        [TestMethod]
        public void TestContains_WrongPlane()
        {
            Arc arc = new Arc(GeoPoint.ToCartesian(new GeoPoint(0, -5), 1),
                GeoPoint.ToCartesian(new GeoPoint(0, 5), 1));
            Vector3D v = GeoPoint.ToCartesian(new GeoPoint(1, 0), 1);

            Assert.IsFalse(arc.Contains(v));
        }

        [TestMethod]
        public void TestContains_SamePlaneYes()
        {
            Arc arc = new Arc(GeoPoint.ToCartesian(new GeoPoint(0, -5), 1),
                GeoPoint.ToCartesian(new GeoPoint(0, 5), 1));
            Vector3D v = GeoPoint.ToCartesian(new GeoPoint(0, 0), 1);

            Assert.IsTrue(arc.Contains(v));
        }

        [TestMethod]
        public void TestContains_Endpoint()
        {
            Arc arc = new Arc(GeoPoint.ToCartesian(new GeoPoint(0, -5), 1),
                GeoPoint.ToCartesian(new GeoPoint(0, 5), 1));
            Vector3D v = GeoPoint.ToCartesian(new GeoPoint(0, 5), 1);

            Assert.IsTrue(arc.Contains(v));
        }

        [TestMethod]
        public void TestContains_SamePlaneNo()
        {
            Arc arc = new Arc(GeoPoint.ToCartesian(new GeoPoint(0, -5), 1),
                GeoPoint.ToCartesian(new GeoPoint(0, 5), 1));
            Vector3D v = GeoPoint.ToCartesian(new GeoPoint(0, 5.1), 1);

            Assert.IsFalse(arc.Contains(v));
        }

        [TestMethod]
        public void TestIntersect_TwoPoints()
        {
            double t = Math.Sqrt(3) / 2, ang = Math.PI / 3;
            Arc arc1 = new Arc(
                new Vector3D(0.5, Math.Cos(ang) * t, -Math.Sin(ang) * t),
                new Vector3D(0.5, Math.Cos(ang) * t, Math.Sin(ang) * t),
                new Vector3D(0.5, 0, 0));
            Arc arc2 = new Arc(
                new Vector3D(Math.Cos(ang) * t, 0.5, -Math.Sin(ang) * t),
                new Vector3D(Math.Cos(ang) * t, 0.5, Math.Sin(ang) * t),
                new Vector3D(0, 0.5, 0));

            Vector3D v1 = new Vector3D(0.5, 0.5, 1 / Math.Sqrt(2));
            Vector3D v2 = new Vector3D(0.5, 0.5, -1 / Math.Sqrt(2));
            bool hasV1 = false;
            bool hasV2 = false;

            IEnumerable<Vector3D> intersection = Arc.Intersect(arc1, arc2);
            foreach (Vector3D v in intersection)
            {
                hasV1 |= Comparison.IsZero(v - v1);
                hasV2 |= Comparison.IsZero(v - v2);
            }

            Assert.IsTrue(hasV1 && hasV2);
        }

        [TestMethod]
        public void TestIntersect_OnePoint()
        {
            Arc arc1 = new Arc(GeoPoint.ToCartesian(new GeoPoint(0, -10), 1),
                GeoPoint.ToCartesian(new GeoPoint(0, 10), 1));
            Arc arc2 = new Arc(GeoPoint.ToCartesian(new GeoPoint(-10, 0), 1),
                GeoPoint.ToCartesian(new GeoPoint(10, 0), 1));

            Vector3D v1 = new Vector3D(1, 0, 0);
            bool hasV1 = false;

            var intersection = Arc.Intersect(arc1, arc2);
            int count = 0;
            foreach (Vector3D v in intersection)
            {
                count += 1;
                hasV1 |= Comparison.IsZero(v - v1);
            }

            Assert.IsTrue((count == 1) && hasV1);
        }

        [TestMethod]
        public void TestIntersect_ZeroPoints()
        {
            Arc arc1 = new Arc(GeoPoint.ToCartesian(new GeoPoint(-5, 0), 1),
                GeoPoint.ToCartesian(new GeoPoint(5, 0), 1));
            Arc arc2 = new Arc(GeoPoint.ToCartesian(new GeoPoint(0, 1), 1),
                GeoPoint.ToCartesian(new GeoPoint(0, 5), 1));
            
            var intersection = Arc.Intersect(arc1, arc2);
            int count = 0;
            foreach (Vector3D v in intersection)
                count += 1;
            
            Assert.IsTrue(count == 0);
        }

        [TestMethod]
        public void TestIntersect_Parallel()
        {
            double t = Math.Sqrt(3) / 2, ang = Math.PI / 3;
            Arc arc1 = new Arc(
                new Vector3D(Math.Cos(ang) * t, 0.5, -Math.Sin(ang) * t),
                new Vector3D(Math.Cos(ang) * t, 0.5, Math.Sin(ang) * t),
                new Vector3D(0, 0.5, 0));
            Arc arc2 = new Arc(GeoPoint.ToCartesian(new GeoPoint(-5, 0), 1),
                GeoPoint.ToCartesian(new GeoPoint(5, 0), 1));

            var intersection = Arc.Intersect(arc1, arc2);
            int count = 0;
            foreach (Vector3D v in intersection)
                count += 1;

            Assert.IsTrue(count == 0);
        }

        [TestMethod]
        public void TestIntersect_SameCircleNoOverlap()
        {
            Arc arc1 = new Arc(GeoPoint.ToCartesian(new GeoPoint(0, -10), 1),
                GeoPoint.ToCartesian(new GeoPoint(0, 0), 1));
            Arc arc2 = new Arc(GeoPoint.ToCartesian(new GeoPoint(0, 5), 1),
                GeoPoint.ToCartesian(new GeoPoint(0, 10), 1));

            var intersection = Arc.Intersect(arc1, arc2);
            int count = 0;
            foreach (Vector3D v in intersection)
                count += 1;

            Assert.IsTrue(count == 0);
        }
        
        [TestMethod]
        public void TestIntersect_SameCircleOverlap()
        {
            Arc arc1 = new Arc(GeoPoint.ToCartesian(new GeoPoint(0, -10), 1),
                GeoPoint.ToCartesian(new GeoPoint(0, 0), 1));
            Arc arc2 = new Arc(GeoPoint.ToCartesian(new GeoPoint(0, -5), 1),
                GeoPoint.ToCartesian(new GeoPoint(0, 5), 1));

            bool overlap = false;
            try
            {
                Arc.Intersect(arc1, arc2);
            }
            catch
            {
                overlap = true;
            }

            Assert.IsTrue(overlap);
        }

        [TestMethod]
        public void TestIntersect_VertexInner()
        {
            Arc arc1 = new Arc(GeoPoint.ToCartesian(new GeoPoint(0, 0), 1),
                GeoPoint.ToCartesian(new GeoPoint(0, 10), 1));
            Arc arc2 = new Arc(GeoPoint.ToCartesian(new GeoPoint(0, 5), 1),
                GeoPoint.ToCartesian(new GeoPoint(10, 5), 1));

            Vector3D v1 = GeoPoint.ToCartesian(new GeoPoint(0, 5), 1);
            bool hasV1 = false;

            var intersection = Arc.Intersect(arc1, arc2);
            int count = 0;
            foreach (Vector3D v in intersection)
            {
                count += 1;
                hasV1 |= Comparison.IsZero(v - v1);
            }

            Assert.IsTrue((count == 1) && hasV1);
        }

        [TestMethod]
        public void TestIntersect_VertexVertex()
        {
            Arc arc1 = new Arc(GeoPoint.ToCartesian(new GeoPoint(0, 0), 1),
                GeoPoint.ToCartesian(new GeoPoint(0, 5), 1));
            Arc arc2 = new Arc(GeoPoint.ToCartesian(new GeoPoint(0, 5), 1),
                GeoPoint.ToCartesian(new GeoPoint(10, 5), 1));

            Vector3D v1 = GeoPoint.ToCartesian(new GeoPoint(0, 5), 1);
            bool hasV1 = false;

            var intersection = Arc.Intersect(arc1, arc2);
            int count = 0;
            foreach (Vector3D v in intersection)
            {
                count += 1;
                hasV1 |= Comparison.IsZero(v - v1);
            }

            Assert.IsTrue((count == 1) && hasV1);
        }

        [TestMethod]
        public void TestUpdateWithIntersections()
        {
            double t = Math.Sqrt(3) / 2, ang = Math.PI / 3;
            Arc arc1 = new Arc(
                new Vector3D(0.5, Math.Cos(ang) * t, -Math.Sin(ang) * t),
                new Vector3D(0.5, Math.Cos(ang) * t, Math.Sin(ang) * t),
                new Vector3D(0.5, 0, 0));
            Arc arc2 = new Arc(
                new Vector3D(Math.Cos(ang) * t, 0.5, -Math.Sin(ang) * t),
                new Vector3D(Math.Cos(ang) * t, 0.5, Math.Sin(ang) * t),
                new Vector3D(0, 0.5, 0));

            Vector3D v1 = new Vector3D(0.5, 0.5, 1 / Math.Sqrt(2));
            Vector3D v2 = new Vector3D(0.5, 0.5, -1/ Math.Sqrt(2));
            bool hasV1_1 = false, hasV1_2 = false;
            bool hasV2_1 = false, hasV2_2 = false;
            int count1 = 0, count2 = 0;

            Arc.UpdateWithIntersections(arc1, arc2);
            foreach (Vector3D v in arc1.IntermediatePoints)
            {
                hasV1_1 |= Comparison.IsZero(v - v1);
                hasV2_1 |= Comparison.IsZero(v - v2);
                count1 += 1;
            }
            foreach (Vector3D v in arc1.IntermediatePoints)
            {
                hasV1_2 |= Comparison.IsZero(v - v1);
                hasV2_2 |= Comparison.IsZero(v - v2);
                count2 += 1;
            }

            Assert.IsTrue(hasV1_1 && hasV1_2 && hasV2_1 && hasV2_2
                && (count1 == 2) && (count2 == 2));           
        }

    }
}
