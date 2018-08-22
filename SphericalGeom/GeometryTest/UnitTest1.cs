using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using SatelliteTrajectory;
using Common;
using Astronomy;
using OptimalChain;
using SphericalGeom;

namespace GeometryTest
{
    [TestClass]
    public class UnitTest
    {
        [TestMethod]
        public void Test_SphericalLine()
        {
            {
                GeoPoint A = new GeoPoint(21.28937435586043, 30.629882812499993);
                GeoPoint B = new GeoPoint(8.233237111274562, 30.98144531249999);
                GeoPoint C = new GeoPoint(16.172472808397515, 44.5166015625);

                SphericalVector AB = new SphericalVector(A, B);
                SphericalVector BA = new SphericalVector(B, A);

                Assert.IsTrue(AB.getPointSide(C) == SphericalVector.PointSide.Left);
                Assert.IsTrue(BA.getPointSide(C) == SphericalVector.PointSide.Right);
            }
            ///////

            {
                GeoPoint A = new GeoPoint(74.8451787339525, -63.1348395505636);
                GeoPoint B = new GeoPoint(75.3782839712454, -63.6369587442901);
                GeoPoint C = new GeoPoint(74.91758915671994, -62.994824847441976);
                
                SphericalVector AB = new SphericalVector(A, B);

                Assert.IsTrue(AB.getPointSide(C) == SphericalVector.PointSide.Right);
            }


           // Polygon from = new Polygon("POLYGON ((-62.994824847441983 74.917589156719927, -63.428327112973811 74.8730232406742, -63.272250887633348 74.773274613632637, -62.839546056121904 74.8167170706072, -62.994824847441983 74.917589156719927))");
           // Polygon to = new Polygon("POLYGON ((-63.494110218017632 75.453723146466842, -63.941479843607347 75.403754318281131, -63.776810543456335 75.303509334225865, -63.330855822873794 75.35224096486526, -63.494110218017632 75.453723146466842))");






        }
    }
}
