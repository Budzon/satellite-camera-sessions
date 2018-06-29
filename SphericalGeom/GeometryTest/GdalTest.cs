using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

using SphericalGeom;
using Common;
using SessionsPlanning;

namespace GeometryTest
{
    [TestClass]
    public class GdalTest
    {
        [TestMethod]
        public void TestDMS()
        {
            double Pi = 13.1415;
            DMS pi = new DMS(Pi);
            Assert.IsTrue(pi.Degrees == 13 && pi.Minutes == 8 && Comparison.IsEqual(pi.DecimalDegrees, Pi));
        }

        [TestMethod]
        public void TestDmeRead()
        {
            GeoPoint gp = new GeoPoint(45.21913847328, 5.73397802);
            DemHandler dem = new DemHandler("D:\\repos_budz\\satellite-camera-sessions\\SphericalGeom\\GeometryTest\\");
            Console.WriteLine(dem.GetHeight(gp));
        }
    }
}
