﻿using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

using SphericalGeom;
using Common;
using SessionsPlanning;

namespace GeometryTest
{
    [TestClass]
    public class GdalTest
    {
        //[TestMethod]
        //public void TestDMS()
        //{
        //    double Pi = 13.1415;
        //    DMS pi = new DMS(Pi);
        //    Assert.IsTrue(pi.Degrees == 13 && pi.Minutes == 8 && Comparison.IsEqual(pi.DecimalDegrees, Pi));
        //}

        [TestMethod]
        public void TestDmeRead()
        {
            SqlServerTypes.Utilities.LoadNativeAssemblies(AppDomain.CurrentDomain.BaseDirectory);
            Polygon p = new Polygon("POLYGON((-123.08532714843749 41.401535582898475,-123.39843749999999 41.37268648186466,-123.30505371093751 41.244772343082076,-123.4149169921875 41.17451935556443,-123.343505859375 41.050359519318874,-123.23913574218749 40.93841495689793,-123.06884765625001 40.93011520598304,-122.8765869140625 40.98404494692812,-122.84362792968749 41.13315883477395,-122.728271484375 41.20345619205128,-122.7667236328125 41.27367811566259,-123.05786132812499 41.26542062892668,-123.08532714843749 41.401535582898475))");
            DemHandler dem = new DemHandler("D:\\repos_budz\\satellite-camera-sessions\\SphericalGeom\\GeometryTest\\hgt_data\\", false, false);
            Console.WriteLine(dem.GetAverageHeight(p));
        }

        [TestMethod]
        public void TestDmeZip()
        {
            DemHandler dem_zip = new DemHandler("D:\\repos_budz\\satellite-camera-sessions\\SphericalGeom\\GeometryTest\\hgt_data\\", true, false);
            DemHandler dem_zip_ftp = new DemHandler("https://dds.cr.usgs.gov/srtm/version2_1/SRTM3/North_America/", true, true);
            //var data = dem_zip.Open("/vsicurl/http://kartat.kapsi.fi/files/orto/etrs-tm35fin/mavi_vv_25000_50/2014/M53/02m/1/M5322A.jp2");
            var res_zip = dem_zip.GetHeight(new GeoPoint(40.5, -123.5));
            //var res_zip_ftp = dem_zip_ftp.GetHeight(new GeoPoint(40.5, -123.5));
            //Assert.AreEqual(res_zip_ftp, res_zip);
        }
    }
}
