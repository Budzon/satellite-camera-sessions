using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

using System.Collections.Generic;
using System.Linq;
using Astronomy;
using System.Windows.Media.Media3D;
using SphericalGeom;
using Common;
using OptimalChain;
using SatelliteSessions;
using DBTables;

namespace GeometryTest
{
    
    [TestClass]
    public class DataBaseTest
    { 
        [TestMethod]
        public void Test_GetMeteoData()
        {
            /*
            // тест рассчитан на то, что в таблице есть только одна запись с  облачносью с 06/06/2019 00:00 по 07/06/2019 00:00

            string cs = System.IO.File.ReadLines("DBstringCUKS.conf").First();
            DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);
            DBTables.DataFetcher fetcher = new DataFetcher(manager);


            { // временной диапазон облачности включает в себя диапазон заданный
                DateTime dt1 = DateTime.Parse("06/06/2019 12:00:0");
                DateTime dt2 = DateTime.Parse("06/06/2019 13:00:0");
                var res = fetcher.GetMeteoData(dt1, dt2);
                Assert.IsTrue(res.Count == 1);
            }
            { // временной диапазон облачности включен в диапазон заданный
                DateTime dt1 = DateTime.Parse("02/06/2019 00:00:0");
                DateTime dt2 = DateTime.Parse("09/06/2019 00:00:0");
                var res = fetcher.GetMeteoData(dt1, dt2);
                Assert.IsTrue(res.Count == 1);
            }
            { // временной диапазон облачности полностью до диапазона заданного
                DateTime dt1 = DateTime.Parse("08/06/2019 12:00:0");
                DateTime dt2 = DateTime.Parse("09/06/2019 13:00:0");
                var res = fetcher.GetMeteoData(dt1, dt2);
                Assert.IsTrue(res.Count == 0);
            }
            { // временной диапазон облачности полностью после диапазона заданного
                DateTime dt1 = DateTime.Parse("02/06/2019 12:00:0");
                DateTime dt2 = DateTime.Parse("03/06/2019 13:00:0");
                var res = fetcher.GetMeteoData(dt1, dt2);
                Assert.IsTrue(res.Count == 0);
            }
            { // временной диапазон облачности включает в себя время начала диапазона заданного
                DateTime dt1 = DateTime.Parse("06/06/2019 23:00:0");
                DateTime dt2 = DateTime.Parse("08/06/2019 00:00:0");
                var res = fetcher.GetMeteoData(dt1, dt2);
                Assert.IsTrue(res.Count == 1);
            }
            {// временной диапазон облачности включает в себя время конца диапазона заданного
                DateTime dt1 = DateTime.Parse("02/06/2019 00:00:0");
                DateTime dt2 = DateTime.Parse("06/06/2019 13:00:0");
                var res = fetcher.GetMeteoData(dt1, dt2);
                Assert.IsTrue(res.Count == 1);
            }


            {// на равенство
                DateTime dt1 = DateTime.Parse("02/06/2019 00:00:0");
                DateTime dt2 = DateTime.Parse("06/06/2019 12:00:0");
                var res = fetcher.GetMeteoData(dt1, dt2);
                Assert.IsTrue(res.Count == 0);
            }

            */
        }
    }
}
