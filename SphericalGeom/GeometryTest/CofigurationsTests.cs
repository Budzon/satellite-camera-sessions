using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Collections.Generic;
using System.Linq;

using SatelliteSessions;
using OptimalChain;
using Astronomy;
using SphericalGeom;

namespace GeometryTest
{
    [TestClass]
    public class CofigurationsTests
    {
        [TestMethod]
        public void Test_getConfsToDrop()
        {


            /*

            // Troute = new DateTime(2019, 1, 4, 0, 0, 20) - new DateTime(2019, 1, 4, 0, 0, 0);// parameters.end - parameters.start;
            // zip_mk = 1;
            // zip_pk = 1;
            
            List<Tuple<DateTime, DateTime>> freeRanges = new List<Tuple<DateTime, DateTime>>();

            DateTime span1_1 = new DateTime(2000, 01, 01);
            DateTime span1_2 = new DateTime(2000, 01, 05);

            DateTime span2_1 = new DateTime(2000, 01, 08);
            DateTime span2_2 = new DateTime(2000, 01, 09);

            freeRanges.Add(new Tuple<DateTime, DateTime>(span1_1, span1_2));
            freeRanges.Add(new Tuple<DateTime, DateTime>(span2_1, span2_2));

            List<RouteMPZ> routesToDrop = new List<RouteMPZ>();
            for (int i = 0; i < 5; i++)
            {
                Order order = new Order();
                order.captured = new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))");
                order.intersection_coeff = 0.1;
                order.request = new RequestParams();

                order.request.timeFrom = new DateTime(2019, 1, 4);
                order.request.timeTo = new DateTime(2019, 1, 5);
                order.request.wktPolygon = "POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))";
                order.request.minCoverPerc = 0.4;
                order.request.Max_SOEN_anlge = AstronomyMath.ToRad(45);
                List<Order> orders = new List<Order>() { order };

                CaptureConf cc = new CaptureConf(new DateTime(2019, 1, 4), new DateTime(2019, 1, 5), 0.1, orders, 0, null);
                StaticConf sc = cc.DefaultStaticConf();
                RouteParams routeParam = new RouteParams(sc);
                routeParam.id = i; 
                routeParam.start = new DateTime(2019, 1, 4);
                routeParam.end = new DateTime(2019, 1, 5);
               // double timedrop = routeParam.getDropTime();

                routesToDrop.Add(new RouteMPZ(routeParam) { NPZ = i, Nroute = i });
            }

            double timedrop = routesToDrop[0].Parameters.getDropTime();
            

            List<CaptureConf> confToDrop = Sessions.getConfsToDrop(routesToDrop, freeRanges);

            Assert.IsTrue(confToDrop.Count == 5);

            confToDrop.Sort(delegate(CaptureConf conf1, CaptureConf conf2) { return conf1.dateFrom.CompareTo(conf2.dateFrom); });

            for (int i = 0; i < 4; i++)
            {
                Assert.IsTrue(span1_1 <= confToDrop[i].dateFrom && confToDrop[i].dateFrom <= span1_2);
                Assert.IsTrue(span1_1 <= confToDrop[i].dateTo && confToDrop[i].dateTo <= span1_2);
            }

            Assert.IsTrue(span2_1 <= confToDrop[4].dateFrom && confToDrop[4].dateFrom <= span2_2);
            Assert.IsTrue(span2_1 <= confToDrop[4].dateTo && confToDrop[4].dateTo <= span2_2);

            //Assert.IsTrue(confToDrop[ == 5);

            */
        }



        [TestMethod]
        public void Test_getConfsToDelete()
        {



        }
    }
}
