using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

using DataParsers;
using SatelliteTrajectory;
using Astronomy;
using Common;
using OptimalChain;
using Microsoft.SqlServer.Types;
using System.Data.SqlTypes;
using DBTables;
using SatelliteSessions;


using SphericalGeom;
namespace GeometryTest
{
    [TestClass]
    public class RoutesAndMPZTest
    {
        [TestMethod]
        public void Test_createPNbOfRoutes()
        {
            ///// Просто проверяем, что роуты правильно распределяются по группам по 12 
            //List<RouteParams> routesParams = new List<RouteParams>();
            //for (int i = 1; i <= 12 + 12 + 3; i++)
            //{
            //    Order order = new Order();
            //    order.captured = new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))");
            //    order.intersection_coeff = 0.1;
            //    order.request = new RequestParams();

            //    order.request.timeFrom = new DateTime(2019, 1, 4);
            //    order.request.timeTo = new DateTime(2019, 1, 5);
            //    order.request.wktPolygon = "POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))";
            //    order.request.minCoverPerc = 0.4;
            //    order.request.Max_SOEN_anlge = AstronomyMath.ToRad(45);
            //    List<Order> orders = new List<Order>() { order };

            //    CaptureConf cc = new CaptureConf(new DateTime(2019, 1, 4), new DateTime(2019, 1, 5), 0.1, orders, 0, null);
            //    StaticConf sc = cc.DefaultStaticConf();
            //    RouteParams routeParam = new RouteParams(sc);
            //    routeParam.id = i;
            //    routeParam.binded_route = new Tuple<int, int>(1, 1);
      
            //    routesParams.Add(routeParam);
            //}

            //List<MPZ> res = Sessions.createPNbOfRoutes(routesParams, 0);

            //Assert.IsTrue(res.Count == 3);

            //Assert.IsTrue(res[0].Routes.Count == 12);
            //Assert.IsTrue(res[1].Routes.Count == 12);
            //Assert.IsTrue(res[2].Routes.Count == 3);             
        }
    }
}
