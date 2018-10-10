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
using SessionsPlanning;
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



        [TestMethod]
        public void Test_checkPNBRouteCompatible() ///(RouteParams routeParams, List<MPZ> pnb, out List<Tuple<int, int>> conflicts)
        {
            //DateTime dt1 = new DateTime(2019, 1, 4, 0, 0, 1);
            //DateTime dt2 = new DateTime(2019, 1, 4, 0, 1, 1);
            ////WorkingType t, DateTime d1, DateTime d2, ShootingType st = ShootingType.Normal, ShootingChannel channel = ShootingChannel.pk, int fs = 1000
            //Order order = new Order();
            //order.captured = new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))");
            //order.intersection_coeff = 0.1;
            //CaptureConf cc = new CaptureConf(dt1, dt2, 0, new List<Order>() { order }, SessionsPlanning.WorkingType.Shooting, null);
            //RouteParams routesParams = new RouteParams(SessionsPlanning.WorkingType.Shooting, dt1, dt2);
            //routesParams.ShootingConf = cc.DefaultStaticConf();
 
            DateTime dt1 = new DateTime(2019, 2, 18, 23, 0, 0);
            DateTime dt2 = new DateTime(2019, 2, 19, 0, 0, 0);

            string cupConnStr = System.IO.File.ReadLines("DBstring.conf").First();
            string cuksConnStr = System.IO.File.ReadLines("DBstringCUKS.conf").First();
            DIOS.Common.SqlManager CUKSmanagerDB = new DIOS.Common.SqlManager(cuksConnStr);
            DIOS.Common.SqlManager CUPmanagerDB = new DIOS.Common.SqlManager(cupConnStr);

            string polwtk = "POLYGON((140.47668457031253 -17.623081791311762,139.603271484375 -17.30606566309359,139.43023681640625 -18.145851771694467,140.5865478515625 -18.19282519773317,140.47668457031253 -17.623081791311762))";

            List<string> holes = new List<string>();

            RequestParams req = new RequestParams(0, 1, dt1, dt2,
                _Max_SOEN_anlge: AstronomyMath.ToRad(50),
                _minCoverPerc: 0.12,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: polwtk,
                _polygonsToSubtract: holes, _requestChannel: ShootingChannel.pk,
                _shootingType: ShootingType.Normal);

            List<Tuple<DateTime, DateTime>> silenceRanges = new List<Tuple<DateTime, DateTime>>();
            List<Tuple<DateTime, DateTime>> inactivityRanges = new List<Tuple<DateTime, DateTime>>();

            List<RouteMPZ> routesToDrop = new List<RouteMPZ>();
            List<RouteMPZ> routesToDelete = new List<RouteMPZ>();

            List<MPZ> mpzArray;
            List<CommunicationSession> sessions;
            var enabled = new List<SessionsPlanning.CommunicationSessionStation>  
            { SessionsPlanning.CommunicationSessionStation.FIGS,
                SessionsPlanning.CommunicationSessionStation.FIGS_Backup,
                SessionsPlanning.CommunicationSessionStation.MIGS };
            Sessions.getMPZArray(new List<RequestParams> { req }, dt1, dt2
            , silenceRanges
            , inactivityRanges
            , routesToDrop
            , routesToDelete
            , cupConnStr
            , cuksConnStr
            , 356
            , out mpzArray
            , out sessions
            , enabled);


            RouteParams routesParams = new RouteParams(mpzArray.First().Routes.First().Parameters);

           // mpzArray.First().Routes.First().Parameters
            List<Tuple<int, int>> conflicts;
            Sessions.checkPNBRouteCompatible(routesParams, mpzArray.Select(mpz => mpz.Parameters).ToList(), out conflicts);
            Assert.AreEqual(conflicts.Count(), 0);
                


        }

    }
}
