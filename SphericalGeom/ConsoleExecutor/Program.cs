using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.Serialization.Formatters.Binary;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
//using Microsoft.SqlServer.Types;
using System.Data.SqlTypes;

using Common;
using SphericalGeom;
using SatelliteRequests;
using SatelliteTrajectory;
using SatelliteSessions;
using DataParsers;
using Astronomy;
using OptimalChain;
using DBTables;
using Constants = OptimalChain.Constants;

using SessionsPlanning;

namespace ConsoleExecutor
{
    class Program
    {
        static public void test_getSunBlindingPeriods()
        {
            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
            DateTime fromDt = DateTime.Parse("10.02.2019 0:0:0");
            //   Sessions.getSunBlindingPeriods(fromDt, fromDt.AddDays(1), managerDB);
        }

        static public IList<CaptureConf> test_getCaptureConfArray()
        {
            DateTime start = DateTime.Now;
            test_getPlainMpzArray();
            DateTime endd = DateTime.Now;
            Console.WriteLine();
            Console.WriteLine("total time = " + (endd - start).TotalSeconds.ToString());
            return null;
        }

        static public void test_checkIfViewLaneIsLit()
        {
            DateTime dt1 = DateTime.Parse("2019-01-01T00:00:00");// new DateTime(2019, 2, 18, 2, 0, 0);
            DateTime dt2 = DateTime.Parse("2019-02-01T00:00:00");// new DateTime(2019, 2, 18, 3, 0, 0);
            List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot;
            Sessions.checkIfViewLaneIsLit(System.IO.File.ReadLines("DBstring.conf").First(), dt1, dt2, out partsLitAndNot);
            Console.WriteLine(partsLitAndNot.Count);
            Console.WriteLine();
            foreach (var tuple in partsLitAndNot)
            {
                Console.WriteLine(tuple.Item1);
                Console.WriteLine();
                Console.WriteLine(Polygon.getMultipolFromPolygons(  tuple.Item2.Select(wpl => new Polygon(wpl.wktPolygon)).ToList() ));
            }

            return;
        }


        static public void test_error()
        {
            #region
            RequestParams req0 = new RequestParams(333, 1,
                DateTime.Parse("2010-02-04T00:00:00"),
                DateTime.Parse("2029-02-04T00:00:00"),
                _Max_SOEN_anlge: 12,
                _minCoverPerc: 333,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: "POLYGON((-61.14715576171875 10.728079873904903,-61.28173828125 10.455401826918404,-61.1553955078125 10.401377554543544,-61.09222412109374 10.70649049256302,-61.14715576171875 10.728079873904903))",
                _polygonToSubtract: new List<string>(),
                _requestChannel: 0,
                _shootingType: ShootingType.Normal,
                _compression: 1,
                _albedo: 0
                ); 



            RequestParams req1 = new RequestParams(334, 1,
                DateTime.Parse("2010-02-04T00:00:00"),
                DateTime.Parse("2029-02-04T00:00:00"),
                _Max_SOEN_anlge: 12,
                _minCoverPerc: 334,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: "POLYGON((-73.08043139995968 -42.52665075099472, -73.04613448725493 -42.53913380634647, -72.95396191058708 -42.28589173325857, -72.98825882329183 -42.27340867790682, -73.08043139995968 -42.52665075099472))",
                _polygonToSubtract: new List<string>(),
                _requestChannel: 0,
                _shootingType: ShootingType.Normal,
                _compression: 1,
                _albedo: 0
                );





            RequestParams req2 = new RequestParams(335, 1,
                DateTime.Parse("2010-02-04T00:00:00"),
                DateTime.Parse("2029-02-04T00:00:00"),
                _Max_SOEN_anlge: 12,
                _minCoverPerc: 335,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: "POLYGON((-72.26477469706055 -40.165038563950425, -72.23169475460733 -40.17707867835463, -72.13952217793948 -39.923836605266736, -72.1726021203927 -39.91179649086252, -72.26477469706055 -40.165038563950425))",
                _polygonToSubtract: new List<string>(),
                _requestChannel: 0,
                _shootingType: ShootingType.Normal,
                _compression: 1,
                _albedo: 0
                );





            RequestParams req3 = new RequestParams(336, 1,
                DateTime.Parse("2010-02-04T00:00:00"),
                DateTime.Parse("2029-02-04T00:00:00"),
                _Max_SOEN_anlge: 12,
                _minCoverPerc: 336,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: "POLYGON((-74.50739160013873 -46.2569644968801, -74.4708453814471 -46.270266232658834, -74.37867280477926 -46.01702415957096, -74.41521902347088 -46.00372242379224, -74.50739160013873 -46.2569644968801))",
                _polygonToSubtract: new List<string>(),
                _requestChannel: 0,
                _shootingType: ShootingType.Normal,
                _compression: 5,
                _albedo: 0
                );





            RequestParams req4 = new RequestParams(338, 1,
                DateTime.Parse("2010-02-04T00:00:00"),
                DateTime.Parse("2029-02-04T00:00:00"),
                _Max_SOEN_anlge: 12,
                _minCoverPerc: 338,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: "POLYGON((-74.85488593740163 -47.77977852386863, -74.81728663926623 -47.79346354921923, -74.72511406259838 -47.54022147613136, -74.76271336073378 -47.52653645078076, -74.85488593740163 -47.77977852386863))",
                _polygonToSubtract: new List<string>(),
                _requestChannel: 0,
                _shootingType: ShootingType.Normal,
                _compression: 1,
                _albedo: 0
                );





            RequestParams req5 = new RequestParams(339, 1,
                DateTime.Parse("2010-02-04T00:00:00"),
                DateTime.Parse("2029-02-04T00:00:00"),
                _Max_SOEN_anlge: 12,
                _minCoverPerc: 339,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: "POLYGON((-73.74378139686242 -44.43018054374746, -73.70839117980545 -44.443061529340426, -73.6162186031376 -44.18981945625255, -73.65160882019457 -44.176938470659586, -73.74378139686242 -44.43018054374746))",
                _polygonToSubtract: new List<string>(),
                _requestChannel: 0,
                _shootingType: ShootingType.Normal,
                _compression: 1,
                _albedo: 0
                );





            RequestParams req6 = new RequestParams(340, 1,
                DateTime.Parse("2010-02-04T00:00:00"),
                DateTime.Parse("2029-02-04T00:00:00"),
                _Max_SOEN_anlge: 12,
                _minCoverPerc: 340,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: "POLYGON((-71.37657118842216 -36.50280484500258, -71.35560138824567 -36.51043722808533, -71.26342881157782 -36.25719515499742, -71.28439861175431 -36.249562771914675, -71.37657118842216 -36.50280484500258))",
                _polygonToSubtract: new List<string>(),
                _requestChannel: 0,
                _shootingType: ShootingType.Normal,
                _compression: 1,
                _albedo: 0
                );





            RequestParams req7 = new RequestParams(341, 1,
                DateTime.Parse("2010-02-04T00:00:00"),
                DateTime.Parse("2029-02-04T00:00:00"),
                _Max_SOEN_anlge: 12,
                _minCoverPerc: 341,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: "POLYGON((-70.48618020457003 -33.372947151486834, -70.46599237209783 -33.38029492160107, -70.37381979542998 -33.127052848513166, -70.39400762790218 -33.11970507839893, -70.48618020457003 -33.372947151486834))",
                _polygonToSubtract: new List<string>(),
                _requestChannel: 0,
                _shootingType: ShootingType.Normal,
                _compression: 1,
                _albedo: 0
                );





            RequestParams req8 = new RequestParams(342, 1,
                DateTime.Parse("2010-02-04T00:00:00"),
                DateTime.Parse("2029-02-04T00:00:00"),
                _Max_SOEN_anlge: 12,
                _minCoverPerc: 342,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: "POLYGON((-69.55580917930405 -29.873082193639796, -69.5363633973638 -29.880159879448087, -69.44419082069595 -29.62691780636019, -69.4636366026362 -29.619840120551913, -69.55580917930405 -29.873082193639796))",
                _polygonToSubtract: new List<string>(),
                _requestChannel: 0,
                _shootingType: ShootingType.Normal,
                _compression: 2,
                _albedo: 0
                );





            RequestParams req9 = new RequestParams(343, 1,
                DateTime.Parse("2010-02-04T00:00:00"),
                DateTime.Parse("2029-02-04T00:00:00"),
                _Max_SOEN_anlge: 12,
                _minCoverPerc: 343,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: "POLYGON((-68.56544500089616 -25.703214743740233, -68.5467275757717 -25.710027329347625, -68.45455499910385 -25.456785256259764, -68.47327242422831 -25.449972670652357, -68.56544500089616 -25.703214743740233))",
                _polygonToSubtract: new List<string>(),
                _requestChannel: 0,
                _shootingType: ShootingType.Normal,
                _compression: 1,
                _albedo: 0
                );





            RequestParams req10 = new RequestParams(344, 1,
                DateTime.Parse("2010-02-04T00:00:00"),
                DateTime.Parse("2029-02-04T00:00:00"),
                _Max_SOEN_anlge: 12,
                _minCoverPerc: 344,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: "POLYGON((-67.38539156998534 -20.947821641487295, -67.28779055761508 -20.983345504824328, -67.25460843001464 -20.892178358512695, -67.3522094423849 -20.856654495175675, -67.38539156998534 -20.947821641487295))",
                _polygonToSubtract: new List<string>(),
                _requestChannel: 0,
                _shootingType: ShootingType.Normal,
                _compression: 1,
                _albedo: 0
                );
#endregion


            DateTime dt1 = DateTime.Parse("2019-02-01T10:00:00");// new DateTime(2019, 2, 18, 2, 0, 0);
            DateTime dt2 = DateTime.Parse("2019-02-01T20:00:00");// new DateTime(2019, 2, 18, 3, 0, 0);

            List<RequestParams> reqlist = new List<RequestParams> { req0,  req1, req2, req3, req4, req5, req6, req7, req8, req9, req10 };


            //Console.Write("GEOMETRYCOLLECTION(");
            //Console.Write(Polygon.getMultipolFromPolygons(reqlist.Select(req => new Polygon(req.wktPolygon) ).ToList()));
            //Console.Write(")");

            //return;

            List<Tuple<DateTime, DateTime>> silenceRanges = new List<Tuple<DateTime, DateTime>>();
            List<Tuple<DateTime, DateTime>> inactivityRanges = new List<Tuple<DateTime, DateTime>>();

            List<RouteMPZ> routesToDrop = new List<RouteMPZ>();
            List<RouteMPZ> routesToDelete = new List<RouteMPZ>();
            string cupConnStr = System.IO.File.ReadLines("DBstring.conf").First();
            string cuksConnStr = System.IO.File.ReadLines("DBstringCUKS.conf").First();
            DIOS.Common.SqlManager CUKSmanagerDB = new DIOS.Common.SqlManager(cuksConnStr);
            DIOS.Common.SqlManager CUPmanagerDB = new DIOS.Common.SqlManager(cupConnStr);

            List<MPZ> mpzArray;
            List<CommunicationSession> sessions;

            List<TimePeriod> shadowPeriods;
            List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot;
            Sessions.checkIfViewLaneIsLitWithTimeSpans(CUPmanagerDB, dt1, dt2, out partsLitAndNot, out shadowPeriods);
            List<TimePeriod> shadowAndInactivityPeriods = new List<TimePeriod>();
            shadowAndInactivityPeriods.AddRange(inactivityRanges.Select(t => new TimePeriod(t.Item1, t.Item2)));
            shadowAndInactivityPeriods.AddRange(shadowPeriods);
            shadowAndInactivityPeriods = TimePeriod.compressTimePeriods(shadowAndInactivityPeriods);


    //        List<CaptureConf> confsToCapture
    //= Sessions.getCaptureConfArray(
    //reqlist,
    //dt1, dt2,
    //CUPmanagerDB,
    //CUKSmanagerDB,
    //inactivityRanges.Select(t => new TimePeriod(t.Item1, t.Item2)).ToList(),
    //new List<TimePeriod>());

    //        Console.Write("GEOMETRYCOLLECTION(");
    //        Console.Write(Polygon.getMultipolFromPolygons(reqlist.Select(req => new Polygon(req.wktPolygon)).ToList()));
    //        Console.Write(",");
    //        Console.Write(Polygon.getMultipolFromPolygons(confsToCapture.Select(c => new Polygon(c.wktPolygon)).ToList()));
    //        Console.Write(")");

    //        return;
            Sessions.getMPZArray(reqlist, dt1, dt2
                    , silenceRanges
                    , inactivityRanges
                    , routesToDrop
                    , routesToDelete
                    , cupConnStr
                    , cuksConnStr
                    , 356
                    , out mpzArray
                    , out sessions
                    , new List<SessionsPlanning.CommunicationSessionStation> 
            { SessionsPlanning.CommunicationSessionStation.FIGS_Main,
                SessionsPlanning.CommunicationSessionStation.FIGS_Backup,
                SessionsPlanning.CommunicationSessionStation.MIGS }
                    );

            var shootingRoutes = mpzArray.SelectMany(mpz => mpz.Routes
                    .Where(r => r.Parameters.type == WorkingType.Shooting || r.Parameters.type == WorkingType.ShootingSending)).ToList();

            var shootingPolygons = shootingRoutes.Select(r => new Polygon(r.Parameters.ShootingConf.wktPolygon)).ToList();


            Console.Write("GEOMETRYCOLLECTION(");
            Console.Write(Polygon.getMultipolFromPolygons(reqlist.Select(req => new Polygon(req.wktPolygon)).ToList()));
            Console.Write(",");
           // Console.WriteLine(WktTestingTools.getWKTStrip(dt1, dt2));
            Console.Write(Polygon.getMultipolFromPolygons(shootingPolygons));
            Console.Write(")");

        }

        static public void test_getPlainMpzArray()
        {
            //test_checkIfViewLaneIsLit();
            //return;
            DateTime dt1 = DateTime.Parse("2019-02-01T00:00:00+03:00");// new DateTime(2019, 2, 18, 2, 0, 0);
            DateTime dt2 = DateTime.Parse("2019-02-04T00:00:00");// new DateTime(2019, 2, 18, 3, 0, 0);
            
            string cupConnStr = System.IO.File.ReadLines("DBstring.conf").First();
            string cuksConnStr = System.IO.File.ReadLines("DBstringCUKS.conf").First();
            DIOS.Common.SqlManager CUKSmanagerDB = new DIOS.Common.SqlManager(cuksConnStr);
            DIOS.Common.SqlManager CUPmanagerDB = new DIOS.Common.SqlManager(cupConnStr);

            string polwtk = "POLYGON((153.17138671875 -28.557988492481016, 153.20983886718747 -28.021075462659887, 153.10546874999997 -28.01137657176146, 153.03955078124997 -28.548338387631418, 153.17138671875 -28.557988492481016))";

            List<string> holes = new List<string>();

            RequestParams req = new RequestParams(0, 1, dt1, dt2,
                _Max_SOEN_anlge: 0.87266462599716477,
                _minCoverPerc: 11,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: polwtk,
                _polygonToSubtract: holes,
                _requestChannel: 0,
                _shootingType: ShootingType.Normal,
                _compression: 10,
                _albedo: 0
                );

            List<Tuple<DateTime, DateTime>> silenceRanges = new List<Tuple<DateTime, DateTime>>();
            List<Tuple<DateTime, DateTime>> inactivityRanges = new List<Tuple<DateTime, DateTime>>();

            List<RouteMPZ> routesToDrop = new List<RouteMPZ>();
            List<RouteMPZ> routesToDelete = new List<RouteMPZ>();

            List<MPZ> mpzArray;
            List<CommunicationSession> sessions;

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
            , new List<SessionsPlanning.CommunicationSessionStation> 
            { SessionsPlanning.CommunicationSessionStation.FIGS_Main,
                SessionsPlanning.CommunicationSessionStation.FIGS_Backup,
                SessionsPlanning.CommunicationSessionStation.MIGS }
            );

            Console.WriteLine("res.Count = {0}", mpzArray.Count());

            if (mpzArray.Count() == 0)
                return;


            //var modRouteParams = new RouteParams(mpzArray.First().Routes.First().Parameters);
            //var copyMpzParams = mpzArray.First().Parameters;
            //if (copyMpzParams.InsertRoute(modRouteParams, DateTime.MinValue, DateTime.MaxValue))
            //{
            //    Console.Write("OOOK");
            //}

            Console.Write("GEOMETRYCOLLECTION(");
            Console.Write(polwtk);
            Console.Write(",");
            Console.Write(Polygon.getMultipolFromPolygons(mpzArray.SelectMany(mpz => mpz.Routes.Select(r => new Polygon(r.Parameters.ShootingConf.wktPolygon))).ToList()));
            Console.Write(")");

            //double roll = mpzArray.First().Routes.First().Parameters.ShootingConf.roll;
            //double pitch = mpzArray.First().Routes.First().Parameters.ShootingConf.pitch;
            //DateTime start = mpzArray.First().Routes.First().startTime;

            //int duration = mpzArray.First().Routes.First().Troute * 200;
            //string viewPol = Sessions.getSOENViewPolygon(start, roll, pitch, duration, CUPmanagerDB, false);
            //Console.Write(viewPol);
        }



        static public void test_TestSessionsSequenses()
        {
            DateTime fromDt = DateTime.Parse("20.02.2019 0:0:0");
            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);

            string cs2 = System.IO.File.ReadLines("DBstringCUKS.conf").First();
            DIOS.Common.SqlManager managerDbCUKS = new DIOS.Common.SqlManager(cs2);



            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get14PlainFrames1Turn(fromDt, managerDB, managerDbCUKS, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get45PlainFrames4Turn(fromDt, managerDB, managerDbCUKS, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get8StereoTriplets1Turn(fromDt, managerDB, managerDbCUKS, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get8StereoPairs1Turn(fromDt, managerDB, managerDbCUKS, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get20StereoTriplets5Turn(fromDt, managerDB, managerDbCUKS, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get20StereoPairs5Turn(fromDt, managerDB, managerDbCUKS, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get8Coridors1Turn(fromDt, managerDB, managerDbCUKS, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get20Coridors5Turn(fromDt, managerDB, managerDbCUKS, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get8AreaShooting1Turn(fromDt, managerDB, managerDbCUKS, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get20AreaShooting5Turn(fromDt, managerDB, managerDbCUKS, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.getStrip4150km1Turn(fromDt, managerDB, managerDbCUKS, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.getStrip12050km5Turn(fromDt, managerDB, managerDbCUKS, out mpzArray);
            }

            List<MPZ> mpzArray2;
            SessionsPlanning.TestSessionsSequenses.get20AreaShooting5Turn(fromDt, managerDB, managerDbCUKS, out mpzArray2);

            var orderPolsList = mpzArray2.SelectMany(mpz => mpz.Routes.SelectMany(r => r.Parameters.ShootingConf.orders.Select(order => order.captured))).ToList();
            var shootingPolsList = mpzArray2.SelectMany(mpz => mpz.Routes.Select(r => new Polygon(r.Parameters.ShootingConf.wktPolygon))).ToList();

            Console.Write("GEOMETRYCOLLECTION(");
            Console.Write(Polygon.getMultipolFromPolygons(orderPolsList));
            Console.Write(",");

            Console.Write(Polygon.getMultipolFromPolygons(shootingPolsList));
            Console.Write(")");
        }

        static void Main(string[] args)
        {
            test_error();
            //test_getCaptureConfArray();
            Console.ReadKey();
        }
    }
}
