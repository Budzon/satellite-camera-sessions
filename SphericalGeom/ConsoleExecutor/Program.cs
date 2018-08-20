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
using Microsoft.SqlServer.Types;
using System.Data.SqlTypes;

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

        
        static public void test_checkIfViewLaneIsLit()
        {
            //DateTime dt1 = DateTime.Parse("2019-01-01T00:00:00");// new DateTime(2019, 2, 18, 2, 0, 0);
            //DateTime dt2 = DateTime.Parse("2019-02-01T00:00:00");// new DateTime(2019, 2, 18, 3, 0, 0);
            //List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot;
            //Sessions.checkIfViewLaneIsLit(System.IO.File.ReadLines("DBstring.conf").First(), dt1, dt2, out partsLitAndNot);
            //Console.WriteLine(partsLitAndNot.Count);
            //Console.WriteLine();
            //foreach (var tuple in partsLitAndNot)
            //{
            //    Console.WriteLine(tuple.Item1);
            //    Console.WriteLine();
            //    Console.WriteLine(Polygon.getMultipolFromPolygons(  tuple.Item2.Select(wpl => new Polygon(wpl.wktPolygon)).ToList() ));
            //}

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

            List<TimePeriod> shadowPeriods = new List<TimePeriod>();
            List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot;
          //  Sessions.checkIfViewLaneIsLitWithTimeSpans(CUPmanagerDB, dt1, dt2, out partsLitAndNot, out shadowPeriods);
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

        static public void test_getCoridorMpzArray()
        {
            DateTime dt1 = DateTime.Parse("2019-02-01T00:00:00");// new DateTime(2019, 2, 18, 2, 0, 0);
            DateTime dt2 = DateTime.Parse("2019-02-03T00:00:00");// new DateTime(2019, 2, 18, 3, 0, 0);

            string cupConnStr = System.IO.File.ReadLines("DBstring.conf").First();
            string cuksConnStr = System.IO.File.ReadLines("DBstringCUKS.conf").First();
            DIOS.Common.SqlManager CUKSmanagerDB = new DIOS.Common.SqlManager(cuksConnStr);
            DIOS.Common.SqlManager CUPmanagerDB = new DIOS.Common.SqlManager(cupConnStr);

            List<string> wktList = new List<string>(){
           // "POLYGON((149.51293945312497 -30.090484220005344,149.5404052734375 -30.033433263128984,149.56787109375 -29.95969381418451,149.60906982421875 -29.893043385434176,149.65301513671875 -29.850173125689892,149.688720703125 -29.81681685764992,149.68597412109372 -29.80251790576446,149.64202880859372 -29.833496383743203,149.59945678710935 -29.872801391992795,149.55825805664062 -29.926374178635747,149.5184326171875 -29.99538103356813,149.4978332519531 -30.041755241903317,149.48959350585935 -30.073847754270204,149.51293945312497 -30.090484220005344))"
          //  ,"POLYGON((-54.64599609374999 -25.32416652573839,-56.260986328124986 -26.961245770526972,-56.326904296875 -27.186242185608734,-56.04125976562499 -27.196014383173292,-55.118408203124986 -26.155437968713542,-53.98681640625 -25.025884063244817,-53.3056640625 -24.55711616430962,-51.95434570312499 -24.116674961751286,-50.614013671875 -23.815500848699656,-50.526123046875 -23.50355189742413,-51.05346679687501 -23.51362636346272,-51.94335937499999 -23.76523688975867,-52.80029296874999 -24.05649649076851,-53.76708984374999 -24.467150664738995,-54.3603515625 -24.886436490787702,-54.64599609374999 -25.32416652573839))"
            "POLYGON((-63.687744140624986 75.16330024622056,-63.06152343749998 75.11822201684026,-62.21557617187499 75.14641157325434,-61.380615234374986 75.21946035674114,-60.611572265624986 75.33950166120164,-60.35888671874999 75.49990834816703,-60.02929687499998 75.89344704833564,-59.732666015624986 76.02140235430915,-57.304687499999986 76.10079606754579,-56.44775390624999 76.06909176390045,-55.66772460937499 76.00281329293702,-55.129394531249986 75.89612445246954,-54.68994140624999 75.7940323033921,-54.26147460937499 75.79133591604713,-54.33837890624999 75.86127933376639,-54.61303710937499 75.93355587409607,-55.01953124999999 76.01609366420993,-55.612792968749986 76.08759454588619,-56.096191406249986 76.12716228011627,-56.67846679687499 76.15084994303763,-57.183837890625 76.15873701495826,-57.81005859374999 76.15347945648796,-58.61206054687499 76.12716228011627,-59.23828124999999 76.10079606754579,-59.67773437499999 76.07438073300233,-60.12817382812499 76.05586072525739,-60.46875 75.98686053114261,-60.64453124999999 75.85054080189764,-60.71044921875 75.73188842222964,-60.666503906249986 75.59860599198842,-60.897216796875 75.40885422846455,-61.10595703125 75.36172949075694,-61.49047851562499 75.29215785826014,-62.34741210937499 75.21385371117896,-62.98461914062499 75.16048677152295,-63.446044921875 75.18578927942625,-63.90747070312499 75.25585291932208,-64.171142578125 75.23346787838844,-63.687744140624986 75.16330024622056))"
          //  ,"POLYGON((143.49517822265625 73.23620856434462,143.26995849609378 73.22907740190121,142.97882080078125 73.21004656128417,142.5421142578125 73.1639687361907,142.2784423828125 73.14088379815641,141.95983886718744 73.07464313148589,140.90515136718747 72.698021973724,141.17431640625 72.67268149675314,141.43798828125 72.76406472320437,141.83349609375 72.89380193362285,142.53662109375 73.02900629225599,143.4375 73.15680773175978,144.2724609375 73.18861179538933,144.8876953125 73.25837557295395,145.6787109375 73.37192828625788,146.09619140625 73.52839948765174,146.46972656250003 73.73890457673937,146.57958984375 74.04976184989872,146.35986328125 74.22393470166762,145.8544921875 74.22990748716416,145.458984375 74.14008387440461,145.74462890625003 74.07389660001783,146.0302734375 74.08595063145893,146.27197265625 73.97107763933991,146.14013671875003 73.81257376708521,145.89843750000003 73.64017137216683,145.78857421875 73.5221682137352,145.43701171875 73.42215560370377,144.90966796875 73.33416078093367,144.44824218750003 73.26470377263848,143.83300781250003 73.21401304858364,143.49517822265625 73.23620856434462))"
                };

            List<string> holes = new List<string>();

            List<RequestParams> reqlist = wktList.Select(polwtk =>
             new RequestParams(0, 1, dt1, dt2,
                _Max_SOEN_anlge: 0.87266462599716477,
                _minCoverPerc: 11,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: polwtk,
                _polygonToSubtract: holes,
                _requestChannel: 0,
                _shootingType: ShootingType.Coridor,
                _compression: 10,
                _albedo: 0
                )
             ).ToList();

            List<Tuple<DateTime, DateTime>> silenceRanges = new List<Tuple<DateTime, DateTime>>();
            List<Tuple<DateTime, DateTime>> inactivityRanges = new List<Tuple<DateTime, DateTime>>();

            List<RouteMPZ> routesToDrop = new List<RouteMPZ>();
            List<RouteMPZ> routesToDelete = new List<RouteMPZ>();

            List<MPZ> mpzArray;
            List<CommunicationSession> sessions;


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
            {   SessionsPlanning.CommunicationSessionStation.FIGS_Main,
                SessionsPlanning.CommunicationSessionStation.FIGS_Backup,
                SessionsPlanning.CommunicationSessionStation.MIGS }
            );

            Console.WriteLine("res.Count = {0}", mpzArray.Count());

            if (mpzArray.Count() == 0)
                return;

            var shootingRoutes = mpzArray.SelectMany(mpz => mpz.Routes
                    .Where(r => r.Parameters.type == WorkingType.Shooting || r.Parameters.type == WorkingType.ShootingSending)).ToList();

            var shootingPolygons = shootingRoutes.Select(r => new Polygon(r.Parameters.ShootingConf.wktPolygon)).ToList();

            Console.Write("GEOMETRYCOLLECTION(");
            Console.Write(Polygon.getMultipolFromPolygons(reqlist.Select(r => new Polygon(r.wktPolygon)).ToList()));
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
            DateTime dt2 = DateTime.Parse("2019-02-15T00:00:00");// new DateTime(2019, 2, 18, 3, 0, 0);
            
            string cupConnStr = System.IO.File.ReadLines("DBstring.conf").First();
            string cuksConnStr = System.IO.File.ReadLines("DBstringCUKS.conf").First();
            DIOS.Common.SqlManager CUKSmanagerDB = new DIOS.Common.SqlManager(cuksConnStr);
            DIOS.Common.SqlManager CUPmanagerDB = new DIOS.Common.SqlManager(cupConnStr);

            List<string> wktList = new List<string>(){
            "POLYGON((153.17138671875 -28.557988492481016, 153.20983886718747 -28.021075462659887, 153.10546874999997 -28.01137657176146, 153.03955078124997 -28.548338387631418, 153.17138671875 -28.557988492481016))"
            ,"POLYGON((154.3393780386478 8.456259424457386,153.88343525756835 7.531485504923765,154.98209256137423 7.346285806781779,154.53713635333284 8.075724865506643,154.3393780386478 8.456259424457386))"
            ,"POLYGON((38.583984375 -10.57422207833281,37.529296875 -11.910353555774094,38.3203125 -11.609193407938946,38.583984375 -10.57422207833281))"
            ,"POLYGON((175.93505859375 52.70301871296326,175.30883789062497 52.2008737173322,176.1767578125 51.99841038239032,176.451416015625 52.43592058359013,175.93505859375 52.70301871296326))"
            ,"POLYGON((293.88977050781244 18.672267305093754,293.8623046875 18.536908560288467,293.97216796874994 18.531700307384043,294.049072265625 18.67747125852607,293.88977050781244 18.672267305093754))"
            ,"POLYGON((316.38427734375 59.88893689676584,316.351318359375 59.81444699201484,316.48315429687506 59.80616004020658,316.59301757812506 60.00722509306874,316.38427734375 59.88893689676584))"
            ,"POLYGON((356.5118408203125 48.81228985866255,356.48712158203125 48.78515199804312,356.56127929687506 48.7742927426751,356.5667724609375 48.81228985866255,356.5118408203125 48.81228985866255))"
            ,"POLYGON((370.19531249999994 49.52164252537975,370.16784667968744 49.475263243038,370.2227783203124 49.45741335279223,370.19531249999994 49.52164252537975))"
            };

            List<string> holes = new List<string>();

            List<RequestParams> reqlist = wktList.Select(polwtk =>
             new RequestParams(0, 1, dt1, dt2,
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
                )
             ).ToList();

            List<Tuple<DateTime, DateTime>> silenceRanges = new List<Tuple<DateTime, DateTime>>();
            List<Tuple<DateTime, DateTime>> inactivityRanges = new List<Tuple<DateTime, DateTime>>();

            List<RouteMPZ> routesToDrop = new List<RouteMPZ>();
            List<RouteMPZ> routesToDelete = new List<RouteMPZ>();

            List<MPZ> mpzArray;
            List<CommunicationSession> sessions;
                       

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
            {   SessionsPlanning.CommunicationSessionStation.FIGS_Main,
                SessionsPlanning.CommunicationSessionStation.FIGS_Backup,
                SessionsPlanning.CommunicationSessionStation.MIGS }
            );

            Console.WriteLine("res.Count = {0}", mpzArray.Count());

            if (mpzArray.Count() == 0)
                return;

            var shootingRoutes = mpzArray.SelectMany(mpz => mpz.Routes
                    .Where(r => r.Parameters.type == WorkingType.Shooting || r.Parameters.type == WorkingType.ShootingSending)).ToList();

            var shootingPolygons = shootingRoutes.Select(r => new Polygon(r.Parameters.ShootingConf.wktPolygon)).ToList();
            
            Console.Write("GEOMETRYCOLLECTION(");
            Console.Write(Polygon.getMultipolFromPolygons(reqlist.Select(r => new Polygon(r.wktPolygon)).ToList()));
            Console.Write(",");
            // Console.WriteLine(WktTestingTools.getWKTStrip(dt1, dt2));
            Console.Write(Polygon.getMultipolFromPolygons(shootingPolygons));
            Console.Write(")");
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

        static void test_Polygons()
        {
            string lineWkt = "POLYGON((249.290771484375 43.91372326852402,249.3017578125 44.621754096233246,249.38964843750003 45.52944081525669,249.8291015625 46.308995694198586,251.47705078125003 46.58906908309183,254.168701171875 46.59661864884464,257.794189453125 46.98025235521882,259.63989257812506 47.36859434521338,261.826171875 47.702368466573716,263.946533203125 48.18806348121143,266.890869140625 48.900837902340896,270.098876953125 49.80254124506294,275.350341796875 50.6599083609394,277.84423828125 51.213765926935025,284.91943359375 52.30176096373671,287.11669921875 52.50953477032729,289.0283203125 52.542955066421285,289.31396484375 52.38901106223457,288.96240234375 52.34876318198809,288.006591796875 52.27488013068054,286.787109375 52.221069523572794,284.4580078125 52.03897658307622,282.315673828125 51.611194610484034,281.2060546875 51.542918822373906,278.89892578125006 51.06211251399776,276.43798828125006 50.6181029049248,275.0537109375 50.2682767372753,273.570556640625 49.98655213050617,271.32934570312494 49.88047763874255,267.91259765624994 48.91527985344385,265.133056640625 48.3270391306348,263.90258789062494 47.931066347509784,261.968994140625 47.45037978769005,259.266357421875 46.98025235521882,255.22338867187497 46.45299704748288,253.44360351562497 46.2634426717799,252.35595703125 46.11132565729798,251.070556640625 45.91294412737392,250.24658203125 45.544831492424635,250.07080078125 45.034714778688624,249.78515624999994 43.73935207915471,248.939208984375 42.27730877423707,247.51098632812494 41.170384477816185,244.99511718749991 40.40513069752788,240.12817382812503 39.36827914916017,239.82055664062503 39.77476948529545,241.95190429687503 40.17887331434699,245.06103515625 40.9964840143779,248.35693359375 43.30919109985686,248.785400390625 43.580390855607874,249.290771484375 43.91372326852402))";

            List<string> wktList = new List<string>()
            {              
"POLYGON((248.73046874999997 36.4566360115962,240.556640625 47.813154517527664,243.45703125 48.107431188480376,250.83984375000003 36.597889133070225,248.73046874999997 36.4566360115962))",

"POLYGON((251.80664062500003 41.17865397233169,272.46093749999994 42.42345651793826,272.98828125 39.23225314171492,254.00390625 39.300299186150255,251.80664062500003 41.17865397233169))",

"POLYGON((255.32226562499997 48.40003249610686,255.41015625 46.01222384063237,258.57421875 46.01222384063237,257.87109375 48.107431188480376,255.32226562499997 48.40003249610686))",

"POLYGON((267.86865234374994 49.37879656435038,268.03894042968744 49.1314084139986,268.363037109375 49.24987918000417,268.1048583984375 49.44312875803007,267.86865234374994 49.37879656435038))",

"POLYGON((273.31787109375 50.155785885384546,273.71337890625 50.282319450081616,274.317626953125 50.2963580211032,273.834228515625 50.17689812200109,273.31787109375 50.155785885384546))",

"POLYGON((276.24023437499994 50.14874640066279,276.59179687499994 51.43346414054375,275.84472656249994 51.48138289610097,276.24023437499994 50.14874640066279))",

"POLYGON((278.11889648437494 52.58970076871776,279.16259765625 52.709675332198856,279.76684570312494 52.809402810688084,280.645751953125 52.8293209103137,281.4697265625 52.842594572239506,282.392578125 52.81604319154934,283.12866210937494 52.71633093604632,284.7216796875 52.789475581398875,285.47973632812506 52.789475581398875,286.138916015625 52.789475581398875,286.69921875 52.709675332198856,287.33642578125 52.509534770327235,287.490234375 52.234528294213646,287.60009765625006 51.917167589090184,287.611083984375 51.570241445811234,286.67724609375 51.22064743038331,285.5126953125 50.82675848236329,284.403076171875 50.59718623058703,283.106689453125 50.33844888725474,281.79931640625 50.24720490139268,278.41552734374994 50.1628243338173,277.965087890625 50.205033264943324,277.459716796875 50.42951794712289,277.3828125 50.701676635764784,277.89916992187494 51.096622945029935,279.10766601562494 51.97811348548811,278.75610351562494 52.126743859642914,278.36059570312494 52.27488013068054,278.031005859375 52.38901106223457,278.11889648437494 52.58970076871776))",

"POLYGON((278.15185546874994 52.616390233045394,278.15185546875 53.402982494248164,275.47119140625 53.80065082633024,275.537109375 52.76289173758374,275.614013671875 52.01869808104436,278.4375 51.95780738871554,278.15185546874994 52.616390233045394))",

"POLYGON((287.65502929687494 53.173119202640635,291.763916015625 51.82898836366914,288.533935546875 51.713416052417614,286.973876953125 51.835777520452496,285.831298828125 52.09975692575725,285.941162109375 52.74294319885715,286.259765625 53.17970389360539,287.35839843749994 53.376774975060215,287.92968749999994 53.265212931246566,288.533935546875 53.21919081798936,289.830322265625 53.08082737207479,289.423828125 52.975108181735294,287.65502929687494 53.173119202640635))",
"POLYGON((244.27001953125 42.00032514831625,247.17041015625 40.128491056854074,247.60986328124997 40.32979574370208,247.3681640625 40.630630083991804,247.10449218749997 40.896905775860006,246.90673828125 41.29431726315258,246.708984375 41.78769700539064,246.357421875 42.27730877423707,246.11572265624997 42.69858589169843,245.54443359375 43.229195113965005,244.51171874999997 43.56447158721812,244.00634765624997 43.421008829947255,243.19335937499997 43.29320031385282,242.68798828124997 43.11702412135048,242.64404296875 42.811521745097906,243.12744140625 42.55308028895581,243.61083984375 42.293564192170095,244.27001953125 42.00032514831625))", 
"POLYGON((262.44140625 50.45750402042057,263.89160156249994 51.0137546571882,265.7373046875 51.261914853084505,267.27539062499994 51.50874245880334,268.857421875 51.64529404930536,270.263671875 51.64529404930536,271.4501953125 51.53608560178475,272.197265625 51.12421275782688,272.76855468749994 50.40151532278236,272.94433593749994 49.582226044621734,272.8564453125 48.74894534343292,272.24121093749994 48.01932418480118,271.0107421875 46.86019101567027,269.25292968749994 45.92058734473366,267.27539062499994 45.12005284153054,264.33105468749994 44.8714427501659,261.5185546875 44.964797930331,259.892578125 45.39844997630408,258.83789062499994 46.286223918067066,258.35449218749994 47.60616304386872,258.4423828125 48.45835188280864,258.9697265625 49.38237278700956,259.67285156249994 50.17689812200109,261.5185546875 50.65294336725708,262.44140625 50.45750402042057))", 
"POLYGON((288.28125 52.86912972768522,288.489990234375 52.789475581398875,288.621826171875 52.60971939156647,288.66577148437494 52.44261787120723,288.66577148437494 52.194139741597525,288.544921875 52.03897658307622,288.138427734375 51.815406979494384,287.6220703125 51.67936786087719,287.05078124999994 51.64529404930536,286.92993164062506 51.70660846336452,288.28125 52.86912972768522))", 
"POLYGON((281.75537109375 54.46365264504479,281.55761718749994 54.514704495736936,281.239013671875 54.52108149544361,280.810546875 54.4381028097402,280.5029296875 54.265224078605684,280.17333984375 54.072282655603885,279.942626953125 53.820111769559674,279.72290039062494 53.54030739150022,279.66796875 53.337433437129675,279.7998046875 53.08742621937262,279.90966796875 52.96849212681394,280.40405273437494 52.8823912222619,280.986328125 52.842594572239506,281.7333984375 52.8492298820527,282.32666015624994 53.05442186546102,282.65624999999994 53.3702205739568,282.83203124999994 53.83308071272799,282.91992187499994 54.252389302768506,282.4365234375 54.46365264504479,282.06298828125 54.46365264504479,281.75537109375 54.46365264504479))", 
"POLYGON((277.6904296875 49.72447918871299,277.5146484375 50.0641917366591,276.81152343749994 50.76425935711646,276.32812499999994 51.50874245880334,275.7568359375 52.21433860825823,275.05371093749994 52.935396658623205,274.21874999999994 53.80065082633024,273.25195312499994 54.13669645687003,272.63671874999994 54.31652324025825,271.8896484375 54.470037612805754,270.5712890625 54.470037612805754,270 54.05938788662357,269.95605468749994 53.696706475303245,270.65917968749994 53.199451902831555,272.32910156249994 52.802761415419695,273.55957031249994 52.321910885947716,274.6142578125 51.672555148396754,275.36132812499994 50.79204706440686,275.49316406249994 50.26125382758474,275.2734375 49.21042044565033,274.65820312499994 47.989921667414166,274.482421875 46.920255315374504,274.482421875 45.64476821775193,275.09765625 44.402391829093915,277.16308593749994 44.27667127377518,277.91015624999994 44.8714427501659,278.12988281249994 45.85941212790755,278.21777343749994 46.67959446564018,278.12988281249994 47.9605023889151,277.64648437499994 49.32512199104002,277.6904296875 49.72447918871299))", 
"POLYGON((254.61914062500003 39.19820534889479,255.19042968750003 41.40977583200956,255.322265625 42.8437513262902,255.49804687500003 43.64402584769951,255.76171875000006 44.433779846068205,255.89355468750003 45.82879925192134,255.76171875000006 47.12995075666308,255.19042968750003 48.719961222646276,253.74023437500003 50.0641917366591,252.15820312499997 50.93073802371819,251.76269531249997 52.07950600379698,251.19140624999997 53.357108745695996,249.9169921875 54.44449176335766,247.8515625 54.85131525968609,246.0498046875 54.85131525968609,244.248046875 54.44449176335766,243.5888671875 53.82659674299413,243.5888671875 53.357108745695996,244.423828125 52.88239122226193,245.6982421875 52.42922227795512,247.19238281249994 52.802761415419695,248.291015625 52.802761415419695,249.609375 51.727028157047755,250.13671875 50.31740811261872,251.2353515625 49.03786794532644,251.4111328125 49.69606181911567,252.46582031249997 48.632908585895365,253.12499999999997 46.49839225859765,253.21289062500006 43.83452678223685,252.81738281250003 41.705728515237524,252.861328125 40.27952566881291,253.95996093750003 39.16414104768742,254.61914062500003 39.19820534889479))",
            "POLYGON((153.17138671875 -28.557988492481016, 153.20983886718747 -28.021075462659887, 153.10546874999997 -28.01137657176146, 153.03955078124997 -28.548338387631418, 153.17138671875 -28.557988492481016))",
            "POLYGON((154.3393780386478 8.456259424457386,153.88343525756835 7.531485504923765,154.98209256137423 7.346285806781779,154.53713635333284 8.075724865506643,154.3393780386478 8.456259424457386))",
            "POLYGON((38.583984375 -10.57422207833281,37.529296875 -11.910353555774094,38.3203125 -11.609193407938946,38.583984375 -10.57422207833281))"
            };
            
            double ours_sum = 0;
            double theirs_sum = 0;

            for (int i = 0; i < 1; i++)
            {
                ours_sum += ours_Polygons(lineWkt, wktList);
                theirs_sum += theirs_Polygons(lineWkt, wktList);
            }
            double ours = ours_sum / 100;
            double theirs = theirs_sum / 100;
            Console.WriteLine("Вариант с  SqlGeography быстрее в {0} раз", ours / theirs);
        }

        static double ours_Polygons(string wkt, List<string> wktList)
        {
            Polygon mainPol = new Polygon(wkt);
            List<Polygon> polygons = wktList.Select(s => new Polygon(s)).ToList();
             
            
            DateTime start = DateTime.Now;
            //for (int i = 0; i < 400; i++)
            foreach (var pol in polygons)
            {
                IList<Polygon> res = Polygon.Intersect(mainPol, pol);
                
            }
            DateTime end = DateTime.Now;
           // Console.WriteLine("ours_Polygons : {0} ", (end - start).ToString());
            return (end - start).TotalMilliseconds;
        }

        static double theirs_Polygons(string wkt, List<string> wktList)
        {
            SqlGeography mainPol = SqlGeography.STGeomFromText(new SqlChars(wkt), 4326);
            List<SqlGeography> polygons = wktList.Select(s => SqlGeography.STGeomFromText(new SqlChars(wkt), 4326)).ToList();


            DateTime start = DateTime.Now;
            //for (int i = 0; i < 400; i++)
                foreach (var pol in polygons)
                {
                    SqlGeography res = mainPol.STIntersection(pol);
                   
                }
            DateTime end = DateTime.Now;
          //  Console.WriteLine("theirs_Polygons : {0} ", (end - start).ToString());
            return (end - start).TotalMilliseconds;
        }

        static void test_ours_Polygons_smallcircle()
        {
            string pol1wkt = "POLYGON((-125.5517578125 53.64463782485652,-71.3232421875 52.66972038368817,-71.2353515625 53.54030739150022,-125.4638671875 54.54657953840504,-125.5517578125 53.64463782485652))";
            string pol2wkt = "POLYGON((-102.39257812500001 43.51668853502909,-100.72265625 43.51668853502909,-99.49218749999996 64.01449619484472,-100.72265624999999 64.01449619484472,-102.39257812500001 43.51668853502909))";

            Polygon pol1 = new Polygon(pol1wkt);
            Polygon pol2 = new Polygon(pol2wkt);

            IList<Polygon> res = Polygon.Intersect(pol1, pol2);

            Console.WriteLine("GEOMETRYCOLLECTION(");
            Console.WriteLine(pol1.ToWtk());
            Console.WriteLine(",");
            Console.WriteLine(res.First().ToWtk());
            Console.WriteLine(",");
            Console.WriteLine(pol2.ToWtk());
            Console.WriteLine(")");
        }

        static void test_theirs_Polygons_smallcircle()
        {
            string pol1wkt = "POLYGON((-125.5517578125 53.64463782485652,-71.3232421875 52.66972038368817,-71.2353515625 53.54030739150022,-125.4638671875 54.54657953840504,-125.5517578125 53.64463782485652))";
            string pol2wkt = "POLYGON((-102.39257812500001 43.51668853502909,-100.72265625 43.51668853502909,-99.49218749999996 64.01449619484472,-100.72265624999999 64.01449619484472,-102.39257812500001 43.51668853502909))";

            SqlGeography pol1 = SqlGeography.STGeomFromText(new SqlChars(pol1wkt), 4326);
            SqlGeography pol2 = SqlGeography.STGeomFromText(new SqlChars(pol2wkt), 4326);

            SqlGeography res = pol1.STIntersection(pol2);

            Console.WriteLine("GEOMETRYCOLLECTION(");
            Console.WriteLine(pol1.ToString());
            Console.WriteLine(",");
            Console.WriteLine(res.ToString());
            Console.WriteLine(",");
            Console.WriteLine(pol2.ToString());
            Console.WriteLine(")");
        }

        static void testddl()
        {
            string pol1wkt = "POLYGON((-82.97767639160156 47.63393279834071,-82.39059448242189 47.630693753685705,-82.4029541015625 47.68110753158709,-82.95913696289062 47.674635091761616,-82.97767639160156 47.63393279834071))";
            string pol2wkt = "POLYGON((-82.51899719238283 47.5913464767971,-82.5286102294922 47.74717289953017,-82.91038513183594 47.74347914666066,-82.90695190429688 47.57976811421673,-82.83210754394531 47.5857891823799,-82.84446716308595 47.70883746550024,-82.59040832519533 47.7134576874889,-82.57118225097656 47.598755284818026,-82.51899719238283 47.5913464767971))";

            SqlGeography pol1 = SqlGeography.STGeomFromText(new SqlChars(pol1wkt), 4326);
            SqlGeography pol2 = SqlGeography.STGeomFromText(new SqlChars(pol2wkt), 4326);

            SqlGeography res = pol1.STIntersection(pol2);
            //res.STGeometryN(1)    
            //res.STNumGeometries()    
            Console.WriteLine("GEOMETRYCOLLECTION(");
            Console.WriteLine(pol1.ToString());
            Console.WriteLine(",");
            Console.WriteLine(res.ToString());
            Console.WriteLine(",");
            Console.WriteLine(pol2.ToString());
            Console.WriteLine(")");
        }


        static void fixPolygons()
        {
            DateTime dt1 = DateTime.Parse("2019-02-01T00:00:00");
            DateTime dt2 = DateTime.Parse("2019-02-02T00:20:00");

            string cupConnStr = System.IO.File.ReadLines("DBstring.conf").First();
            string cuksConnStr = System.IO.File.ReadLines("DBstringCUKS.conf").First();
            DIOS.Common.SqlManager CUKSmanagerDB = new DIOS.Common.SqlManager(cuksConnStr);
            DIOS.Common.SqlManager CUPmanagerDB = new DIOS.Common.SqlManager(cupConnStr);

            var trajectory = new DataFetcher(CUPmanagerDB).GetTrajectorySat(dt1, dt2);

            SatLane viewLane = new SatLane(trajectory, 0, OptimalChain.Constants.camera_angle*10);
            //Console.WriteLine(viewLane.Sectors.First().polygon.ToWtk());
            //return;
            Polygon reqPol = new Polygon("POLYGON((-30.14648437500001 -51.39920565355377,-14.853515625000012 -48.16608541901252,-17.314453125000014 -44.59046718130883,-31.46484375000001 -48.516604348867475,-30.14648437500001 -51.39920565355377))");


            var request = new RequestParams(0, 1, dt1, dt2,
                _Max_SOEN_anlge: 0.87266462599716477,
                _minCoverPerc: 11,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: "POLYGON((-30.14648437500001 -51.39920565355377,-14.853515625000012 -48.16608541901252,-17.314453125000014 -44.59046718130883,-31.46484375000001 -48.516604348867475,-30.14648437500001 -51.39920565355377))",
                _polygonToSubtract: new List<string>(),
                _requestChannel: 0,
                _shootingType: ShootingType.Normal,
                _compression: 10,
                _albedo: 0
                );                        

            List<CaptureConf> confs = viewLane.getCaptureConfs(request);
                                    
            List<Polygon> pols = confs.Select(cc => viewLane.getSegment(confs.First().dateFrom, confs.First().dateTo) ).ToList();
            Console.WriteLine(Polygon.getMultipolFromPolygons(pols));
        }

        static void Main(string[] args)
        {
            DateTime start = DateTime.Now;

            // fixPolygons();
            test_getCoridorMpzArray();
            //test_Polygons();
            
            DateTime end = DateTime.Now;
            Console.WriteLine("Время выполнения : {0} ", (end - start).ToString());
            Console.ReadKey();
        }
    }
}