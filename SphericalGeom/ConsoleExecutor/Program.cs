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
                Console.WriteLine(Polygon.getMultipolFromPolygons(tuple.Item2.Select(wpl => new Polygon(wpl.wktPolygon)).ToList()));
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
                _polygonsToSubtract: new List<string>(),
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
                _polygonsToSubtract: new List<string>(),
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
                _polygonsToSubtract: new List<string>(),
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
                _polygonsToSubtract: new List<string>(),
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
                _polygonsToSubtract: new List<string>(),
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
                _polygonsToSubtract: new List<string>(),
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
                _polygonsToSubtract: new List<string>(),
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
                _polygonsToSubtract: new List<string>(),
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
                _polygonsToSubtract: new List<string>(),
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
                _polygonsToSubtract: new List<string>(),
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
                _polygonsToSubtract: new List<string>(),
                _requestChannel: 0,
                _shootingType: ShootingType.Normal,
                _compression: 1,
                _albedo: 0
                );
            #endregion


            DateTime dt1 = DateTime.Parse("2019-02-01T10:00:00");// new DateTime(2019, 2, 18, 2, 0, 0);
            DateTime dt2 = DateTime.Parse("2019-02-01T20:00:00");// new DateTime(2019, 2, 18, 3, 0, 0);

            List<RequestParams> reqlist = new List<RequestParams> { req0, req1, req2, req3, req4, req5, req6, req7, req8, req9, req10 };


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
                "POLYGON((-73.75946044921875 40.963307953073524,-73.88305664062499 40.95915977213494,-74.0478515625 40.932190241465634,-74.18792724609374 40.88652440805993,-74.35272216796874 40.815887914415896,-74.4378662109375 40.72852712420601,-74.49554443359374 40.61186744303009,-74.51202392578124 40.472024396920574,-74.46533203124999 40.42604212826495,-74.42687988281249 40.45948689837198,-74.45983886718749 40.53467678061543,-74.4378662109375 40.60978237983301,-74.40765380859375 40.701463603604594,-74.33349609375 40.77638178482897,-74.22363281249999 40.82628035667713,-74.06433105468749 40.867833841384936,-73.95996093749999 40.903133814658474,-73.80615234374999 40.91766362458114,-73.67980957031249 40.95708558389899,-73.75946044921875 40.963307953073524))"
            ,"POLYGON((149.51293945312497 -30.090484220005344,149.5404052734375 -30.033433263128984,149.56787109375 -29.95969381418451,149.60906982421875 -29.893043385434176,149.65301513671875 -29.850173125689892,149.688720703125 -29.81681685764992,149.68597412109372 -29.80251790576446,149.64202880859372 -29.833496383743203,149.59945678710935 -29.872801391992795,149.55825805664062 -29.926374178635747,149.5184326171875 -29.99538103356813,149.4978332519531 -30.041755241903317,149.48959350585935 -30.073847754270204,149.51293945312497 -30.090484220005344))"
            ,"POLYGON((-54.64599609374999 -25.32416652573839,-56.260986328124986 -26.961245770526972,-56.326904296875 -27.186242185608734,-56.04125976562499 -27.196014383173292,-55.118408203124986 -26.155437968713542,-53.98681640625 -25.025884063244817,-53.3056640625 -24.55711616430962,-51.95434570312499 -24.116674961751286,-50.614013671875 -23.815500848699656,-50.526123046875 -23.50355189742413,-51.05346679687501 -23.51362636346272,-51.94335937499999 -23.76523688975867,-52.80029296874999 -24.05649649076851,-53.76708984374999 -24.467150664738995,-54.3603515625 -24.886436490787702,-54.64599609374999 -25.32416652573839))"
            ,"POLYGON((-63.687744140624986 75.16330024622056,-63.06152343749998 75.11822201684026,-62.21557617187499 75.14641157325434,-61.380615234374986 75.21946035674114,-60.611572265624986 75.33950166120164,-60.35888671874999 75.49990834816703,-60.02929687499998 75.89344704833564,-59.732666015624986 76.02140235430915,-57.304687499999986 76.10079606754579,-56.44775390624999 76.06909176390045,-55.66772460937499 76.00281329293702,-55.129394531249986 75.89612445246954,-54.68994140624999 75.7940323033921,-54.26147460937499 75.79133591604713,-54.33837890624999 75.86127933376639,-54.61303710937499 75.93355587409607,-55.01953124999999 76.01609366420993,-55.612792968749986 76.08759454588619,-56.096191406249986 76.12716228011627,-56.67846679687499 76.15084994303763,-57.183837890625 76.15873701495826,-57.81005859374999 76.15347945648796,-58.61206054687499 76.12716228011627,-59.23828124999999 76.10079606754579,-59.67773437499999 76.07438073300233,-60.12817382812499 76.05586072525739,-60.46875 75.98686053114261,-60.64453124999999 75.85054080189764,-60.71044921875 75.73188842222964,-60.666503906249986 75.59860599198842,-60.897216796875 75.40885422846455,-61.10595703125 75.36172949075694,-61.49047851562499 75.29215785826014,-62.34741210937499 75.21385371117896,-62.98461914062499 75.16048677152295,-63.446044921875 75.18578927942625,-63.90747070312499 75.25585291932208,-64.171142578125 75.23346787838844,-63.687744140624986 75.16330024622056))"
            ,"POLYGON((143.49517822265625 73.23620856434462,143.26995849609378 73.22907740190121,142.97882080078125 73.21004656128417,142.5421142578125 73.1639687361907,142.2784423828125 73.14088379815641,141.95983886718744 73.07464313148589,140.90515136718747 72.698021973724,141.17431640625 72.67268149675314,141.43798828125 72.76406472320437,141.83349609375 72.89380193362285,142.53662109375 73.02900629225599,143.4375 73.15680773175978,144.2724609375 73.18861179538933,144.8876953125 73.25837557295395,145.6787109375 73.37192828625788,146.09619140625 73.52839948765174,146.46972656250003 73.73890457673937,146.57958984375 74.04976184989872,146.35986328125 74.22393470166762,145.8544921875 74.22990748716416,145.458984375 74.14008387440461,145.74462890625003 74.07389660001783,146.0302734375 74.08595063145893,146.27197265625 73.97107763933991,146.14013671875003 73.81257376708521,145.89843750000003 73.64017137216683,145.78857421875 73.5221682137352,145.43701171875 73.42215560370377,144.90966796875 73.33416078093367,144.44824218750003 73.26470377263848,143.83300781250003 73.21401304858364,143.49517822265625 73.23620856434462))"
                };

            List<string> holes = new List<string>();

            List<RequestParams> reqlist = wktList.Select(polwtk =>
             new RequestParams(0, 1, dt1, dt2,
                _Max_SOEN_anlge: 0.87266462599716477,
                _minCoverPerc: 11,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: polwtk,
                _polygonsToSubtract: holes,
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
            DateTime dt1 = DateTime.Parse("2019-02-01T00:00:00");// new DateTime(2019, 2, 18, 2, 0, 0);
            DateTime dt2 = DateTime.Parse("2019-02-04T00:00:00");// new DateTime(2019, 2, 18, 3, 0, 0);

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
                _polygonsToSubtract: holes,
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

        static public void test_getCaptureConf()
        {


            List<string> wktList = new List<string>(){
"POLYGON ((64.74111258378889 41.686629965884222, 61.857845990629286 39.304004006224282, 60.902899565200954 35.251934424349443, 60.900118751046904 35.050993339098795, 65.088523000477 28.301561852323381, 66.099890808811438 28.545589414943674, 67.420766764267043 29.592509191345822, 64.74111258378889 41.686629965884222))"
,"POLYGON ((-153.92844837404516 1.9048881535635396, -153.5431969809294 -2.45077514379084, -153.18919369500043 -2.779550197270928, -149.930229854616 1.6199048231448909, -150.89172997870045 2.83838375289371, -151.88004999375036 3.1960008332638923, -153.92844837404516 1.9048881535635396))"
,"POLYGON ((-139.82036075219855 -20.270070044153243, -138.73459004325184 -20.734835631705998, -136.51913730461442 -20.980038994296279, -133.70966606739063 -19.980321211411336, -135.20015404498369 -9.285998580685721, -136.23515855854379 -9.0506286377112737, -139.82036075219855 -20.270070044153243))"
,"POLYGON ((142.0097278062056 49.800484753257628, 142.10205417785818 49.356767611286848, 150.98282736118043 48.065432732614624, 151.97021601099038 49.651242392832415, 151.99762915302739 50.098532669063893, 147.04999916667083 53.199840001333328, 142.0097278062056 49.800484753257628))"
,"POLYGON ((163.22180676589233 69.45901519692184, 161.99901385764358 69.219355672528238, 159.7554360446581 67.940310176627079, 159.39096924111345 64.501651114678623, 167.59249490542791 63.515726052732582, 168.49798481511323 67.6496492746081, 166.92162404651245 68.8687680624592, 163.22180676589233 69.45901519692184))"
,"POLYGON ((-66.940970519698979 62.044342511338691, -71.244871037629281 58.577591704463458, -71.0677248626119 58.139094070024328, -58.998769976088951 58.026239452608536, -58.91045533145023 58.179805093671938, -58.837361377109325 58.335420794851892, -61.600318092975996 61.694083120519259, -66.940970519698979 62.044342511338691))"
,"POLYGON ((-125.863468342602 69.990092924681676, -115.57486182314516 64.559647846934453, -116.92616203377942 70.547731668986685, -125.863468342602 69.990092924681676))"
,"POLYGON ((132.53114790668164 -72.4000839305366, 127.90547949770817 -79.288158753348853, 129.18999594800022 -83.182016160373891, 136.60701726942637 -85.147690565716033, 138.50793219616278 -83.581157522043, 132.53114790668164 -72.4000839305366))"
,"POLYGON ((61.802855940862081 38.941816616617807, 63.603376695409771 30.101247781726709, 66.440250119332717 30.134497194567881, 68.3307614748374 33.571508705355811, 68.565176494962174 35.111970976304782, 66.23443210683962 42.011985362223228, 61.802855940862081 38.941816616617807))"
,"POLYGON ((0.84394125461876812 -39.354318352840458, 0.13722307856255966 -43.841143952012914, 0.77325365335004392 -46.479226519442349, 1.2063204820608926 -47.372333802304439, 11.875696982772077 -42.31996281651768, 11.863823344128704 -42.170726651349476, 11.715419090697763 -41.1386841126072, 0.84394125461876812 -39.354318352840458))"
            };

            List<Polygon> polygons = wktList.Select(wkt => new Polygon(wkt)).ToList();

            string cuksConnStr = System.IO.File.ReadLines("DBstringCUKS.conf").First();
            DIOS.Common.SqlManager CUKSmanager = new DIOS.Common.SqlManager(cuksConnStr);
            string cupConnStr = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerCUP = new DIOS.Common.SqlManager(cupConnStr);

            DateTime dt1 = new DateTime(2019, 1, 4);
            DateTime dt2 = new DateTime(2019, 1, 8);

            var inactivityRanges = new List<TimePeriod>();
            inactivityRanges.Add(new TimePeriod(new DateTime(2019, 1, 5), new DateTime(2019, 1, 6)));


            DataFetcher fetcher = new DataFetcher(managerCUP);
            Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

            if (trajectory.Count == 0)
                throw new Exception("На эти даты нет траектории в БД, тест некорректный");

            int id = 0;
            List<RequestParams> requests = new List<RequestParams>();
            foreach (var pol in polygons)
            {
                RequestParams reqparams = new RequestParams(id, 1, dt1, dt2, AstronomyMath.ToRad(45), 0.4, 1, 1, pol.ToWtk());
                requests.Add(reqparams);
                id++;
            }
            var res = Sessions.getCaptureConfArray(requests, dt1, dt2, managerCUP, CUKSmanager, inactivityRanges, new List<TimePeriod>());

        }


        static public void Test_getMPZArray()
        {


            List<string> wktList = new List<string>(){
                  
"POLYGON ((-146.939928441149 54.492209161446738, -143.83257766050622 54.100462783025712, -142.79541654172559 54.693254767286064, -141.76525923979946 56.386544674711764, -141.83854766104557 57.888817149859065, -146.939928441149 54.492209161446738))"
,"POLYGON ((36.178666303411319 -76.0852505171355, 35.171879068984182 -77.148286326049643, 37.890749289083743 -79.880223042262969, 38.363277520046665 -76.463453375364992, 37.318636413228489 -75.926822704911189, 36.178666303411319 -76.0852505171355))"
,"POLYGON ((44.334092568504772 28.448571960257429, 43.584896602599791 28.2584758189864, 41.731196431309847 26.342942780252269, 47.776854249866055 27.35075576467035, 44.334092568504772 28.448571960257429))"
,"POLYGON ((-135.27978653791976 32.439516857250943, -136.24856068007875 32.064795352369394, -131.64027060210233 27.997398739477003, -135.27978653791976 32.439516857250943))"
,"POLYGON ((-91.180152176982986 -24.788325330136171, -89.089856047868466 -18.883314322288374, -89.272055068177323 -18.492064045637068, -90.083198545635824 -17.5149408732969, -91.180152176982986 -24.788325330136171))"
,"POLYGON ((98.26963258591438 -34.584481476384205, 100.92089255491653 -32.852431988870563, 101.19906192669131 -31.1109581847449, 100.94030717514688 -29.208952216139789, 98.26963258591438 -34.584481476384205))"
,"POLYGON ((-70.828582624243538 -6.8601855521392325, -66.947010443107914 -9.6338259176254333, -66.760598347321888 -9.5258813538665379, -65.328024662044953 -7.5158705381225328, -70.828582624243538 -6.8601855521392325))"
,"POLYGON ((-86.662555278771961 78.673610862197563, -85.800169232639334 77.496619672013708, -82.192550394600786 81.244564133460415, -86.662555278771961 78.673610862197563))"
,"POLYGON ((179.76052631281007 -60.511390998159264, -177.663888813167 -60.3811563692531, -177.66626072999262 -55.615441912496408, 179.76052631281007 -60.511390998159264))"
,"POLYGON ((-110.84899593896716 14.740455997873337, -110.33522488008843 14.450929323547671, -109.04114892279159 21.159297222805339, -110.84899593896716 14.740455997873337))"
,"POLYGON ((-12.643912243719926 -87.404309410527276, -12.791317933131394 -87.858125007491211, -12.776025813807165 -88.2355203065637, -11.721733162671088 -89.46548148250757, -5.6308177250919496 -87.028780729561959, -12.643912243719926 -87.404309410527276))"
,"POLYGON ((109.10361472365378 -54.61570727751711, 109.89946528388944 -52.072969473026426, 109.76106199061249 -50.8378957481625, 108.73186677995093 -48.9520361200397, 107.40467503246829 -48.237179214391986, 109.10361472365378 -54.61570727751711))"
,"POLYGON ((-129.24731534558526 39.268480095621712, -129.17936940691817 34.681978628896324, -126.43223085194634 38.975559945959318, -126.7203696473006 39.243539833570438, -129.24731534558526 39.268480095621712))"
,"POLYGON ((72.07313019562028 -13.525734978649549, 71.720126446579783 -15.356406999358962, 72.172337775557992 -16.639133564955184, 73.479218999920064 -17.629876272836906, 75.697242153552921 -13.177835447807578, 72.07313019562028 -13.525734978649549))"
,"POLYGON ((-47.15941124719113 -6.9955128448428239, -48.452797732892 -8.6702677653994176, -43.760543844423154 -13.486222660474034, -47.15941124719113 -6.9955128448428239))"
,"POLYGON ((-26.486135087492009 -64.579898427278877, -26.786597763783497 -64.824355756166852, -27.374763173534468 -65.5290916916911, -20.406020802091756 -65.960651889795926, -21.43770593581305 -64.636900612065318, -26.486135087492009 -64.579898427278877))"
,"POLYGON ((109.5031976733283 4.99574761024498, 109.00046520999399 3.0633963934523063, 113.59355859376265 6.0501183996482979, 109.5031976733283 4.99574761024498))"
,"POLYGON ((-83.48429631381471 23.134172109129512, -84.13624159299664 22.818653750315548, -84.6589442518331 22.312441314785893, -80.686440809234128 19.148915598051271, -83.48429631381471 23.134172109129512))"
,"POLYGON ((-77.8881338999701 62.869539952090705, -79.456131531560828 61.373024738845153, -79.300320134389054 60.216764748771133, -79.001666955439546 59.80178014718463, -77.8881338999701 62.869539952090705))"
,"POLYGON ((79.508161238465334 2.2615174084179444, 78.627646490752 1.9807425594714978, 77.303059614375883 0.10946397851691196, 78.202658640375475 -1.7163417629985467, 81.321829838431839 -2.0055215788594083, 79.508161238465334 2.2615174084179444))"
             };

            List<Polygon> polygons = wktList.Select(wkt => new Polygon(wkt)).ToList();

            string cuksConnStr = System.IO.File.ReadLines("DBstringCUKS.conf").First();
            DIOS.Common.SqlManager managerCUKS = new DIOS.Common.SqlManager(cuksConnStr);

            string cupConnStr = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerCUP = new DIOS.Common.SqlManager(cupConnStr);

            DateTime dt1 = new DateTime(2019, 1, 4);
            DateTime dt2 = new DateTime(2019, 1, 4, 20, 0, 0);

            DataFetcher fetcher = new DataFetcher(managerCUP);
            Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

            if (trajectory.Count == 0)
                throw new Exception("На эти даты нет траектории в БД, тест некорректный");


            int id = 0;
            List<RequestParams> requests = new List<RequestParams>();
            foreach (var pol in polygons)
            {
                RequestParams reqparams = new RequestParams(id, 1, dt1, dt2, AstronomyMath.ToRad(45), 0.4, 1, 1, pol.ToWtk());
                requests.Add(reqparams);
                id++;
            }
            //  var res = Sessions.getCaptureConfArray(requests, dt1, dt2, manager, new List<Tuple<DateTime, DateTime>>());

            Order order = new Order();
            order.captured = new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))");
            order.intersection_coeff = 0.1;
            order.request = new RequestParams(id, 1, dt1, dt2, AstronomyMath.ToRad(45), 0.4, 1, 1, "POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))");

            List<Order> orders = new List<Order>() { order };

            CaptureConf ccToDrop = new CaptureConf(new DateTime(2019, 1, 4), new DateTime(2019, 1, 5), 0.1, orders, WorkingType.Downloading, null);
            StaticConf sc = ccToDrop.DefaultStaticConf();
            RouteParams routeParamtoDrop = new RouteParams(sc);
            routeParamtoDrop.NRoute = 10;
            routeParamtoDrop.NPZ = 10;
            routeParamtoDrop.start = new DateTime(2019, 1, 4);
            routeParamtoDrop.end = new DateTime(2019, 1, 5);
            //routeParamtoDrop.File_Size = 1000;
            routeParamtoDrop.binded_route = null;
            // double timedrop = routeParam.getDropTime();

            RouteMPZ routempzToDrop = new RouteMPZ(routeParamtoDrop, managerCUP) { NPZ = 0, Nroute = 0 };

            List<RouteMPZ> routesToDrop = new List<RouteMPZ>();
            routesToDrop.Add(routempzToDrop);


            CaptureConf ccToDelete = new CaptureConf(new DateTime(2019, 1, 4), new DateTime(2019, 1, 5), 0.1, orders, WorkingType.Removal, null);
            StaticConf scToDelete = ccToDelete.DefaultStaticConf();
            RouteParams routeParamtoDelete = new RouteParams(scToDelete);
            routeParamtoDelete.NRoute = 0;
            routeParamtoDelete.start = new DateTime(2019, 1, 4);
            routeParamtoDelete.end = new DateTime(2019, 1, 5);
            //routeParamtoDelete.File_Size = 1000;
            routeParamtoDelete.binded_route = routeParamtoDrop;
            RouteMPZ routempzToDelete = new RouteMPZ(routeParamtoDelete, managerCUP) { NPZ = 0, Nroute = 0 };

            List<RouteMPZ> routesToDelete = new List<RouteMPZ>();
            routesToDelete.Add(routempzToDelete);

            List<Tuple<DateTime, DateTime>> silenceRanges = new List<Tuple<DateTime, DateTime>>();
            // silenceRanges.Add(Tuple.Create(new DateTime(2019, 1, 6), new DateTime(2019, 1, 9)));
            //silenceRanges.Add(Tuple.Create(new DateTime(2019, 1, 6), new DateTime(2019, 1, 6)));


            var inactivityRanges = new List<Tuple<DateTime, DateTime>>();
            // inactivityRanges.Add(Tuple.Create(new DateTime(2019, 1, 5), new DateTime(2019, 1, 6))); 


            List<MPZ> mpzArray;
            List<CommunicationSession> sessions;
            var enabled = new List<SessionsPlanning.CommunicationSessionStation>  
                    {                 
                SessionsPlanning.CommunicationSessionStation.FIGS_Main,
                SessionsPlanning.CommunicationSessionStation.FIGS_Backup,
                SessionsPlanning.CommunicationSessionStation.MIGS };


            Sessions.getMPZArray(requests
                , dt1, dt2
                , silenceRanges
                , inactivityRanges
                , routesToDrop
                , routesToDelete
                , cupConnStr
                , cuksConnStr
                , 0
                , out mpzArray
                , out sessions
                , enabled);
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
            List<string> str = new List<string>(){             
"POLYGON  ((-140.90240551518033 -62.596681146184444, -142.28795449997176 -66.378177647253736, -140.06532810953229 -69.698507192331917, -140.90240551518033 -62.596681146184444))"
,"POLYGON  ((11.132990788861918 -64.510946757679861, 9.4079630505447742 -65.69097625435451, 11.526762917570258 -69.567447401763047, 11.132990788861918 -64.510946757679861))"
,"POLYGON  ((-113.59018450062918 24.055244508520257, -112.32935234166799 18.816300154247195, -108.6351831102097 19.642051275419036, -108.06623005193035 23.397688352921961, -113.59018450062918 24.055244508520257))"
,"POLYGON  ((6.8977473068190269 84.900847269217024, 10.448857097041209 87.602101940227186, 8.9426753754422883 88.767992047840877, 6.8977473068190269 84.900847269217024))"
,"POLYGON  ((123.55765361761642 -68.816670984569541, 127.17774646293752 -72.033830850200729, 127.42927429937774 -71.669460376331628, 124.03599940000302 -68.800109999083332, 123.55765361761642 -68.816670984569541))"
,"POLYGON  ((-177.98859846743579 -62.103301410765447, -174.78714353180848 -67.438856111299671, -173.29225124025871 -64.342979938720873, -177.98859846743579 -62.103301410765447))"
,"POLYGON  ((-85.597772421005658 -9.4179242876637073, -84.137655966004374 -11.472201371937871, -80.20338871828254 -8.1868985232738485, -85.597772421005658 -9.4179242876637073))"
,"POLYGON  ((-146.7338204131002 58.318335594188646, -148.14509678131122 56.180852381294258, -143.80177050422626 51.36617408237457, -146.7338204131002 58.318335594188646))"
,"POLYGON  ((-129.62739925769034 -31.6036756729262, -129.28857716996811 -34.626991363270371, -126.25273571843462 -35.14620702467839, -125.70235262702207 -35.033465232991418, -125.3106075313313 -31.090797804709172, -129.62739925769034 -31.6036756729262))"
,"POLYGON  ((121.90905684397012 -11.805211161958368, 121.4758457632942 -12.57171662325081, 121.20106724796028 -13.908895212201102, 122.96556531607547 -17.066538877635608, 125.27205587552486 -16.939791360375906, 121.90905684397012 -11.805211161958368))"
   };

            List<Polygon> polygons = str.Select(s => new Polygon(s)).ToList();
             


            string cuksConnStr = System.IO.File.ReadLines("DBstringCUKS.conf").First();
            DIOS.Common.SqlManager managerCUKS = new DIOS.Common.SqlManager(cuksConnStr);

            string cupConnStr = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerCUP = new DIOS.Common.SqlManager(cupConnStr);

            DateTime dt1 = new DateTime(2019, 1, 4);
            DateTime dt2 = new DateTime(2019, 1, 8);

            DataFetcher fetcher = new DataFetcher(managerCUP);
            Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

            if (trajectory.Count == 0)
                throw new Exception("На эти даты нет траектории в БД, тест некорректный");

            int id = 0;
            List<RequestParams> requests = new List<RequestParams>();
            foreach (var pol in polygons)
            {
                RequestParams reqparams = new RequestParams(id, 1, dt1, dt2, AstronomyMath.ToRad(45), 0.4, 1, 1, pol.ToWtk());
                requests.Add(reqparams);
                id++;
            }
            //  var res = Sessions.getCaptureConfArray(requests, dt1, dt2, manager, new List<Tuple<DateTime, DateTime>>());

            Order order = new Order();
            order.captured = new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))");
            order.intersection_coeff = 0.1;
            order.request = new RequestParams(id, 1, dt1, dt2, AstronomyMath.ToRad(45), 0.4, 1, 1, "POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))");

            List<Order> orders = new List<Order>() { order };

            CaptureConf ccToDrop = new CaptureConf(new DateTime(2019, 1, 4), new DateTime(2019, 1, 5), 0.1, orders, WorkingType.Downloading, null);
            StaticConf sc = ccToDrop.DefaultStaticConf();
            RouteParams routeParamtoDrop = new RouteParams(sc);
            routeParamtoDrop.NRoute = 10;
            routeParamtoDrop.NPZ = 10;
            routeParamtoDrop.start = new DateTime(2019, 1, 4);
            routeParamtoDrop.end = new DateTime(2019, 1, 5);
            //routeParamtoDrop.File_Size = 1000;
            routeParamtoDrop.binded_route = null;
            // double timedrop = routeParam.getDropTime();

            RouteMPZ routempzToDrop = new RouteMPZ(routeParamtoDrop, managerCUP) { NPZ = 0, Nroute = 0 };

            List<RouteMPZ> routesToDrop = new List<RouteMPZ>();
            routesToDrop.Add(routempzToDrop);


            CaptureConf ccToDelete = new CaptureConf(new DateTime(2019, 1, 4), new DateTime(2019, 1, 5), 0.1, orders, WorkingType.Removal, null);
            StaticConf scToDelete = ccToDelete.DefaultStaticConf();
            RouteParams routeParamtoDelete = new RouteParams(scToDelete);
            routeParamtoDelete.NRoute = 0;
            routeParamtoDelete.start = new DateTime(2019, 1, 4);
            routeParamtoDelete.end = new DateTime(2019, 1, 5);
            //routeParamtoDelete.File_Size = 1000;
            routeParamtoDelete.binded_route = routeParamtoDrop;
            RouteMPZ routempzToDelete = new RouteMPZ(routeParamtoDelete, managerCUP) { NPZ = 0, Nroute = 0 };

            List<RouteMPZ> routesToDelete = new List<RouteMPZ>();
            routesToDelete.Add(routempzToDelete);

            List<Tuple<DateTime, DateTime>> silenceRanges = new List<Tuple<DateTime, DateTime>>();
            // silenceRanges.Add(Tuple.Create(new DateTime(2019, 1, 6), new DateTime(2019, 1, 9)));
            //silenceRanges.Add(Tuple.Create(new DateTime(2019, 1, 6), new DateTime(2019, 1, 6)));


            var inactivityRanges = new List<Tuple<DateTime, DateTime>>();
            // inactivityRanges.Add(Tuple.Create(new DateTime(2019, 1, 5), new DateTime(2019, 1, 6))); 


            List<MPZ> mpzArray;
            List<CommunicationSession> sessions;
            var enabled = new List<SessionsPlanning.CommunicationSessionStation>  
            { SessionsPlanning.CommunicationSessionStation.FIGS_Main,
                SessionsPlanning.CommunicationSessionStation.FIGS_Backup,
                SessionsPlanning.CommunicationSessionStation.MIGS };
            Sessions.getMPZArray(requests, dt1, dt2
                                                , silenceRanges
                                                , inactivityRanges
                                                 , routesToDrop
                                                 , routesToDelete
                                                  , cupConnStr
                                                  , cuksConnStr
                                                  , 0
                                                 , out mpzArray
                                                 , out sessions
                                                 , enabled);

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

            SatLane viewLane = new SatLane(trajectory, 0, OptimalChain.Constants.camera_angle * 10);
            //Console.WriteLine(viewLane.Sectors.First().polygon.ToWtk());
            //return;
            Polygon reqPol = new Polygon("POLYGON((-30.14648437500001 -51.39920565355377,-14.853515625000012 -48.16608541901252,-17.314453125000014 -44.59046718130883,-31.46484375000001 -48.516604348867475,-30.14648437500001 -51.39920565355377))");


            var request = new RequestParams(0, 1, dt1, dt2,
                _Max_SOEN_anlge: 0.87266462599716477,
                _minCoverPerc: 11,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: "POLYGON((-30.14648437500001 -51.39920565355377,-14.853515625000012 -48.16608541901252,-17.314453125000014 -44.59046718130883,-31.46484375000001 -48.516604348867475,-30.14648437500001 -51.39920565355377))",
                _polygonsToSubtract: new List<string>(),
                _requestChannel: 0,
                _shootingType: ShootingType.Normal,
                _compression: 10,
                _albedo: 0
                );

            List<CaptureConf> confs = viewLane.getCaptureConfs(request);

            List<Polygon> pols = confs.Select(cc => viewLane.getSegment(confs.First().dateFrom, confs.First().dateTo)).ToList();
            Console.WriteLine(Polygon.getMultipolFromPolygons(pols));
        }



        static void testGetRollPitch()
        {
            DateTime dt2 = DateTime.Parse("2019-02-02T01:30:00");

            string cupConnStr = System.IO.File.ReadLines("DBstring.conf").First();
            string cuksConnStr = System.IO.File.ReadLines("DBstringCUKS.conf").First();
            DIOS.Common.SqlManager CUKSmanagerDB = new DIOS.Common.SqlManager(cuksConnStr);
            DIOS.Common.SqlManager CUPmanagerDB = new DIOS.Common.SqlManager(cupConnStr);

            DataFetcher fetcher = new DataFetcher(CUPmanagerDB);

            TrajectoryPoint trajPoint = fetcher.GetSingleSatPoint(dt2).Value;

            GeoPoint goal = new GeoPoint(28.26568239014648, 130.25390624999997); // 130.25390624999997 28.26568239014648

            double roll, pitch;
            Routines.GetRollPitch(trajPoint, goal, out roll, out pitch);

            SatelliteCoordinates sat = new SatelliteCoordinates(trajPoint);
            sat.addRollPitchRot(roll, pitch);
            //, roll, pitch);

            Console.WriteLine("GEOMETRYCOLLECTION(");

            //Console.WriteLine("trajPpoint = ");
            Console.WriteLine(GeoPoint.FromCartesian(trajPoint.Position.ToVector()).ToWkt());
            Console.WriteLine();

            Console.WriteLine(",");

            //Console.WriteLine("goal = ");
            Console.WriteLine(goal.ToWkt());

            Console.WriteLine(",");
            //Console.WriteLine("goal = ");
            Console.WriteLine(sat.ViewPolygon.ToWtk());

            //Console.WriteLine();
            Console.WriteLine(")");

        }


        static void testBooblick()
        {
            DateTime dt1 = DateTime.Parse("2019-02-01T00:00:00+03:00");// new DateTime(2019, 2, 18, 2, 0, 0);
            DateTime dt2 = DateTime.Parse("2019-02-3T00:00:00");// new DateTime(2019, 2, 18, 3, 0, 0);

            string cupConnStr = System.IO.File.ReadLines("DBstring.conf").First();
            string cuksConnStr = System.IO.File.ReadLines("DBstringCUKS.conf").First();
            DIOS.Common.SqlManager CUKSmanagerDB = new DIOS.Common.SqlManager(cuksConnStr);
            DIOS.Common.SqlManager CUPmanagerDB = new DIOS.Common.SqlManager(cupConnStr);

            List<string> wktList = new List<string>(){
             "POLYGON((120.23025512695312 23.548880923858746,120.41839599609375 23.560210740100587,120.52963256835939 23.605520232835957,120.63125610351565 23.74889691939815,120.63262939453126 23.8330886396801,120.60791015625003 23.89713827430829,120.55847167968753 23.93103407144254,120.50216674804688 23.947351097609143,120.44448852539062 23.959901252071035,120.377197265625 23.956136333969283,120.26596069335935 23.949861226199317,120.19317626953124 23.934799722118214,120.1025390625 23.89964937898631,120.05859375 23.7916278920882,120.03387451171874 23.74889691939815,120.03799438476562 23.70112217564032,120.10528564453124 23.652072011367167,120.15747070312499 23.67345511584803,120.17669677734374 23.711181471130104,120.21377563476562 23.753924821039774,120.2728271484375 23.7916278920882,120.32226562499999 23.79916719450499,120.38269042968749 23.79916719450499,120.41564941406249 23.774034485612958,120.44586181640624 23.719982718449657,120.421142578125 23.670939638014275,120.38406372070312 23.635717866758824,120.32501220703122 23.619361679019548,120.27145385742188 23.614328594991676,120.21377563476562 23.61181198048864,120.19866943359374 23.576574305545677,120.20416259765624 23.55391651832163,120.23025512695312 23.548880923858746))"
            };

            List<string> holes = new List<string>(){
              "POLYGON((120.49530029296875 23.4191480235653,120.52001953125 23.429228934100195,120.52139282226562 23.45568766694339,120.50491333007811 23.521181707248573,120.45272827148439 23.590418806655435,120.39916992187499 23.663392914050704,120.35385131835938 23.71369617382328,120.25360107421875 23.804193153061718,120.22613525390625 23.854442037096888,120.19866943359376 23.89588270368263,120.13687133789062 23.963666060236662,120.07919311523438 24.011344109236347,120.00228881835939 23.99628979062841,119.981689453125 23.963666060236662,120.03387451171876 23.943585813147905,120.06408691406251 23.936054914599836,120.10528564453124 23.910948746482546,120.15747070312501 23.868257066593785,120.20278930664062 23.81424448670424,120.23574829101562 23.753924821039774,120.29067993164062 23.693577195179472,120.33737182617188 23.631943543199654,120.40603637695311 23.56524590044569,120.43487548828124 23.523700058824147,120.46234130859375 23.464506066910445,120.49530029296875 23.4191480235653))"
            };

            List<RequestParams> reqlist = wktList.Select(polwtk =>
             new RequestParams(0, 1, dt1, dt2,
                _Max_SOEN_anlge: 0.1,//0.87266462599716477,
                _minCoverPerc: 11,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: polwtk,
                _polygonsToSubtract: holes,
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
            Console.Write(Polygon.getMultipolFromPolygons(reqlist.SelectMany(r => r.polygons).ToList()));
            Console.Write(",");
            // Console.WriteLine(WktTestingTools.getWKTStrip(dt1, dt2));
            Console.Write(Polygon.getMultipolFromPolygons(shootingPolygons));
            Console.Write(")");

        }

        static void Main(string[] args)
        {
            DateTime start = DateTime.Now;

            // fixPolygons();
            testddl();
            //test_Polygons();

            DateTime end = DateTime.Now;
            Console.WriteLine("Время выполнения : {0} ", (end - start).ToString());
            Console.ReadKey();
        }
    }
}