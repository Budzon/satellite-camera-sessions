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
         
        static public void test_getPlainMpzArray()
        {
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
                _albedo : 0
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
            , out sessions);

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

            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get14PlainFrames1Turn(fromDt, managerDB, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get45PlainFrames4Turn(fromDt, managerDB, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get8StereoTriplets1Turn(fromDt, managerDB, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get8StereoPairs1Turn(fromDt, managerDB, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get20StereoTriplets5Turn(fromDt, managerDB, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get20StereoPairs5Turn(fromDt, managerDB, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get8Coridors1Turn(fromDt, managerDB, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get20Coridors5Turn(fromDt, managerDB, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get8AreaShooting1Turn(fromDt, managerDB, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.get20AreaShooting5Turn(fromDt, managerDB, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.getStrip4150km1Turn(fromDt, managerDB, out mpzArray);
            }
            {
                List<MPZ> mpzArray;
                SessionsPlanning.TestSessionsSequenses.getStrip12050km5Turn(fromDt, managerDB, out mpzArray);
            }

            List<MPZ> mpzArray2;
            SessionsPlanning.TestSessionsSequenses.get20AreaShooting5Turn(fromDt, managerDB, out mpzArray2);

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
            test_getCaptureConfArray();
            Console.ReadKey();
        }
    }
}
