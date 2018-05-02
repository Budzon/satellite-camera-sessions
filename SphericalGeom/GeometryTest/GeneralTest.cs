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
    public class GeneralTest
    {
        [TestMethod]
        public void TestLitSpans()
        {
            DateTime dt1 = DateTime.Parse("01.02.2019 0:47:50");
            DateTime dt2 = DateTime.Parse("01.02.2019 1:39:30");
            //DateTime dt2 = DateTime.Parse("01.02.2019 2:39:30");
            string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);


            for (int i = 0; i < 50; i++)
            {
                List<Tuple<DateTime, DateTime>> shadowPeriods;
                List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot;
                Sessions.checkIfViewLaneIsLitWithTimeSpans(managerDB, dt1, dt2, out partsLitAndNot, out shadowPeriods);

                Console.WriteLine(i + " count=" + shadowPeriods.Count);
                if (shadowPeriods.Count != 0)
                    foreach (var loop_wkts in partsLitAndNot)
                        Console.WriteLine(Polygon.getMultipolFromWkts(loop_wkts.Item2.Select(wktlit => wktlit.wktPolygon).ToList()));
                foreach (var period in shadowPeriods)
                    Console.WriteLine(period.Item1 + " " + period.Item2);
            }
        }

        [TestMethod]
        public void TestMPZ()
        {
            string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
            DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);

            List<RouteParams> param = new List<RouteParams>();
            OptimalChain.StaticConf conf;

            string[] chan = new string[3] { "pk", "mk", "cm" };
            int[] regime = new int[4] { 0, 1, 2, 3 }; // Zi, Vi, Si, Np
            int[] shooting = new int[3] { 0, 1, 2 }; // прост, стерео, коридор
            int[] compression = new int[5] { 0, 1, 2, 7, 10 };
            DateTime from = new DateTime(2019, 1, 5);
            DateTime to = from.AddSeconds(5);
            
            int k = 0;
            foreach(string ch in chan)
                foreach(int r in regime)
                    foreach(int s in shooting)
                        foreach (int c in compression)
                        {
                            conf = new StaticConf(k, from, to, 0, 0, 0, null, "", c, 0.3, r, ch, s);
                            RouteParams p = new RouteParams(conf);
                            p.albedo = 0.3;
                            p.coridorAzimuth = 0.5;
                            p.coridorLength = 40000;
                            p.Delta_T = 0;
                            p.duration = 10;
                            p.TNPos = 0;
                            p.binded_route = Tuple.Create(101, 1);
                            param.Add(p);
                            from = to.AddSeconds(70);
                            to = from.AddSeconds(5);
                            k++;
                        }
            var mpzParams = OptimalChain.MPZParams.FillMPZ(param);
            FlagsMPZ flags = new FlagsMPZ();
            var mpzs = mpzParams.Select(p => new MPZ(p, manager, flags));
            string mpz_string = mpzs.Aggregate("", (s, mpz) => s + mpz.ToString() + "\n");
            System.IO.File.WriteAllText(@"mpz_text.txt", mpz_string);
        }

        [TestMethod]
        public void TestLitSpans()
        {
            DateTime dt1 = DateTime.Parse("01.02.2019 0:47:50");
            DateTime dt2 = DateTime.Parse("01.02.2019 1:39:30");
            string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);


            for (int i = 0; i < 10; i++)
            {
                List<Tuple<DateTime, DateTime>> shadowPeriods;
                List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot;
                Sessions.checkIfViewLaneIsLitWithTimeSpans(managerDB, dt1, dt2, out partsLitAndNot, out shadowPeriods);

                Console.WriteLine(i + " count=" + shadowPeriods.Count);
                foreach (var loop_wkts in partsLitAndNot)
                    Console.WriteLine(Polygon.getMultipolFromWkts(loop_wkts.Item2.Select(wktlit => wktlit.wktPolygon).ToList()));
                foreach (var period in shadowPeriods)
                    Console.WriteLine(period.Item1 + " " + period.Item2);
            }
        }

        [TestMethod]
        public void TestPiecewiseCoridor()
        {
            string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
            DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);


            // -------------------------------- СЕВЕРНОЕ ПОЛУШАРИЕ ПРИМЕР
            // def f(x): return 81.8 + 0.1*np.cos(np.pi/10*x)
            //lons = np.linspace(60, 90, 100)
            //DateTime dt1 = new DateTime(2019, 1, 1, 10, 47, 30);
            
            //List<string> wkts;
            //List<GeoPoint> satPos;
            //List<GeoPoint> verts = new List<GeoPoint>() {
            //    new GeoPoint(81.7, 90),
            //    //new GeoPoint(81.8, 85),
            //    new GeoPoint(81.9, 80),
            //    //new GeoPoint(81.8, 75),
            //    new GeoPoint(81.7, 70),
            //    //new GeoPoint(81.8, 65),
            //    new GeoPoint(81.9, 60)
            //};    

            // --------------ЮЖНОЕ
            //DateTime dt1 = new DateTime(2019, 1, 1, 9, 58, 00);
//            def f(x):
//    return -81.8 - 0.1*sp.sinc((x + 90) / 3)
    
//# lons = np.linspace(-80, -100, 100)
            //List<string> wkts;
            //List<GeoPoint> satPos;
            //List<GeoPoint> verts = new List<GeoPoint>() {
            //    new GeoPoint(-81.79173007, -100),
            //    new GeoPoint(-81.8127324, -97.5),
            //    new GeoPoint(-81.77932517, -94),
            //    new GeoPoint(-81.9, -90),
            //    new GeoPoint(-81.77932517, -86),
            //    new GeoPoint(-81.8127324, -82.5),
            //    new GeoPoint(-81.79173007, -80)
            //}; 
     
            //Sessions.getPieciwiseCoridor(dt1, verts, manager, out wkts, out satPos);

            /// ---------------- СЕВЕР КАВАЙНЫЙ ПРИМЕР
            /// f(x) = 81.8 + 0.05*np.cos(np.pi/5*x)
            /// lons = np.linspace(60, 90, 200)
            /// DateTime dt1 = new DateTime(2019, 1, 1, 10, 47, 30);

            // CUSTOM
            DateTime dt1 = new DateTime(2019, 1, 1, 10, 56, 30);
            int steps = 100;
            double lon0 = -15, lon1 = -5, dlon = lon1 - lon0, step = dlon / steps;
            
            double[] lons = new double[steps];
            for (int i = 0; i < lons.Length; ++i)
            {
                lons[i] = lon0 + step * (lons.Length - i - 1);
            }
            double[] lats = new double[steps];
            for (int i = 0; i < lats.Length; ++i)
            {
                lats[i] = 50 + 3 * Math.Cos(Math.PI / 4 * lons[i]) + 0.5 * Math.Sin(lons[i] * 3);
            }
            //+ 2e-4 * (lons[i] - 60) * Math.Pow(lons[i] - 90, 2);
            List<string> wkts;
            List<GeoPoint> satPos;
            List<GeoPoint> curve = new List<GeoPoint>();
            for (int i = 0; i < lons.Length; ++i)
                curve.Add(new GeoPoint(lats[i], lons[i]));

            Sessions.getPieciwiseCoridor(dt1, curve, manager, out wkts, out satPos, custom: true);
            Console.WriteLine(wkts.Aggregate("[", (tail, wkt) => tail + ", '" + wkt + "'") + "]");
            Console.WriteLine(satPos.Aggregate("[", (tail, pos) => tail + ", (" + pos + ")") + "]");
        }

        [TestMethod]
        public void TestCoridorPoly()
        {
            string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
            DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);

            DateTime dt1 = new DateTime(2019, 1, 1, 10, 47, 30);
            string wkt;
            double dur, dist = 50e3;
            double roll = 0, pitch = 0, az = 0;
            SatelliteSessions.Sessions.getCoridorPoly(
                dt1,
                AstronomyMath.ToRad(roll), AstronomyMath.ToRad(pitch),
                dist, AstronomyMath.ToRad(az),
                manager, out wkt, out dur);
            Console.WriteLine(wkt);
            //SatelliteSessions.Sessions.getCoridorPoly(
            //    dt1,
            //    AstronomyMath.ToRad(roll), AstronomyMath.ToRad(pitch),
            //    new GeoPoint(6.32, 143.55),
            //    manager, out wkt, out dur, out dist);
            //Console.WriteLine(wkt);
        }

        [TestMethod]
        public void TestGetCaptureConfArrayOnRandomPolygons()
        {
            for (int testi = 0; testi < 20; testi++)
           {
                List<Polygon> polygons = new List<Polygon>();
                Random rand = new Random((int)DateTime.Now.Ticks);
                for (int i = 0; i < 30; i++)
                {
                    Polygon randpol = getRandomPolygon(rand, 3, 8, 2, 8);
                    polygons.Add(randpol);
                }
                
                string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
                DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);

                DateTime dt1 = new DateTime(2019, 1, 4);
                DateTime dt2 = new DateTime(2019, 1, 8);

                var inactivityRanges = new List<Tuple<DateTime, DateTime>>();
                inactivityRanges.Add(Tuple.Create(new DateTime(2019, 1, 5), new DateTime(2019, 1, 6))); 


                DataFetcher fetcher = new DataFetcher(manager);
                Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

                if (trajectory.Count == 0)
                    throw new Exception("На эти даты нет траектории в БД, тест некорректный");

                try
                {
                    int id = 0;
                    List<RequestParams> requests = new List<RequestParams>();
                    foreach (var pol in polygons)
                    {
                        RequestParams reqparams = new RequestParams();
                        reqparams.id = id;
                        reqparams.timeFrom = dt1;
                        reqparams.timeTo = dt2;
                        reqparams.priority = 1;
                        reqparams.minCoverPerc = 0.4;
                        reqparams.Max_SOEN_anlge = AstronomyMath.ToRad(45);
                        reqparams.wktPolygon = pol.ToWtk();
                        requests.Add(reqparams);
                        id++;
                    }
                    var res = Sessions.getCaptureConfArray(requests, dt1, dt2, manager, inactivityRanges, new List<Tuple<DateTime,DateTime>>());
                }

                catch (Exception ex)
                {
                    List<string> lines = new List<string>();
                    Console.WriteLine("Ошибка обнаружена на следующем наборе полигонов:");
                    foreach (var pol in polygons)
                    {
                        Console.WriteLine(pol.ToWtk());
                        lines.Add(pol.ToWtk());
                    }
                    System.IO.File.WriteAllLines(@"badPolygons.txt", lines);
                    throw ex;
                }
            }
        }


        [TestMethod]
        public void Test_getMPZArray()
        {
            List<Polygon> polygons = new List<Polygon>();
            Random rand = new Random((int)DateTime.Now.Ticks);
            for (int i = 0; i < 2; i++) 
            {
                Polygon randpol = getRandomPolygon(rand, 3, 6, 2, 4);
                polygons.Add(randpol);
            }

            string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
            DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);

            DateTime dt1 = new DateTime(2019, 1, 4);
            DateTime dt2 = new DateTime(2019, 1, 4, 20,0,0);

            DataFetcher fetcher = new DataFetcher(manager);
            Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

            if (trajectory.Count == 0)
                throw new Exception("На эти даты нет траектории в БД, тест некорректный");

            try
            {
                int id = 0;
                List<RequestParams> requests = new List<RequestParams>();
                foreach (var pol in polygons)
                {
                    RequestParams reqparams = new RequestParams();
                    reqparams.id = id;
                    reqparams.timeFrom = dt1;
                    reqparams.timeTo = dt2;
                    reqparams.priority = 1;
                    reqparams.compression = 10;
                    reqparams.minCoverPerc = 0.4;
                    reqparams.Max_SOEN_anlge = AstronomyMath.ToRad(45);
                    reqparams.wktPolygon = pol.ToWtk();
                    requests.Add(reqparams);
                    id++;
                }
              //  var res = Sessions.getCaptureConfArray(requests, dt1, dt2, manager, new List<Tuple<DateTime, DateTime>>());

                Order order = new Order();
                order.captured = new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))");
                order.intersection_coeff = 0.1;
                order.request = new RequestParams();
                order.request.priority = 1;
                order.request.timeFrom = new DateTime(2019, 1, 4);
                order.request.timeTo = new DateTime(2019, 1, 5);
                order.request.wktPolygon = "POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))";
                order.request.minCoverPerc = 0.4;
                order.request.Max_SOEN_anlge = AstronomyMath.ToRad(45);
                List<Order> orders = new List<Order>() { order };

                CaptureConf ccToDrop = new CaptureConf(new DateTime(2019, 1, 4), new DateTime(2019, 1, 5), 0.1, orders, 1, null);
                StaticConf sc = ccToDrop.DefaultStaticConf();
                RouteParams routeParamtoDrop = new RouteParams(sc);
                routeParamtoDrop.id = 0;
                routeParamtoDrop.start = new DateTime(2019, 1, 4);
                routeParamtoDrop.end = new DateTime(2019, 1, 5);
                routeParamtoDrop.File_Size = 1000;
                routeParamtoDrop.binded_route = new Tuple<int, int>(1, 1);
                // double timedrop = routeParam.getDropTime();

                RouteMPZ routempzToDrop = new RouteMPZ(routeParamtoDrop, manager) { NPZ = 0, Nroute = 0 };

                List<RouteMPZ> routesToDrop = new List<RouteMPZ>();
                routesToDrop.Add(routempzToDrop);


                CaptureConf ccToDelete = new CaptureConf(new DateTime(2019, 1, 4), new DateTime(2019, 1, 5), 0.1, orders, 2, null);
                StaticConf scToDelete = ccToDelete.DefaultStaticConf();
                RouteParams routeParamtoDelete = new RouteParams(scToDelete);
                routeParamtoDelete.id = 0;
                routeParamtoDelete.start = new DateTime(2019, 1, 4);
                routeParamtoDelete.end = new DateTime(2019, 1, 5);
                routeParamtoDelete.File_Size = 1000;
                routeParamtoDelete.binded_route = new Tuple<int, int>(1, 1);
                RouteMPZ routempzToDelete = new RouteMPZ(routeParamtoDelete, manager) { NPZ = 0, Nroute = 0 };

                List<RouteMPZ> routesToDelete = new List<RouteMPZ>();
                routesToDelete.Add(routempzToDelete);

                List<Tuple<DateTime, DateTime>> silenceRanges = new List<Tuple<DateTime, DateTime>>();
               // silenceRanges.Add(Tuple.Create(new DateTime(2019, 1, 6), new DateTime(2019, 1, 9)));
                //silenceRanges.Add(Tuple.Create(new DateTime(2019, 1, 6), new DateTime(2019, 1, 6)));
                 

                var inactivityRanges = new List<Tuple<DateTime, DateTime>>();
               // inactivityRanges.Add(Tuple.Create(new DateTime(2019, 1, 5), new DateTime(2019, 1, 6))); 

                 
                List<MPZ> mpzArray;
                List<CommunicationSession> sessions;

                Sessions.getMPZArray(requests, dt1, dt2
                                                    , silenceRanges
                                                    , inactivityRanges
                                                     , routesToDrop
                                                     , routesToDelete
                                                      , manager
                                                      ,0
                                                     , out mpzArray
                                                     , out sessions);
            }

            catch (Exception ex)
            {
                List<string> lines = new List<string>();
                Console.WriteLine("Ошибка обнаружена на следующем наборе полигонов:");
                foreach (var pol in polygons)
                {
                    Console.WriteLine(pol.ToWtk());
                    lines.Add(pol.ToWtk());
                }
                System.IO.File.WriteAllLines(@"badPolygons.txt", lines);
                throw ex;
            }

        }
         


        [TestMethod]
        public void TestIsRequestFeasible()
        {

            for (int testi = 0; testi < 2; testi++)
            {

                List<Polygon> polygons = new List<Polygon>();
                Random rand = new Random((int)DateTime.Now.Ticks);
                for (int i = 0; i < 10; i++)
                {
                    Polygon randpol = getRandomPolygon(rand, 3, 12, 2, 8);
                    polygons.Add(randpol);
                }

                string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
                DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);

                DateTime dt1 = new DateTime(2019, 1, 5);
                DateTime dt2 = new DateTime(2019, 1, 6);

                DataFetcher fetcher = new DataFetcher(manager);
                Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

                if (trajectory.Count == 0)
                    throw new Exception("На эти даты нет траектории в БД, тест некорректный");


                foreach (var pol in polygons)
                {
                    try
                    {
                        RequestParams reqparams = new RequestParams();
                        reqparams.id = 0;
                        reqparams.timeFrom = dt1;
                        reqparams.timeTo = dt2;
                        reqparams.priority = 1;
                        reqparams.minCoverPerc = 0.4;
                        reqparams.Max_SOEN_anlge = AstronomyMath.ToRad(45);
                        reqparams.wktPolygon = pol.ToWtk();

                        double cover;
                        List<CaptureConf> output;
                        Sessions.isRequestFeasible(reqparams, dt1, dt2, manager, out cover, out output);
                    }
                    catch (Exception ex)
                    {
                        List<string> lines = new List<string>();
                        Console.WriteLine("Ошибка обнаружена на следующем полигонt:");
                        Console.WriteLine(pol.ToWtk());
                        lines.Add(pol.ToWtk());                        
                        System.IO.File.WriteAllLines(@"badPolygons.txt", lines);
                        throw ex;
                    }
                    
                }
            }

        }

           



            //    DBTables.DataFetcher fetcher = new DBTables.DataFetcher(manager);

            //    DateTime from = new DateTime(2019, 01, 01, 1, 0, 0);
            //    DateTime to = new DateTime(2019, 01, 05, 12, 0, 0);

            //    //var sun = fetcher.GetPositionSun(from, to);
            //    //sun.Clear();
            //    //var sat = fetcher.GetPositionSat(from, to);
            //    //sat.Clear();
            //    //var traj = fetcher.GetTrajectorySat(from, to);
            //    var viewLane = fetcher.GetViewLane(from, to);
            //    //var orbit = fetcher.GetDataBetweenDates(OrbitTable.Name, OrbitTable.TimeEquator, from, to);
            //    //List<Tuple<int, DateTime>> orbit_turns = orbit.Select(row => Tuple.Create(OrbitTable.GetNumTurn(row), OrbitTable.GetTimeEquator(row))).ToList();
            //    //var turns = fetcher.GetViewLaneBrokenIntoTurns(from, to);
            //
      


        /// <summary>
        /// генерируем случайный полигон
        /// </summary>
        /// <param name="minVertNumb"> минимальное колво вершин </param>
        /// <param name="maxVertNumb"> максимальное колво вершин </param>
        /// <param name="minBoundSize"> минимальный размер полигона (коробочный) в градусах</param>
        /// <param name="maxBoundSize"> максимальный размер полигона (коробочный) в градусах</param>
        /// <returns>полигон</returns>
        public Polygon getRandomPolygon(Random rand, int minVertNumb, int maxVertNumb, double minBoundSize, double maxBoundSize)
        {
            double lat0 = (double)rand.Next(-90, 90);
            double lon0 = (double)rand.Next(-179, 180);

            double a = (double)rand.Next((int)(minBoundSize) * 10, (int)(maxBoundSize * 10)) / 10;
            double b = (double)rand.Next((int)(minBoundSize) * 10, (int)(maxBoundSize * 10)) / 10;

            int vertNum = rand.Next(minVertNumb, maxVertNumb);

            SortedDictionary<double, Vector3D> verts = new SortedDictionary<double, Vector3D>();

            double minDist = AstronomyMath.ToRad(0.1);// Math.Min(Math.PI / 10, Math.PI * 2 / vertNum); // не дадим генерировать точки слишком близко к друг другу.

            for (int i = 0; i < vertNum; i++)
            {
            Begin:
                double t = (double)(rand.Next(0, (int)(2 * Math.PI * 100))) / 100;
                double lat = lat0 + a * Math.Cos(t);
                double lon = lon0 + b * Math.Sin(t);
                Vector3D vert = GeoPoint.ToCartesian(new GeoPoint(lat, lon), 1);
                foreach (var item in verts) // проверим, нету ли слишком близкой точки
                {
                    if (GeoPoint.DistanceOverSurface(item.Value, vert) < minDist)
                        goto Begin; // да, это goto.
                }
                verts[t] = vert;
            }

            return new Polygon(verts.Values.ToList<Vector3D>());
        }
    }
}
