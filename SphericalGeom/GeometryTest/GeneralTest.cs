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
        public void TestCoridorPoly()
        {
            string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
            DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);

            DateTime dt1 = new DateTime(2019, 1, 1);
            string wkt2 = SatelliteSessions.Sessions.getSOENViewPolygon(dt1, AstronomyMath.ToRad(20), AstronomyMath.ToRad(10), 5000, manager, true);
        }

        [TestMethod]
        public void TestGetCaptureConfArrayOnRandomPolygons()
        {
            for (int testi = 0; testi < 5; testi++)
            {
                List<Polygon> polygons = new List<Polygon>();
                Random rand = new Random((int)DateTime.Now.Ticks);
                for (int i = 0; i < 15; i++)
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
                    var res = Sessions.getCaptureConfArray(requests, dt1, dt2, manager, inactivityRanges);
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
            for (int i = 0; i < 2; i++) // у меня памяти не хватает на больший тест
            {
                Polygon randpol = getRandomPolygon(rand, 3, 6, 2, 4);
                polygons.Add(randpol);
            }

            string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
            DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);

            DateTime dt1 = new DateTime(2019, 1, 4);
            DateTime dt2 = new DateTime(2019, 1, 8);

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

                RouteMPZ routempzToDrop = new RouteMPZ(routeParamtoDrop) { NPZ = 0, Nroute = 0 };

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
                RouteMPZ routempzToDelete = new RouteMPZ(routeParamtoDelete) { NPZ = 0, Nroute = 0 };

                List<RouteMPZ> routesToDelete = new List<RouteMPZ>();
                routesToDelete.Add(routempzToDelete);

                List<Tuple<DateTime, DateTime>> silenceRanges = new List<Tuple<DateTime, DateTime>>();
                silenceRanges.Add(Tuple.Create(new DateTime(2019, 1, 4), new DateTime(2019, 1, 5)));
                silenceRanges.Add(Tuple.Create(new DateTime(2019, 1, 6), new DateTime(2019, 1, 6)));
                 

                var inactivityRanges = new List<Tuple<DateTime, DateTime>>();
                inactivityRanges.Add(Tuple.Create(new DateTime(2019, 1, 5), new DateTime(2019, 1, 6))); 

                 
                List<MPZ> mpzArray;
                List<CommunicationSession> sessions;

                Sessions.getMPZArray(requests, dt1, dt2
                                                    , silenceRanges
                                                    , inactivityRanges
                                                     , routesToDrop
                                                     , routesToDelete
                                                      , manager
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
