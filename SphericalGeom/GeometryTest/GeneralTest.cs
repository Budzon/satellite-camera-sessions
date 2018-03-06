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
        public void TestGetCaptureConfArrayOnRandomPolygons()
        {
            List<Polygon> polygons = new List<Polygon>();
            Random rand = new Random((int)DateTime.Now.Ticks);
            for (int i = 0; i < 30; i++)
            {
                Polygon randpol = getRandomPolygon(rand, 3, 12, 2, 8);
                polygons.Add(randpol);
            }

            try
            {
                int id = 0;
                List<RequestParams> requests = new List<RequestParams>();
                foreach (var pol in polygons)
                {
                    RequestParams reqparams = new RequestParams();
                    reqparams.id = id;
                    reqparams.timeFrom = new DateTime(2015, 3, 12); // 12.03.2015 по 14.03.2016 
                    reqparams.timeTo = new DateTime(2016, 3, 14);
                    reqparams.priority = 1;
                    reqparams.minCoverPerc = 0.4;
                    reqparams.Max_SOEN_anlge = AstronomyMath.ToRad(45);
                    reqparams.wktPolygon = pol.ToWtk();
                    requests.Add(reqparams);
                    id++;
                }
                var res = Sessions.getCaptureConfArray(
                                    requests,
                                    new DateTime(2000, 03, 13, 4, 0, 0),
                                    new DateTime(2115, 03, 13, 4, 4, 0));                
            }
            catch (Exception ex)
            {
                Console.WriteLine("Ошибка обнаружена на следующем наборе полигонов:");
                foreach (var pol in polygons)
                {                    
                    Console.WriteLine(pol.ToWtk());
                }
                throw new Exception("error");
            }
 


        }

        //[TestMethod]
        //public void TestIsRequestFeasible()
        //{
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
        //}



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
