using System;
using System.Collections.Generic;
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
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using Microsoft.SqlServer.Types;
using System.Data.SqlTypes;

using Common;
using SphericalGeom;
using SatelliteRequests;
using SatelliteTrajectory;
using SatelliteSessions;
using DataParsers;
using Astronomy;
using OptimalChain;

namespace ViewModel
{
    public class Requests : ObservableCollection<Request>
    {
        public Requests()
            : base()
        { }
    }

    public class EarthSatelliteViewModel : INotifyPropertyChanged
    {
        private SphericalGeom.Camera camera;
        private Polygon coneBase;
        private Polygon curRequest;

        public Polygon pol1;
        public Polygon pol2;
        public Polygon pol3;

        public List<Polygon> polygons;
        public List<Polygon> captureIntervals;
        public List<Polygon> blackPolygons;

        private List<Polygon> curBbox;
        private List<Polygon> curIntersection;
        private List<Polygon> curDifference;
        private List<SatLane> captureLanes;


        private bool hasChanged;

        private readonly DelegateCommand addRequestCmd;
        private readonly DelegateCommand removeRequestCmd;
        private readonly DelegateCommand addPointCmd;
        private readonly DelegateCommand removePointCmd;
        private readonly DelegateCommand loadWTK;
        //private readonly DelegateCommand verifyIfRegionCanBeSeenCmd;

        public event PropertyChangedEventHandler PropertyChanged;

        private void UpdateConeBase()
        {
            hasChanged = true;
            curIntersection.Clear();
            curDifference.Clear();
            coneBase = Routines.ProjectConeOntoSphere(camera.Position, camera.InnerNormalsToCone);
        }

        private void UpdateCurRequest()
        {
            hasChanged = true;
            curIntersection.Clear();
            curDifference.Clear();
            if (SelectedRequest > -1)
            {
                curRequest = new SphericalGeom.Polygon(
                    Requests[SelectedRequest].Polygon.Select(sp => GeoPoint.ToCartesian(new GeoPoint(sp.Lat, sp.Lon), 1)),
                    new Vector3D(0, 0, 0));
                if (curRequest.Vertices.Count() > 3)
                    curBbox = Routines.SliceIntoSquares(curRequest, new Vector3D(1, 0, 0), 30, 4).Where((s, ind) => ind % 3 == 0).ToList();
            }
            else
                curRequest = null;
        }

        public EarthSatelliteViewModel()
        {
            polygons = new List<Polygon>();
            blackPolygons = new List<Polygon>();
            camera = new SphericalGeom.Camera();
            curIntersection = new List<Polygon>();
            curDifference = new List<Polygon>();
            captureLanes = new List<SatLane>();
            captureIntervals = new List<Polygon>();
            UpdateConeBase();
            Requests = new Requests();
            DatFileName = "trajectory.dat";
            wtkPolygonStr = "POLYGON ((20 -20, 20 20, -20 20, -20 -20, 20 -20))";
            addRequestCmd = new DelegateCommand(_ =>
            {
                Requests.Add(new Request(RequestId));
                SelectedRequest = Requests.Count - 1;
                RaisePropertyChanged("SelectedRequest");
                RequestId = 0;
                RaisePropertyChanged("RequestId");
            });

            removeRequestCmd = new DelegateCommand(_ =>
            {
                Requests.RemoveAt(SelectedRequest);
            }, _ => { return SelectedRequest != -1; });

            addPointCmd = new DelegateCommand(_ =>
            {
                Requests.ElementAt(SelectedRequest).Polygon.Add(new SurfacePoint(NewPointLat, NewPointLon));
                UpdateCurRequest();
                NewPointLon = 0;
                NewPointLat = 0;
                RaisePropertyChanged("NewPointLon");
                RaisePropertyChanged("NewPointLat");
            }, _ => { return SelectedRequest != -1; });

            removePointCmd = new DelegateCommand(_ =>
            {
                Requests.ElementAt(SelectedRequest).Polygon.RemoveAt(SelectedPoint);
                UpdateCurRequest();
            }, _ => { return (SelectedRequest != -1 && SelectedPoint != -1); });

            loadWTK = new DelegateCommand(_ =>
            {
                RequestId = Requests.Count;
                addRequestCmd.Execute(new object());
                SqlGeography geom = SqlGeography.STGeomFromText(new SqlChars(wtkPolygonStr), 4326);
                for (int i = 1; i < geom.STNumPoints(); i++)
                {
                    NewPointLat = (double)geom.STPointN(i).Lat;
                    NewPointLon = (double)geom.STPointN(i).Long;
                    addPointCmd.Execute(new object());
                }
                curRequest = new SphericalGeom.Polygon(Requests[Requests.Count - 1].Polygon.Select(sp => GeoPoint.ToCartesian(new GeoPoint(sp.Lat, sp.Lon), 1)).ToList<Vector3D>(), new Vector3D(0, 0, 0));
            }, _ => { return true; });

            //verifyIfRegionCanBeSeenCmd = new DelegateCommand(_ =>
            //{
            //    var result = camera.RegionCanBeSeen(Requests.ElementAt(SelectedRequest).Polygon.Select(point => new vector3(new direction3(point.Lat, point.Lon), 1)).ToList());

            //    RegionCanBeCaptured = result.Item1;
            //    SatellitePitch = result.Item2.Lat;
            //    SatelliteYaw = result.Item2.Lon;
            //    SatelliteRoll = result.Item3;
            //    RaisePropertyChanged("RegionCanBeCaptured");
            //    RaisePropertyChanged("SatelliteRoll");
            //    RaisePropertyChanged("SatellitePitch");
            //    RaisePropertyChanged("SatelliteYaw");
            //}, _ => { return SelectedRequest != -1; });
        }

        public void RaisePropertyChanged(string s)
        {
            if (PropertyChanged != null)
                PropertyChanged(this, new PropertyChangedEventArgs(s));
        }

        public Requests Requests { get; set; }

        public double SatelliteLat
        {
            get { return camera.PositionDirection.Latitude; }
            set
            {
                camera.Position = GeoPoint.ToCartesian(new GeoPoint(value, SatelliteLon), SatelliteAltitude + 1);
                UpdateConeBase();
            }
        }
        public double SatelliteLon
        {
            get { return camera.PositionDirection.Longitude; }
            set
            {
                camera.Position = GeoPoint.ToCartesian(new GeoPoint(SatelliteLat, value), SatelliteAltitude + 1);
                UpdateConeBase();
            }
        }
        public double SatelliteAltitude
        {
            get { return camera.Position.Length - 1; }
            set
            {
                camera.Position = GeoPoint.ToCartesian(new GeoPoint(SatelliteLat, SatelliteLon), value + 1);
                UpdateConeBase();
            }
        }
        public double SatelliteVerticalAngleOfView
        {
            get { return camera.VerticalHalfAngleOfView * 2; }
            set
            {
                camera.VerticalHalfAngleOfView = value / 2;
                UpdateConeBase();
            }
        }
        public double SatelliteHorizontalAngleOfView
        {
            get { return camera.HorizontalHalfAngleOfView * 2; }
            set
            {
                camera.HorizontalHalfAngleOfView = value / 2;
                UpdateConeBase();
            }
        }

        public int RequestId { get; set; }
        private int selectedRequest;
        public int SelectedRequest
        {
            get
            {
                return selectedRequest;
            }
            set
            {
                selectedRequest = value;
                UpdateCurRequest();
            }
        }

        public double NewPointLon { get; set; }
        public double NewPointLat { get; set; }
        public int SelectedPoint { get; set; }
        public string DatFileName { get; set; }
        public string wtkPolygonStr { get; set; }

        public bool PointInPolygon(double x, double y, double z)
        {
            if (curRequest != null)
                return curRequest.Contains(new Vector3D(x, y, z));
            else
                return false;
        }

        public bool PointInBoundingBox(double x, double y, double z)
        {
            Vector3D v = new Vector3D(x, y, z);
            return curBbox.Any(square => square.Contains(v));
        }

        public bool PointInCamera(double x, double y, double z)
        {
            return coneBase.Contains(new Vector3D(x, y, z));
        }
        public bool PointInRegion(double x, double y, double z)
        {
            return curRequest.Contains(new Vector3D(x, y, z));
        }
        public bool PointInCaptureInterval(double x, double y, double z)
        {
            Vector3D v = new Vector3D(x, y, z);
            foreach (Polygon pol in captureIntervals)
            {
                if (pol.Contains(v))
                    return true;
            }
            return false;
        }
        public bool PointInLane(double x, double y, double z)
        {
            Vector3D v = new Vector3D(x, y, z);
            foreach (SatLane lane in captureLanes)
            {
                foreach (LaneSector sector in lane.Sectors)
                {
                    if (sector.polygon.Contains(v))
                        return true;
                }
            }
            return false;
        }

        public bool PointInIntersection(double x, double y, double z)
        {
            if (hasChanged)
            {
                hasChanged = false;
                var tmp = Polygon.IntersectAndSubtract(curRequest, coneBase);
                curIntersection = tmp.Item1.ToList();
                curDifference = tmp.Item2.ToList();
            }
            var v = new Vector3D(x, y, z);
            return curIntersection.Any(p => p.Contains(v));
        }

        public bool PointInDifference(double x, double y, double z)
        {
            if (hasChanged)
            {
                hasChanged = false;
                var tmp = Polygon.IntersectAndSubtract(curRequest, coneBase);
                curIntersection = tmp.Item1.ToList();
                curDifference = tmp.Item2.ToList();
            }
            var v = new Vector3D(x, y, z);
            return curDifference.Any(p => p.Contains(v));
        }

        public void test_vectors(Vector3D crossVector_1, Vector3D crossVector_2, Vector3D crossVector_3, Vector3D crossVector_4)
        {
            double lenght = 100;
            crossVector_1.Normalize();
            crossVector_2.Normalize();
            crossVector_3.Normalize();
            crossVector_4.Normalize();

            crossVector_1 = lenght * crossVector_1;
            crossVector_2 = lenght * crossVector_2;
            crossVector_3 = lenght * crossVector_3;
            crossVector_4 = lenght * crossVector_4;

            double dist_12 = (crossVector_2 - crossVector_1).Length;
            double dist_23 = (crossVector_3 - crossVector_2).Length;
            double dist_34 = (crossVector_4 - crossVector_3).Length;
            double dist_41 = (crossVector_4 - crossVector_1).Length;

            Console.WriteLine("dist_12 = {0}", dist_12);
            Console.WriteLine("dist_23 = {0}", dist_23);
            Console.WriteLine("dist_34 = {0}", dist_34);
            Console.WriteLine("dist_41 = {0}", dist_41);
            Console.WriteLine();
            Console.WriteLine("angle_12 = {0}; ", Vector3D.AngleBetween(crossVector_1, crossVector_2));
            Console.WriteLine("angle_23 = {0}; ", Vector3D.AngleBetween(crossVector_3, crossVector_2));
            Console.WriteLine("angle_34 = {0}; ", Vector3D.AngleBetween(crossVector_4, crossVector_3));
            Console.WriteLine("angle_41 = {0}; ", Vector3D.AngleBetween(crossVector_4, crossVector_1));
            Console.WriteLine();
            Console.WriteLine();
        }

        public double getRollCorrection(double h, double v, double w, double b, double pitch)
        {
            double I = OptimalChain.Constants.orbital_inclination;
            double R = Astronomy.Constants.EarthRadius;
            //double bm = b + Math.Sin(I) * (Math.Acos(Math.Sqrt(1 - (R + h) * (R + h) / R / R * Math.Sin(pitch) * Math.Sin(pitch))) - pitch);
            //double d = Math.Cos(bm) * w / v * pitch * Math.Sin(I);
            //double sinRoll = R * Math.Sin(d) / Math.Sqrt(R * R + (R + h) * (R + h) - 2 * R * (R + h) * Math.Cos(d));
            //return Math.Asin(sinRoll); 
            double bm = b + Math.Sin(I) * (Math.Acos(Math.Sqrt(1 - Math.Pow((R + h) / R * Math.Sin(pitch), 2))) - pitch);
            double d = Math.Cos(bm) * w / v * pitch * Math.Sin(I);
            double sinRoll = R * Math.Sin(d) / Math.Sqrt(Math.Pow(R, 2) + Math.Pow(R + h, 2) - 2 * R * (R + h) * Math.Cos(d));
            return Math.Asin(sinRoll);
        }

        public Polygon getPointPolygon(Vector3D point, double delta = 0.1)
        {
            GeoPoint gp = GeoPoint.FromCartesian(point);
            Console.WriteLine(gp.ToString());


            GeoPoint gp1 = new GeoPoint(gp.Latitude + delta, gp.Longitude);
            GeoPoint gp2 = new GeoPoint(gp.Latitude, gp.Longitude + delta);
            GeoPoint gp3 = new GeoPoint(gp.Latitude - delta, gp.Longitude);
            GeoPoint gp4 = new GeoPoint(gp.Latitude, gp.Longitude - delta);

            List<Vector3D> verts = new List<Vector3D>();
            verts.Add(GeoPoint.ToCartesian(gp1, 1));
            verts.Add(GeoPoint.ToCartesian(gp2, 1));
            verts.Add(GeoPoint.ToCartesian(gp3, 1));
            verts.Add(GeoPoint.ToCartesian(gp4, 1));

            return new Polygon(verts);
        }

        public void test_roll_correction_noroll()
        {
            double viewAngle = AstronomyMath.ToRad(1);
            Astronomy.Trajectory trajectory = DatParser.getTrajectoryFromDatFile(DatFileName, new DateTime(2000, 1, 1), new DateTime(2020, 1, 1));

            SatLane strip = new SatLane(trajectory, 0, 0, viewAngle);
            captureLanes.Add(strip);

            int count = trajectory.Count;
            var points = trajectory.Points;
            var point = points[514];

            // начальные данные
            double pitchAngle = AstronomyMath.ToRad(30);

            var nadirPoint = LanePos.getSurfacePoint(point, 0, 0);
            Vector3D pointPitch = LanePos.getSurfacePoint(point, 0, pitchAngle);
            double h = point.Position.ToVector().Length - Astronomy.Constants.EarthRadius;
            double w = OptimalChain.Constants.earthRotSpeed;
            double b = AstronomyMath.ToRad(GeoPoint.FromCartesian(nadirPoint).Latitude);
            double velo = point.Velocity.Length / (Astronomy.Constants.EarthRadius + h); // скорость в радианах
            Console.WriteLine("point = {0};", point.Position);
            Console.WriteLine("velo = {0};", point.Velocity);
            Console.WriteLine("point geo = {0};", GeoPoint.FromCartesian(point.Position.ToVector()));

            Console.WriteLine("nadirPoint = {0};", nadirPoint);

            Console.WriteLine("h = {0};", h);
            Console.WriteLine("i = {0};", OptimalChain.Constants.orbital_inclination);
            Console.WriteLine("w = {0};", w);
            Console.WriteLine("b = {0};", b);
            Console.WriteLine("alpha = {0};", pitchAngle);
            Console.WriteLine("R = {0};", Astronomy.Constants.EarthRadius);
            Console.WriteLine("v = {0};", velo);

            // расчитаем поправку по крену
            double rollDelta = -AstronomyMath.ToRad(0.0928849189990871);//
            // double rollDelta = getRollCorrection(h, velo, w, b, pitchAngle);
            Console.WriteLine("{0} рад - рассчитаная поправка по крену. В градусах - {1}", rollDelta, AstronomyMath.ToDegrees(rollDelta));
            var resTestPoint = LanePos.getSurfacePoint(point, 0 + rollDelta, pitchAngle);

            // расстояние между точкой в надир и точкой с тангажом
            double dist = GeoPoint.DistanceOverSurface(nadirPoint, pointPitch);
            double deltaTime = Math.Sign(pitchAngle) * dist / velo; // время, которое потребуется для преодаления dist с текущей скоростью.
            Console.WriteLine("Упреждение по времени = {0} секунд ", deltaTime);
            Console.WriteLine("Упреждение по расстоянию = {0} км ", dist * Astronomy.Constants.EarthRadius);
            DateTime time_2 = point.Time.AddSeconds(deltaTime);
            TrajectoryPoint point_2 = trajectory.GetPoint(time_2); // получим точку траектории, отстающую от начальной на deltaTime
            var nadirPoint_2 = LanePos.getSurfacePoint(point_2, 0, 0);

            Console.WriteLine("{0} км - расстояние между p2 и точкой, полученной с рассчитанным упреждением по крену и начальным тангажем ",
                 GeoPoint.DistanceOverSurface(resTestPoint, nadirPoint_2) * Astronomy.Constants.EarthRadius);

            Console.WriteLine("{0} км - расстояние между p2 и pitchPoint",
                 GeoPoint.DistanceOverSurface(pointPitch, nadirPoint_2) * Astronomy.Constants.EarthRadius);

            polygons.Add(getPointPolygon(nadirPoint));
            //polygons.Add(getPointPolygon(pointPitch));
            //polygons.Add(getPointPolygon(nadirPoint_2));
            polygons.Add(getPointPolygon(resTestPoint));

        }


        public void test_roll_correction()
        {
            test_roll_correction_noroll();
            return;

            //double rollAngleDegree = 30;
            double rollAngle = 0;// AstronomyMath.ToRad(30);
            double pitchAngle = AstronomyMath.ToRad(30);
            double viewAngle = AstronomyMath.ToRad(1);
            //  SatLane strip = new SatLane(trajectory, rollAngle, viewAngle, readStep: 1, polygonStep:  OptimalChain.Constants.stripPolygonStep);

            Astronomy.Trajectory trajectory = DatParser.getTrajectoryFromDatFile(DatFileName, new DateTime(2000, 1, 1), new DateTime(2020, 1, 1));

            int count = trajectory.Count;
            var points = trajectory.Points;

            double minDist = 10000000000;
            int pindd = 0;
            for (int ii = 0; ii < count; ii++)
            {
                var trp = trajectory.Points[ii];
                var cur_dist = GeoPoint.DistanceOverSurface(trp.Position.ToVector(), GeoPoint.ToCartesian(new GeoPoint(0, 0), Astronomy.Constants.EarthRadius));
                if (minDist > cur_dist)
                {
                    minDist = cur_dist;
                    pindd = ii;
                }
            }
            Console.WriteLine("pind = {0}", pindd);

            var point = points[514];

            // начальные данные
            Vector3D pointRoll = LanePos.getSurfacePoint(point, rollAngle, 0);
            Vector3D pointRollPitch = LanePos.getSurfacePoint(point, rollAngle, pitchAngle);
            double h = point.Position.ToVector().Length - Astronomy.Constants.EarthRadius;
            double w = OptimalChain.Constants.earthRotSpeed;
            double b = AstronomyMath.ToRad(GeoPoint.FromCartesian(point.Position.ToVector()).Latitude);
            double velo = point.Velocity.Length / (Astronomy.Constants.EarthRadius + h);

            polygons.Add(getPointPolygon(point.Position.ToVector()));
            polygons.Add(getPointPolygon(pointRoll));
            polygons.Add(getPointPolygon(LanePos.getSurfacePoint(point, 0, pitchAngle)));
            polygons.Add(getPointPolygon(pointRollPitch));

            test_vectors(-point.Position.ToVector(),
                 LanePos.getDirectionVector_TEST(point, rollAngle, 0)
                , LanePos.getDirectionVector_TEST(point, rollAngle, pitchAngle)
                , LanePos.getDirectionVector_TEST(point, 0, pitchAngle));


            Vector3D pp1 = LanePos.getDirectionVector_TEST(point, 0, pitchAngle);
            Vector3D pp2 = LanePos.getDirectionVector_TEST(point, rollAngle, pitchAngle);

            Console.WriteLine("pp1 = {0}", pp1);
            Console.WriteLine("pp2 = {0}", pp2);
            Console.WriteLine("angle_12 = {0}; ", Vector3D.AngleBetween(pp1, pp2));
            Console.WriteLine("rollAngle = {0}; ", AstronomyMath.ToDegrees(rollAngle));

            //polygons.Add(getPointPolygon(GeoPoint.ToCartesian(new GeoPoint(0, 0), 1)));

            //Vector3D pp1 = LanePos.getDirectionVector(point, AstronomyMath.ToRad(2), AstronomyMath.ToRad(13));
            //Vector3D pp2 = LanePos.getDirectionVector_TEST(point, AstronomyMath.ToRad(2), AstronomyMath.ToRad(13));

            //Console.WriteLine("pp1 = {0}", pp1);
            //Console.WriteLine("pp2 = {0}", pp2);         

            // расчитаем поправку по крену
            double rollDelta = getRollCorrection(h, velo, w, b, pitchAngle);
            Console.WriteLine("{0} рад - рассчитаная поправка по крену. В градусах - {1}", rollDelta, AstronomyMath.ToDegrees(rollDelta));

            // точка, полученная из начальной с учетом поправки по крену
            Vector3D resPointRollPitch = LanePos.getSurfacePoint(point, rollAngle + rollDelta, pitchAngle);

            // расстояние между точкой, взятой только с креном, и точкой, взятой и с креном и с тангажом 
            double dist = GeoPoint.DistanceOverSurface(pointRoll, pointRollPitch);
            double deltaTime = Math.Sign(pitchAngle) * dist / velo; // время, которое потребуется для преодаления dist с текущей скоростью.
            Console.WriteLine("Упреждение по времени = {0} секунд ", deltaTime);
            Console.WriteLine("Упреждение по расстоянию = {0} км ", dist * Astronomy.Constants.EarthRadius);
            DateTime pre_time = point.Time.AddSeconds(deltaTime);
            TrajectoryPoint pre_point = trajectory.GetPoint(pre_time); // получим точку траектории, отстающую от начальной на deltaTime

            Vector3D preRollPoint = LanePos.getSurfacePoint(pre_point, rollAngle, 0);
            double dist_roll = GeoPoint.DistanceOverSurface(preRollPoint, resPointRollPitch);

            Console.WriteLine("{0} км - расстояние до точки, в которую хотим попасть (должно быить близко к 0)", dist_roll * Astronomy.Constants.EarthRadius);

            Console.WriteLine("{0} км - расстояние между точкой с тангажом и креном без упреждений и  точкой  c креном (полученной с упреждением по времени)",
                GeoPoint.DistanceOverSurface(pointRollPitch, preRollPoint) * Astronomy.Constants.EarthRadius);

            Console.WriteLine("{0} км - расстояние между точкой с креном и тангажом и точкой, полученной при помощи упреждения по крену ",
                GeoPoint.DistanceOverSurface(pointRollPitch, resPointRollPitch) * Astronomy.Constants.EarthRadius);

            //double dist_roll = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(point.Position.ToVector()), GeoPoint.FromCartesian(pre_point.Position.ToVector()));
            //Console.WriteLine("dist_roll = {0}", dist_roll);

            //LanePos pos = new LanePos(point, viewAngle, rollAngle);
            // captureLanes.Add(strip15);
        }

        public void testGetSegment()
        {
            double rollAngle = -0.78539816339744828;
            double viewAngle = OptimalChain.Constants.camera_angle;
            var trajectory = DatParser.getTrajectoryFromDatFile("trajectory_1day.dat", new DateTime(2000, 1, 1), new DateTime(2020, 1, 1));
            SatLane viewLane = new SatLane(trajectory, rollAngle, 0, viewAngle, polygonStep: OptimalChain.Constants.stripPolygonStep);

            DateTime dt1 = DateTime.Parse("12.03.2015 19:11:38");
            DateTime dt2 = DateTime.Parse("12.03.2015 19:17:27");

            var pol = viewLane.getSegment(dt1, dt2);

            var str = pol.ToWtk();
        }

        void testbd()
        {
            string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
            DBTables.DataFetcher fetcher = new DBTables.DataFetcher(managerDB);

            DateTime dt1 = DateTime.Parse("12.03.2012 19:11:38");
            DateTime dt2 = DateTime.Parse("12.03.2015 19:17:27");
            DateTime dtx = DateTime.Parse("13.07.2014 0:57:00");
            // List<DBTables.SpaceTime> points = fetcher.GetPositionSat(dt1, dt2);
            TrajectoryPoint? point = fetcher.GetPositionSat(dtx);

            int i = 4;
        }


        public void testViewPolygon()
        {
            GeoPoint gppoint = new GeoPoint(0, 0);
            double h = 725;
            Vector3D position = GeoPoint.ToCartesian(gppoint, Astronomy.Constants.EarthRadius + h);// new Vector3D(6363.5108478, -1786.1444021, 2561.4350056); 
            //Vector3D eDirVect = new Vector3D(-position.X, -position.Y, -position.Z);
            Vector3D velo = new Vector3D(0, 0, 1);
            TrajectoryPoint point = new TrajectoryPoint(new DateTime(), position.ToPoint(), velo);
            double rollAngle = AstronomyMath.ToRad(0);
            double pitchAngle = AstronomyMath.ToRad(0);
            double viewAngle = AstronomyMath.ToRad(20);

            Vector3D dir = LanePos.getDirectionVector(point, rollAngle, pitchAngle);
            var pol1 = Routines.getViewPolygon(point, dir, viewAngle);
            //  Console.WriteLine(pol1.ToWtk());
            //  polygons.Add(pol1);
            //     polygons.Add(getPointPolygon(position, 6));
            // var p = Routines.SphereVectIntersect(dir, point.Position, Astronomy.Constants.EarthRadius);
            // polygons.Add(getPointPolygon(p, 10));
            //  polygons.Add(new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))"));
        }


        public void testGetSoenPolygon()
        {
            string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
            DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);
            DBTables.DataFetcher fetcher = new DBTables.DataFetcher(manager);
            DateTime dtx = DateTime.Parse("13.07.2014 0:57:00");
            string wkt = Sessions.getSOENViewPolygon(dtx, rollAngle: 0, pitchAngle: 0, duration: 1 * 60 * 1000, managerDB: manager);
            Console.WriteLine(wkt);

            //Polygon pol = new Polygon(wkt);
            //polygons.Add(pol);

            //string wkt = "POLYGON ((143.31151656322 21.1384733467903,143.325989824923 21.1551503774817,143.34046584544 21.1718254813315,143.35494500364 21.1884990899768,143.369427678717 21.2051716348972,143.383914250232 21.2218435474621,143.312353550372 21.2758490005165,143.330248453101 21.2623504694091,143.34813989278 21.2488502827733,143.366028336297 21.2353480917981,143.29445471734 21.2893462247353,143.279971857154 21.2726665055836,143.265492781527 21.2559862452807,143.251017111003 21.2393050122197,143.236544466485 21.2226223746932,143.222074469189 21.2059379008455,143.239969981605 21.1924485189598,143.257861480248 21.1789577211096,143.275749432268 21.1654651588919,143.293634304438 21.1519704837213,143.31151656322 21.1384733467903))";
            // polygons.Add(new Polygon("POLYGON ((143.31151656322 21.1384733467903,143.325989824923 21.1551503774817,143.34046584544 21.1718254813315,143.35494500364 21.1884990899768,143.369427678717 21.2051716348972,143.383914250232 21.2218435474621,143.312353550372 21.2758490005165,143.330248453101 21.2623504694091,143.34813989278 21.2488502827733,143.366028336297 21.2353480917981,143.29445471734 21.2893462247353,143.279971857154 21.2726665055836,143.265492781527 21.2559862452807,143.251017111003 21.2393050122197,143.236544466485 21.2226223746932,143.222074469189 21.2059379008455,143.239969981605 21.1924485189598,143.257861480248 21.1789577211096,143.275749432268 21.1654651588919,143.293634304438 21.1519704837213,143.31151656322 21.1384733467903))"));
            // polygons.Add(new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))"));
        }


        public void sdfsdf()
        {
            string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
            DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);
            DBTables.DataFetcher fetcher = new DBTables.DataFetcher(manager);
            DateTime dtx = DateTime.Parse("01.01.2019 0:57:00");
            DateTime dtx2 = DateTime.Parse("13.02.2019 3:57:00");

            List<CommunicationZoneMNKPOI> zones;
            Sessions.getMNKPOICommunicationZones(manager, dtx, dtx2, out zones);

            List<CommunicationSession> sessions = Sessions.createCommunicationSessions(dtx, dtx2, manager);
            int i = 3;
        }

        public void CreateCaptureIntervals()
        {
            sdfsdf();
            return;


            ////////  testViewPolygon(); 

            return;

            //polygons.Add(new Polygon("POLYGON((-315.14378163320714 -1.645382936563152, -306.1789378832072 9.73302071251409, -341.3351878832072 29.937923070513676, -351.70628163320714 8.344268391587661, -315.14378163320714 -1.645382936563152))"));
            polygons.Add(new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))"));
            polygons.Add(new Polygon("POLYGON ((175.953568029491 2.93951720723808,177.053149762516 -4.04109892624247,-175.953568029491 -2.93951720723808,-177.053149762516 4.04109892624247,175.953568029491 2.93951720723808))"));
            return;
            //      polygons.Add(new Polygon("POLYGON ((0 0, 0 4, -4 4, -4 0, 0 0))"));

            //      polygons.Add(new Polygon("POLYGON ((-14 22, -18 26, -22 22, -18 18, -14 22))"));

            //      polygons.Add(new Polygon("POLYGON ((-24 16, -12 28, -16 32, -28 20, -24 16))"));

            //     polygons.Add(new Polygon("POLYGON ((-29 27, -27 25, -21 31, -23 33 , -29 27))"));

            //   return;

            /*
            pol1 = new Polygon(new List<Vector3D> ()
            {
            new Vector3D(0.459967485357843, 0.732720228700573, -0.50154858076418),
            new Vector3D(0.104824261623476, 0.727253223684617, -0.678317494109819 ),
            new Vector3D(0.0652235934480991, 0.839070407591418, -0.540098818700883),
            new Vector3D( 0.400136497206795, 0.839672576696237, -0.367206682329419),            
            });

            pol2 = new Polygon(new List<Vector3D>()
            {
            new Vector3D(0.459967485357843, 0.732720228700573, -0.50154858076418),
            new Vector3D(0.104824261623476, 0.727253223684617, -0.678317494109819),
            new Vector3D(0.0124341950010332, 0.802304028997096, -0.596786088854041),
            new Vector3D(0.362681427636791, 0.820547827463648, -0.441773069450851),            
            });


            pol3 = new Polygon(new List<Vector3D>()
            {
            new Vector3D(0.417782970887108, 0.73168620575953, -0.538602530200114),
            new Vector3D(0.0548093251924261, 0.71261626919262, -0.699409744537451),
            new Vector3D(0.0652235934480991, 0.839070407591418, -0.540098818700883),
            new Vector3D( 0.400136497206795, 0.839672576696237, -0.367206682329419),            
            });

            return;*/

            double rollAngle = AstronomyMath.ToRad(0); // -0.78539816339744828;
            double viewAngle = AstronomyMath.ToRad(90);

            Astronomy.Trajectory trajectory;
            try
            {
                trajectory = DatParser.getTrajectoryFromDatFile(DatFileName, new DateTime(2000, 1, 1), new DateTime(2020, 1, 1));
            }
            catch (FileNotFoundException exc)
            {
                // MessageBox.Show("The specified file does not exist!", "Error");
                return;
            }
            catch (ArgumentException exc)
            {
                // MessageBox.Show("The trajectory data in file is incorrect!", "Error");
                return;
            }

            SatLane strip15 = new SatLane(trajectory, rollAngle, 0, viewAngle, polygonStep: OptimalChain.Constants.stripPolygonStep);

            captureLanes.Add(strip15);
        }

        public void test_isRequestFeasible()
        {
           //// if (curRequest == null)
               // return;
            RequestParams reqparams = new RequestParams();
            reqparams.id = 1;

            ///12.03.2015
            //  по 
            //14.03.2015

            DateTime dt1 = new DateTime(2019, 1, 5);
            DateTime dt2 = new DateTime(2019, 1, 6);

            //DateTime dt1 = new DateTime(2019, 1, 4);
            //DateTime dt2 = new DateTime(2025, 3, 14);


            //Polygon pol = new Polygon("POLYGON ((-126.734018422276 -22.0749545024952,-130.817004196723 -24.9951369749654,-127.160780596832 -29.8248985732365,-120.731159883255 -28.8811236147716,-119.107074097174 -26.7711150394608,-119.002845016367 -25.8737550177619,-119.34506718841 -24.6296253187895,-119.70825315869 -24.0675537068877,-122.608344032093 -22.2398046264519,-126.734018422276 -22.0749545024952))");
            
            reqparams.timeFrom = dt1; // new DateTime(2000, 1, 1);
            reqparams.timeTo = dt2; // new DateTime(2020, 1, 1);            
            reqparams.priority = 1;
            reqparams.minCoverPerc = 0.4;
            reqparams.wktPolygon = "POLYGON((78.58566289767623 17.130947926520477, 78.74633794650434 17.252957595077618, 78.8424683175981 17.44302666622589, 78.70376592501998 17.59363047055203, 78.52249151095748 17.639441559013832, 78.32611089572309 17.61195630153543, 78.29589849337934 17.602793618503057, 78.27941900119185 17.304098970796602, 78.58566289767623 17.130947926520477))";// pol.ToWtk();            
            reqparams.Max_SOEN_anlge = AstronomyMath.ToRad(45);

            string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);

            double isfes;
            List<CaptureConf> possibleConfs;
            Sessions.isRequestFeasible(reqparams, dt1, dt2, managerDB, out isfes, out possibleConfs);
            Console.WriteLine(isfes);
        }

        public void testPMI()
        {
            string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);


            DateTime dt1 = new DateTime(2019, 1, 4);
            DateTime dt2 = new DateTime(2019, 1, 8);

            //SatLane strip = new SatLane(trajectory, 0, 0, viewAngle);
            //captureLanes.Add(strip);


            string wkt1 = Sessions.getSOENViewPolygon(dt1, rollAngle: 0, pitchAngle: 0, duration: 260, managerDB: managerDB, isCoridor: true);
            string wkt2 = Sessions.getSOENViewPolygon(dt1, rollAngle: Math.PI/6 , pitchAngle: 0, duration: 260, managerDB: managerDB, isCoridor: true);


            polygons.Add(new Polygon(wkt1));
            polygons.Add(new Polygon(wkt2));


        }
 
        public IList<CaptureConf> test_getCaptureConfArray()
        {
            DateTime dt1 = new DateTime(2019, 2, 1, 0, 00, 00);
            DateTime dt2 = new DateTime(2019, 2, 8, 0, 00, 00);

            string cs = "Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER";
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
 
            DateTime start = DateTime.Now;
          

            List<RequestParams> requests = new List<RequestParams>();

            polygons.Clear(); 

            polygons.Add(new Polygon("POLYGON((-43.04908666268235 -3.044447530220779, -43.53966399220003 -4.8753070490222115, -43.435288337317644 -4.903274421457425, -42.94471100779996 -3.072414902655993, -43.04908666268235 -3.044447530220779))"));
            polygons.Add(new Polygon("POLYGON((-50.31716140529085 -0.5747537972736154, -50.335880302737785 -0.6809139400476596, -50.22971359470917 -0.6996339951261064, -50.21099469726223 -0.5934738523520622, -50.31716140529085 -0.5747537972736154))"));
            polygons.Add(new Polygon("POLYGON((-50.757977960298355 -4.601211337347138, -50.79484699096549 -4.702508166582305, -50.66780328970164 -4.748748292293342, -50.63093425903449 -4.647451463058189, -50.757977960298355 -4.601211337347138))"));
            polygons.Add(new Polygon("POLYGON((-51.53707293487392 -10.95804247857238, -51.57394196554107 -11.059339307807562, -51.47073956512608 -11.096901909663444, -51.43387053445894 -10.995605080428277, -51.53707293487392 -10.95804247857238))"));
            polygons.Add(new Polygon("POLYGON((-50.0445858914435 -17.92960004489511, -50.08145492211064 -18.030896874130264, -49.974945358556525 -18.069663184928643, -49.93807632788937 -17.96836635569349, -50.0445858914435 -17.92960004489511))"));
            polygons.Add(new Polygon("POLYGON((-51.29401917952477 -27.05982002977042, -51.4885101436886 -27.17210944028536, -51.362230820475226 -27.390831644036325, -51.16773985631141 -27.2785422335214, -51.29401917952477 -27.05982002977042))"));
            polygons.Add(new Polygon("POLYGON((-51.01542607825896 -27.806425118514035, -51.0522951089261 -27.90772194774918, -50.937698921741074 -27.949431548844935, -50.900829891073926 -27.84813471960979, -51.01542607825896 -27.806425118514035))"));
            polygons.Add(new Polygon("POLYGON((-52.76928527125471 -32.0626588950123, -52.81484260451908 -32.16035691147769, -52.69946472874528 -32.21415849853267, -52.653907395480914 -32.116460482067275, -52.76928527125471 -32.0626588950123))"));
            polygons.Add(new Polygon("POLYGON((-76.05760560063729 -0.3044491885171112, -76.09447463130444 -0.40574601775226427, -75.9931756493627 -0.4426158319405289, -75.95630661869555 -0.34131900270537585, -76.05760560063729 -0.3044491885171112))"));
            polygons.Add(new Polygon("POLYGON((-75.02261245242727 -7.145101806681566, -75.02261245242727 -7.252899640775922, -74.4793406725727 -7.252899640775922, -74.4793406725727 -7.145101806681566, -75.02261245242727 -7.145101806681566))"));
            polygons.Add(new Polygon("POLYGON((-71.50374721803709 -16.36160036124477, -71.61049597102021 -16.34659780239707, -71.62613559446291 -16.457879505510718, -71.51938684147979 -16.472882064358416, -71.50374721803709 -16.36160036124477))"));
            polygons.Add(new Polygon("POLYGON((-70.341356382513 -18.260661169175805, -70.41816686306956 -18.471696230082372, -70.195752992487 -18.55264825866236, -70.11894251193044 -18.341613197755777, -70.341356382513 -18.260661169175805))"));
            polygons.Add(new Polygon("POLYGON((-70.10729191358107 -22.690346453785182, -70.1441609442482 -22.791643283020335, -70.03430964891893 -22.831625884715763, -69.9974406182518 -22.73032905548061, -70.10729191358107 -22.690346453785182))"));
            polygons.Add(new Polygon("POLYGON((-73.6440945444437 -37.57108600065295, -73.66281344189062 -37.67724614342698, -73.5287570180563 -37.70088390788626, -73.51003812060938 -37.59472376511223, -73.6440945444437 -37.57108600065295))"));
            polygons.Add(new Polygon("POLYGON((-74.24014189543877 -41.28663429970019, -74.27701092610593 -41.38793112893536, -73.2622995108112 -41.75725588047276, -73.22543048014404 -41.655959051237595, -74.24014189543877 -41.28663429970019))"));
            polygons.Add(new Polygon("POLYGON((-74.02992281372227 -41.50300106334383, -74.21426796705799 -42.00948520951961, -73.67027249877776 -42.2074833675493, -73.48592734544204 -41.700999221373536, -74.02992281372227 -41.50300106334383))"));
            polygons.Add(new Polygon("POLYGON((-73.8783475012522 -42.38835527514226, -73.91521653191933 -42.48965210437743, -73.7779024987478 -42.53963032519891, -73.74103346808067 -42.438333495963754, -73.8783475012522 -42.38835527514226))"));
            polygons.Add(new Polygon("POLYGON((-73.13456455512787 -45.17485422757945, -73.17143358579503 -45.27615105681462, -73.02754481987213 -45.328522284655854, -72.99067578920497 -45.22722545542068, -73.13456455512787 -45.17485422757945))"));
            polygons.Add(new Polygon("POLYGON((-73.62010006363346 -46.79009087295539, -73.65696909430062 -46.891387702190556, -73.50880618636654 -46.945314590500885, -73.47193715569938 -46.84401776126571, -73.62010006363346 -46.79009087295539))"));
            polygons.Add(new Polygon("POLYGON((-74.06313603850693 -49.21755569317754, -74.10000506917409 -49.31885252241271, -73.9446764614931 -49.375387512138595, -73.90780743082594 -49.27409068290343, -74.06313603850693 -49.21755569317754))"));
 
            int id = 310;
            foreach (var pol in polygons)
            {
                RequestParams reqparams = new RequestParams();
                reqparams.id = id;
                reqparams.timeFrom = new DateTime(2019, 2, 01, 0, 00, 00);
                reqparams.timeTo = new DateTime(2019, 2, 25, 0, 00, 00);
                reqparams.shootingType = 0;
                reqparams.minCoverPerc = 70;
                reqparams.Max_SOEN_anlge = 0.9599310885968813;//AstronomyMath.ToRad(45);
                reqparams.wktPolygon = pol.ToWtk();
                reqparams.albedo = 0;
                reqparams.compression = 0;
                requests.Add(reqparams);
                id++;
            }


            requests[0].compression = 2;
            requests[1].compression = 1;
            requests[2].compression = 1;
            requests[3].compression = 1;
            requests[4].compression = 1;
            requests[5].compression = 9;
            requests[6].compression = 1;
            requests[7].compression = 8;
            requests[8].compression = 1;
            requests[9].compression = 10;
            requests[10].compression = 1;
            requests[11].compression = 10;
            requests[12].compression = 5;
            requests[13].compression = 10;
            requests[14].compression = 1;
            requests[15].compression = 10;
            requests[16].compression = 6;
            requests[17].compression = 2;
            requests[18].compression = 2;
            requests[19].compression = 1;             
            
            List<Tuple<DateTime, DateTime>> silenceRanges = new List<Tuple<DateTime, DateTime>>();
            List<Tuple<DateTime, DateTime>> inactivityRanges = new List<Tuple<DateTime, DateTime>>();
            
            List<MPZ> mpzArray;
            List<CommunicationSession> sessions;

            Order order = new Order();
            order.captured = new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))");
            order.intersection_coeff = 0.1;
            order.request = new RequestParams();
            order.request.priority = 1;
            order.request.timeFrom = dt1;
            order.request.timeTo = dt2;
            order.request.wktPolygon = "POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))";
            order.request.minCoverPerc = 0.4;
            order.request.Max_SOEN_anlge = AstronomyMath.ToRad(45);
            List<Order> orders = new List<Order>() { order };

            CaptureConf cc = new CaptureConf(dt1, dt2, 0.1, orders, 1, null);
            StaticConf sc = cc.DefaultStaticConf();
            RouteParams routeParam = new RouteParams(sc);
            routeParam.id = 0;
            routeParam.start = dt1;
            routeParam.end = dt2;
            routeParam.File_Size = 1000;
            routeParam.binded_route = new Tuple<int, int>(1, 1);
            // double timedrop = routeParam.getDropTime();

            RouteMPZ routempz = new RouteMPZ(routeParam, managerDB) { NPZ = 0, Nroute = 0 };
            List<RouteMPZ> routesToDrop = new List<RouteMPZ>();
        //    routesToDrop.Add(routempz);

            CaptureConf ccToDelete = new CaptureConf(new DateTime(2019, 1, 4), new DateTime(2019, 1, 5), 0.1, orders, 2, null);
            StaticConf scToDelete = ccToDelete.DefaultStaticConf();
            RouteParams routeParamtoDelete = new RouteParams(scToDelete);
            routeParamtoDelete.id = 0;
            routeParamtoDelete.start = new DateTime(2019, 1, 4);
            routeParamtoDelete.end = new DateTime(2019, 1, 5);
            routeParamtoDelete.File_Size = 1000;
            routeParamtoDelete.binded_route = new Tuple<int, int>(1, 1);
            RouteMPZ routempzToDelete = new RouteMPZ(routeParamtoDelete, managerDB) { NPZ = 0, Nroute = 0 };

            List<RouteMPZ> routesToDelete = new List<RouteMPZ>();
          //  routesToDelete.Add(routempzToDelete);

            

        //    string s = "POLYGON((126.62833380556265 -14.265775772684364, 126.61297170945132 -14.307982784865686, 126.6565272839827 -14.323835717531452, 126.671889380094 -14.281628705350116, 126.62833380556265 -14.265775772684364))";
          //  RequestParams rp = new RequestParams(i : 301, p: 3, d1: dt1, d2: dt2, max_a: 55 * Math.PI / 180, min_p: 50, max_s_a: 10, min_s_a: 90, polygon: s, alb : 0.36, comp : 10, sT : 1);
            //12.03.2015 по 14.03.2015
            // Sessions.getMPZArray(new List<RequestParams> {rp}, dt1, dt2
            Sessions.getMPZArray(requests, dt1, dt2
                , silenceRanges
                , inactivityRanges
                , routesToDrop
                , routesToDelete
                , managerDB
                , 356
                , out mpzArray
                , out sessions);

            DateTime endd = DateTime.Now;
            Console.WriteLine("total time = " + (endd - start).TotalSeconds.ToString());
            
            foreach (var mpz in mpzArray)
            {
               // MPZ mpz = new MPZ(mpz)
                Console.WriteLine("mpz.Header.NPZ = {0}", mpz.Header.NPZ);
                foreach (var route in mpz.Parameters.routes)
                {
                    if (route.ShootingConf != null)
                    {                        
                        string wkt = route.ShootingConf.wktPolygon;
                        ///Console.WriteLine("wkt = {0}", wkt);
                        captureIntervals.Add(new Polygon(wkt));
                    }
                }
            }
            //foreach (var mpz in mpzArray)
            //{ 
            //    Console.WriteLine("mpz.Header.NPZ = {0}", mpz.Header.NPZ);
            //}
            //Console.WriteLine("0-------------------------------------");

            foreach (var mpz in mpzArray)
            { 
                foreach (var route in mpz.Routes)
                {
                    
                    if (route.RegimeType != RegimeTypes.NP)
                    {
                        Console.WriteLine("route = {0}", route.RegimeType);
                    }
                }
            }


            Console.WriteLine("res.Count = {0}", mpzArray.Count());
            return new List<CaptureConf>();

        }

        public bool RegionCanBeCaptured { get; private set; }
        public double SatelliteRoll { get; private set; }
        public double SatellitePitch { get; private set; }
        public double SatelliteYaw { get; private set; }

        public ICommand AddRequestCmd { get { return addRequestCmd; } }
        public ICommand RemoveRequestCmd { get { return removeRequestCmd; } }
        public ICommand AddPointCmd { get { return addPointCmd; } }
        public ICommand LoadWTK { get { return loadWTK; } }
        public ICommand RemovePointCmd { get { return removePointCmd; } }
        //public ICommand VerifyIfRegionCanBeSeenCmd { get { return verifyIfRegionCanBeSeenCmd; } }
    }

    public class DegreeToRadianConverter : IValueConverter
    {
        public object ConvertBack(object value,
                              Type targetType,
                              object parameter,
                              System.Globalization.CultureInfo culture)
        {
            return Double.Parse(value.ToString(), culture) / 180 * Math.PI;
        }

        public object Convert(object value,
                                  Type targetType,
                                  object parameter,
                                  System.Globalization.CultureInfo culture)
        {
            return ((double)value * 180 / Math.PI).ToString(culture);
        }
    }

    public class DoubleInRange : ValidationRule
    {
        private double _min;
        private double _max;

        public DoubleInRange()
        {
        }

        public double Min
        {
            get { return _min; }
            set { _min = value; }
        }

        public double Max
        {
            get { return _max; }
            set { _max = value; }
        }

        public override ValidationResult Validate(object value,
            System.Globalization.CultureInfo cultureInfo)
        {
            double val = 0;

            try
            {
                if (((string)value).Length > 0)
                    val = Double.Parse((String)value, cultureInfo);
            }
            catch (Exception e)
            {
                return new ValidationResult(false, "Illegal characters or " + e.Message);
            }

            if ((Min < Max) && ((val < Min) || (val > Max)) || (val < Min) && (Min > Max))
            {
                return new ValidationResult(false,
                  "Must be in range: " + Min + " - " + Max + ".");
            }
            else
            {
                return new ValidationResult(true, null);
            }
        }
    }

    public class CanComputeToTextConverter : IValueConverter
    {
        public object ConvertBack(object value,
                              Type targetType,
                              object parameter,
                              System.Globalization.CultureInfo culture)
        {
            return true;
        }

        public object Convert(object value,
                                  Type targetType,
                                  object parameter,
                                  System.Globalization.CultureInfo culture)
        {
            if (bool.Parse(value.ToString()))
            {
                return "Region can be captured?";
            }
            else
            {
                return "Computing...";
            }
        }
    }

    public class DelegateCommand : ICommand
    {
        private readonly Predicate<object> _canExecute;
        private readonly Action<object> _execute;

        public event EventHandler CanExecuteChanged;

        public DelegateCommand(Action<object> execute)
            : this(execute, null)
        {
        }

        public DelegateCommand(Action<object> execute,
                       Predicate<object> canExecute)
        {
            _execute = execute;
            _canExecute = canExecute;
        }

        public bool CanExecute(object parameter)
        {
            if (_canExecute == null)
            {
                return true;
            }

            return _canExecute(parameter);
        }

        public void Execute(object parameter)
        {
            _execute(parameter);
        }

        public void RaiseCanExecuteChanged()
        {
            if (CanExecuteChanged != null)
            {
                CanExecuteChanged(this, EventArgs.Empty);
            }
        }
    }
}
