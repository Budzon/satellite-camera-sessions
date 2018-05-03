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
using DBTables;

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

            SatLane strip = new SatLane(trajectory, 0, viewAngle);
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
           // double rollDelta = -AstronomyMath.ToRad(0.0928849189990871);//
            double rollDelta = getRollCorrection(h, velo, w, b, pitchAngle);
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
                 LanePos.getDirectionVector(point, rollAngle, 0)
                , LanePos.getDirectionVector(point, rollAngle, pitchAngle)
                , LanePos.getDirectionVector(point, 0, pitchAngle));

            Vector3D pp1 = LanePos.getDirectionVector(point, 0, pitchAngle);
            Vector3D pp2 = LanePos.getDirectionVector(point, rollAngle, pitchAngle);

            Console.WriteLine("pp1 = {0}", pp1);
            Console.WriteLine("pp2 = {0}", pp2);
            Console.WriteLine("angle_12 = {0}; " , Vector3D.AngleBetween(pp1, pp2));
            Console.WriteLine("rollAngle = {0}; ", AstronomyMath.ToDegrees(rollAngle));

            //polygons.Add(getPointPolygon(GeoPoint.ToCartesian(new GeoPoint(0, 0), 1)));

            //Vector3D pp1 = LanePos.getDirectionVector(point, AstronomyMath.ToRad(2), AstronomyMath.ToRad(13));
            //Vector3D pp2 = LanePos.getDirectionVector(point, AstronomyMath.ToRad(2), AstronomyMath.ToRad(13));

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
            SatLane viewLane = new SatLane(trajectory, rollAngle, viewAngle);

            DateTime dt1 = DateTime.Parse("12.03.2015 19:11:38");
            DateTime dt2 = DateTime.Parse("12.03.2015 19:17:27");

            var pol = viewLane.getSegment(dt1, dt2);

            var str = pol.ToWtk();
        }

        void testbd()
        {
                            string cs = System.IO.File.ReadLines("DBstring.conf").First();
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

            SatelliteCoordinates kaPos = new SatelliteCoordinates(point, rollAngle, pitchAngle);
            var pol1 = kaPos.ViewPolygon;
            //  Console.WriteLine(pol1.ToWtk());
            //  polygons.Add(pol1);
            //     polygons.Add(getPointPolygon(position, 6));
            // var p = Routines.SphereVectIntersect(dir, point.Position, Astronomy.Constants.EarthRadius);
            // polygons.Add(getPointPolygon(p, 10));
            //  polygons.Add(new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))"));
        }


        public void testGetSoenPolygon()
        {
                            string cs = System.IO.File.ReadLines("DBstring.conf").First();
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
                            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);
            DBTables.DataFetcher fetcher = new DBTables.DataFetcher(manager);
            DateTime dtx = DateTime.Parse("01.01.2019 0:57:00");
            DateTime dtx2 = DateTime.Parse("13.02.2019 3:57:00");

            List<CommunicationZoneMNKPOI> zones;
            CommunicationZone.getMNKPOICommunicationZones(manager, dtx, dtx2, out zones);

            List<CommunicationSession> sessions = CommunicationSession.createCommunicationSessions(dtx, dtx2, manager);
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

            SatLane strip15 = new SatLane(trajectory, rollAngle, viewAngle);

            captureLanes.Add(strip15);
        }




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




        void tedfdfst()
        {
            Vector3D sun = new Vector3D(-125889950.019308,50789227.1698602,-56664005.2550829);
            List<Vector3D> Apexes = new List<Vector3D>()
            {
                 new Vector3D(0.0922945601729868,0.0588161960823579,0.0217377326659687),
                 new Vector3D(0,0,0),
                 new Vector3D(-0.0923087184673386,-0.0588331312144336,-0.0217268204343814),
                 new Vector3D(0,0,0)
            };


            List<Vector3D> Verts = new List<Vector3D>()
            {
                 new Vector3D(-0.462356449436716,0.78441115091137,0.413431566272067),
                 new Vector3D(-0.463011646780311,0.789028386938173,0.403799974679951),
                 new Vector3D(-0.647633788493741,0.671395998770318,0.360274740769391),
                 new Vector3D(-0.646940323001679,0.66674411493871,0.370027706623543)
            };
            Polygon sector = new Polygon(Verts,Apexes);
            var LitAndNot = SphericalGeom.Polygon.IntersectAndSubtract(sector, SphericalGeom.Polygon.Hemisphere(sun));
            //Console.WriteLine(sector.ToWtk());


            foreach (SphericalGeom.Polygon p in LitAndNot.Item1)
                foreach (Polygon piece in p.BreakIntoLobes())
                    Console.WriteLine(piece.ToWtk());
        }


        public void test_isRequestFeasible()
        {
            //tedfdfst();
            //return;

            for (int testi = 0; testi < 2; testi++)
            {

                List<Polygon> polygons = new List<Polygon>();
                Random rand = new Random((int)DateTime.Now.Ticks);
                for (int i = 0; i < 20; i++)
                {
                    Polygon randpol = getRandomPolygon(rand, 3, 12, 2, 8);
                    polygons.Add(randpol);
                }

                string cs = System.IO.File.ReadLines("DBstring.conf").First();
                DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);

                DateTime dt1 = new DateTime(2019, 1, 5);
                DateTime dt2 = new DateTime(2019, 1, 6);

                DataFetcher fetcher = new DataFetcher(manager);
                Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

                if (trajectory.Count == 0)
                    throw new Exception("На эти даты нет траектории в БД, тест некорректный");


                foreach (var pol in polygons)
                {
                    //try
                    //{
                    RequestParams reqparams = new RequestParams(0, 1, dt1, dt2, AstronomyMath.ToRad(45), 0.4, 1, 1, pol.ToWtk());

                    double cover;
                    List<CaptureConf> output;
                    Sessions.isRequestFeasible(reqparams, dt1, dt2, manager, out cover, out output);
                    //}
                    //catch (Exception ex)
                    //{
                    //    List<string> lines = new List<string>();
                    //    Console.WriteLine("Ошибка обнаружена на следующем полигонt:");
                    //    Console.WriteLine(pol.ToWtk());
                    //    lines.Add(pol.ToWtk());                        
                    //    System.IO.File.WriteAllLines(@"badPolygons.txt", lines);
                    //    throw ex;
                    //}

                }
            }


           ////// if (curRequest == null)
           //    // return;
       
           // ///12.03.2015
           // //  по 
           // //14.03.2015

           // DateTime dt1 = new DateTime(2019, 1, 5);
           // DateTime dt2 = new DateTime(2019, 1, 6);

           // //DateTime dt1 = new DateTime(2019, 1, 4);
           // //DateTime dt2 = new DateTime(2025, 3, 14);


           // //Polygon pol = new Polygon("POLYGON ((-126.734018422276 -22.0749545024952,-130.817004196723 -24.9951369749654,-127.160780596832 -29.8248985732365,-120.731159883255 -28.8811236147716,-119.107074097174 -26.7711150394608,-119.002845016367 -25.8737550177619,-119.34506718841 -24.6296253187895,-119.70825315869 -24.0675537068877,-122.608344032093 -22.2398046264519,-126.734018422276 -22.0749545024952))");

           // RequestParams reqparams = new RequestParams(0, 1, dt1, dt2, AstronomyMath.ToRad(45), 0.4, 1, 1, 
           //     "POLYGON ((-151.392459628474 45.4924697572034,-139.546126267888 46.6789117628991,-139.7918444387 47.7999583568984,-140.750872347367 48.592427667584,-141.382703729286 48.900382352261,-142.276361509347 49.2130496481828,-151.392459628474 45.4924697572034))"
           //     );
               
            
           //                 string cs = System.IO.File.ReadLines("DBstring.conf").First();
           // DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);

 
           // double isfes;
           // List<CaptureConf> possibleConfs;
           // Sessions.isRequestFeasible(reqparams, dt1, dt2, managerDB, out isfes, out possibleConfs);
           // Console.WriteLine(isfes);
        }

        public void testPMI()
        {
            string cs = System.IO.File.ReadLines("DBstring.conf").First();
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

        public string getWKTTrajectory(Trajectory trajectory)
        {
            string res = "";
            Char separator = System.Globalization.CultureInfo.CurrentCulture.NumberFormat.CurrencyDecimalSeparator[0];
            res += "LINESTRING(\n";
            for (int i = 0; i < trajectory.Count; i++ )
            {
                TrajectoryPoint trajPoint = trajectory.Points[i];
                GeoPoint pos = GeoPoint.FromCartesian(trajPoint.Position.ToVector());
                if (i != 0)
                    res += " , ";
                res += string.Format("{0} {1}", pos.Longitude.ToString().Replace(separator, '.'), pos.Latitude.ToString().Replace(separator, '.'));
            }
            res += ")\n";
            return res;
        }

        public string getWKTLinePoints(List<Vector3D> points)
        {
            string res = "";
            Char separator = System.Globalization.CultureInfo.CurrentCulture.NumberFormat.CurrencyDecimalSeparator[0];
            res += "LINESTRING(\n";
            for (int i = 0; i < points.Count; i++)
            {
                GeoPoint pos = GeoPoint.FromCartesian(points[i]);
                if (i != 0)
                    res += " , ";
                res += string.Format("{0} {1}", pos.Longitude.ToString().Replace(separator, '.'), pos.Latitude.ToString().Replace(separator, '.'));
            }
            res += ")\n";
            return res;
        }

        public string getWKTInterpolateTrajectory(Trajectory trajectory,DateTime segmdt1, DateTime segmdt2, int step)
        {
            string res = "";
            Char separator = System.Globalization.CultureInfo.CurrentCulture.NumberFormat.CurrencyDecimalSeparator[0];
            res += "LINESTRING(\n";
            for (int t = 0; t <= (segmdt2 - segmdt1).TotalSeconds; t += step)
            {
                DateTime curDt = segmdt1.AddSeconds(t);
                TrajectoryPoint trajPoint = trajectory.GetPoint(curDt);
                GeoPoint pos = GeoPoint.FromCartesian(trajPoint.Position.ToVector());
                if (t != 0)
                    res += " , ";
                res += string.Format("{0} {1}", pos.Longitude.ToString().Replace(separator, '.'), pos.Latitude.ToString().Replace(separator, '.'));
            }
            res += ")\n";
            return res;
        }

        public string getWKTViewInterpolPolygons(Trajectory trajectory, DateTime segmdt1, DateTime segmdt2, double rollAngle, double pitchAngle, int step)
        {            
            var allPols = new List<Polygon>();
            double t = 0;
            while (true)
            {
                DateTime dtcur = segmdt1.AddSeconds(t);
                TrajectoryPoint p1 = trajectory.GetPoint(dtcur);
                SatelliteTrajectory.SatelliteCoordinates kp = new SatelliteTrajectory.SatelliteCoordinates(p1, rollAngle, pitchAngle);
                Polygon testkpvp = kp.ViewPolygon;
                allPols.Add(testkpvp);

                if (t >= (segmdt2 - segmdt1).TotalSeconds)
                    break;

                t += step;

                if (t > (segmdt2 - segmdt1).TotalSeconds)
                    t = (segmdt2 - segmdt1).TotalSeconds;
            }
            return Polygon.getMultipolFromPolygons(allPols);  
        }

        public string getWKTViewPolygons(Trajectory trajectory, DateTime segmdt1, DateTime segmdt2, double rollAngle, double pitchAngle)
        {
            var allPols = new List<Polygon>();
            foreach (var point in trajectory.Points)
            {
                if (point.Time < segmdt1)
                    continue;
                if (point.Time > segmdt2)
                    break;

                SatelliteTrajectory.SatelliteCoordinates kp = new SatelliteTrajectory.SatelliteCoordinates(point, rollAngle, pitchAngle);
                Polygon testkpvp = kp.ViewPolygon;
                allPols.Add(testkpvp);
            }
            return Polygon.getMultipolFromPolygons(allPols);
        }

        public string getWKTViewLine(Trajectory trajectory, DateTime segmdt1, DateTime segmdt2, double rollAngle, double pitchAngle, int step)
        {
            string res = "";
            Char separator = System.Globalization.CultureInfo.CurrentCulture.NumberFormat.CurrencyDecimalSeparator[0];
            res += "LINESTRING(\n";
            double t = 0; 
            while (true)
            {
                DateTime curDt = segmdt1.AddSeconds(t);
                TrajectoryPoint trajPoint = trajectory.GetPoint(curDt);
                SatelliteTrajectory.SatelliteCoordinates kp = new SatelliteTrajectory.SatelliteCoordinates(trajPoint, rollAngle, pitchAngle);
                Vector3D vp = Routines.SphereVectIntersect(kp.ViewDir, trajPoint.Position, Astronomy.Constants.EarthRadius);
                GeoPoint pos = GeoPoint.FromCartesian(vp);
                if (t != 0)
                    res += " , ";
                res += string.Format("{0} {1}", pos.Longitude.ToString().Replace(separator, '.'), pos.Latitude.ToString().Replace(separator, '.'));

                if (t >= (segmdt2 - segmdt1).TotalSeconds)
                    break;

                t += step;
                               
                if (t > (segmdt2 - segmdt1).TotalSeconds)
                    t = (segmdt2 - segmdt1).TotalSeconds; 
            }
           
            res += ")\n";
            return res;
        }

        public string getWKTStrip(Trajectory trajectory, double rollAngle, int step)
        { 
            SatLane lane = new SatLane(trajectory, rollAngle, OptimalChain.Constants.camera_angle);
            string res = Polygon.getMultipolFromPolygons(lane.Sectors.Select(sect => sect.polygon).ToList());
            return res;
        }

        public string getWKTStripSegment(Trajectory trajectory, DateTime segmdt1, DateTime segmdt2, double rollAngle)
        {
            SatLane lane = new SatLane(trajectory, rollAngle, OptimalChain.Constants.camera_angle);         
            return lane.getSegment(segmdt1, segmdt2).ToWtk();
        }


        public string getWKTViewPolygon(Trajectory trajectory, double rollAngle, double pitchAngle, DateTime dtime)
        {
            TrajectoryPoint p1 = trajectory.GetPoint(dtime);
            SatelliteTrajectory.SatelliteCoordinates kp = new SatelliteTrajectory.SatelliteCoordinates(p1, rollAngle, pitchAngle);
            return kp.ViewPolygon.ToWtk();
        }


        public void test_PlotTrajectoryAndSeveralViewPolygons()
        {
            var allPols = new List<Polygon>();
                            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
            DataFetcher fetcher = new DataFetcher(managerDB);
            double rollAngle = AstronomyMath.ToRad(45);
            double pitchAngle = AstronomyMath.ToRad(30);
            double viewAngle = OptimalChain.Constants.camera_angle;

            DateTime dt1 = new DateTime(2019, 2, 2, 0, 00, 00);
            DateTime dt2 = new DateTime(2019, 2, 2, 1, 9, 00);

            DateTime segmdt1 = new DateTime(2019, 2, 2, 0, 00, 00); 
            DateTime segmdt2 = new DateTime(2019, 2, 2, 0, 01, 00);

            Trajectory trajectory = fetcher.GetTrajectorySat(segmdt1, segmdt2);
                       
            int step = 1;

            Console.Write("GEOMETRYCOLLECTION(");

            Console.WriteLine(getWKTInterpolateTrajectory(trajectory, segmdt1, segmdt2, step));
           
            Console.Write(",");

            Console.WriteLine(getWKTViewInterpolPolygons(trajectory, segmdt1, segmdt2, rollAngle, pitchAngle, step));
            
            Console.Write(")");

        }

        public void testTrajectoryInterpolationFromDat()
        {
            double rollAngle = AstronomyMath.ToRad(45);
            double pitchAngle = AstronomyMath.ToRad(30);
            double viewAngle = OptimalChain.Constants.camera_angle;

            DateTime dt1 = new DateTime(2015, 3, 12, 0, 00, 00);
            DateTime dt2 = new DateTime(2015, 3, 12, 7, 6, 00);
            Astronomy.Trajectory trajFull = DatParser.getTrajectoryFromDatFile("trajectory_full.dat", dt1, dt2);

            TrajectoryPoint[] points = trajFull.Points;
            int count = trajFull.Count();

            List<TrajectoryPoint> points_test = new List<TrajectoryPoint>();
            for (int i = 0; i < count; i += 6)
            {
                points_test.Add(points[i]);
            }
            Trajectory testTrajectoryFull = Trajectory.Create(points_test.ToArray());

            List<SpaceTime> preTrajectory = new List<SpaceTime>();

            foreach (var p in testTrajectoryFull.Points)
                preTrajectory.Add(new SpaceTime() { Position = p.Position.ToVector(), Time = p.Time });

            Astronomy.Trajectory testTrajectory = SpaceTime.createTrajectory(preTrajectory);

           
            double maxDist = 0;
            double minDist = Astronomy.Constants.EarthRadius;
            double maxDistViewp = 0;
            double minDistViewp = Astronomy.Constants.EarthRadius;
            for (int i = 0; i < count; i ++)
            {
                DateTime dt = points[i].Time;
                TrajectoryPoint etalonPoint = points[i];
                TrajectoryPoint testPoint = testTrajectory.GetPoint(dt);
                double dist = (etalonPoint.Position - testPoint.Position).Length;
                if (dist < minDist)
                    minDist = dist;
                if (dist > maxDist)
                    maxDist = dist;

                // далее сравниваем две точки взора камеры

                SatelliteTrajectory.SatelliteCoordinates kpEtalon = new SatelliteTrajectory.SatelliteCoordinates(etalonPoint, rollAngle, pitchAngle);
                Vector3D vpEtalon = Routines.SphereVectIntersect(kpEtalon.ViewDir, etalonPoint.Position, Astronomy.Constants.EarthRadius);


                SatelliteTrajectory.SatelliteCoordinates kp = new SatelliteTrajectory.SatelliteCoordinates(testPoint, rollAngle, pitchAngle);
                Vector3D vp = Routines.SphereVectIntersect(kp.ViewDir, testPoint.Position, Astronomy.Constants.EarthRadius);

                double dist_viewp = (vpEtalon - vp).Length;// GeoPoint.DistanceOverSurface(vpEtalon, vp) * Astronomy.Constants.EarthRadius;
                if (dist_viewp < minDistViewp)
                    minDistViewp = dist_viewp;
                if (dist_viewp > maxDistViewp)
                    maxDistViewp = dist_viewp;
            }

            Console.WriteLine("minDist = {0}", minDist);
            Console.WriteLine("maxDist = {0}", maxDist);
            Console.WriteLine();
            Console.WriteLine("minDistViewp = {0}", minDistViewp);
            Console.WriteLine("maxDistViewp = {0}", maxDistViewp);
            Console.WriteLine();
        }

        public void testTrajectoryInterpolation()
        {
                            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
            DataFetcher fetcher = new DataFetcher(managerDB);
            DateTime dt1 = new DateTime(2019, 2, 1, 0, 00, 00);
            DateTime dt2 = new DateTime(2019, 2, 2, 14, 9, 00);            
            Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

            TrajectoryPoint[] points = trajectory.Points;
            int count = trajectory.Count();

            int testNum = count / 2;

            TrajectoryPoint testPoint = points[testNum];

            TrajectoryPoint[] newPoints = new TrajectoryPoint[count / 2]; // оставим каждую вторую точку

            for (int i= 0; i < count; i+=2)
            {
                int j = i/2;
                newPoints[j] = points[i];
            }

            Trajectory testTrajectory = Trajectory.Create(newPoints);

            Point3D testPos = testTrajectory.GetPosition(testPoint.Time);
            double dist = (testPos.ToVector() - testPoint.Position.ToVector()).Length;
            Console.WriteLine("расстояние между расчётной и реальной точками: {0} км", dist);
            
        }

        public void test_calculatePitchArray()
        {
            DateTime dt1 = DateTime.Parse("01.02.2019 00:00:00");
            DateTime dtMid = DateTime.Parse("01.02.2019 00:05:00");
            DateTime dt2 = DateTime.Parse("01.02.2019 00:10:00");

                            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);

            DataFetcher fetcher = new DataFetcher(managerDB);
            double rollAngle = AstronomyMath.ToRad(45);
            double pitchAngle = AstronomyMath.ToRad(30);
            double viewAngle = OptimalChain.Constants.camera_angle;


            Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);
            TrajectoryPoint pp1 = (TrajectoryPoint)fetcher.GetPositionSat(dtMid);

            List<Vector3D> pitchLine = new List<Vector3D>();
            double pitchStep = AstronomyMath.ToRad(1);
            for (double pitch = -pitchAngle; pitch <= pitchAngle; pitch += pitchStep)
            {
                Vector3D surfPoint = LanePos.getSurfacePoint(pp1, rollAngle, pitch);
                pitchLine.Add(surfPoint);
            }

            List<Vector3D> nopitchLine = new List<Vector3D>();
            foreach (var trp in trajectory)
            {
                Vector3D surfPoint = LanePos.getSurfacePoint(trp, rollAngle, 0);
                nopitchLine.Add(surfPoint);
            }

            List<Vector3D> rollCorrPitchLine = new List<Vector3D>();
            for (double pitch = -pitchAngle; pitch <= pitchAngle; pitch += pitchStep)
            {
                double height = pp1.Position.ToVector().Length - Astronomy.Constants.EarthRadius;
                double velo = pp1.Velocity.Length / (Astronomy.Constants.EarthRadius + height);
                GeoPoint kaGeoPoint = GeoPoint.FromCartesian(pp1.Position.ToVector());
                double rollCor = Sessions.getRollCorrection(height, velo, AstronomyMath.ToRad(kaGeoPoint.Latitude), pitch);

                Vector3D surfPoint = LanePos.getSurfacePoint(pp1, rollAngle + rollCor, pitch);
                rollCorrPitchLine.Add(surfPoint);
                Console.WriteLine(rollCor);
            }

            //GeoPoint p1 = new GeoPoint(-49.01265386395501, -14.562377929687498);
            //GeoPoint p2 = new GeoPoint(-49.11343354595957, -14.974365234374996);
            //Console.WriteLine("err = {0}", GeoPoint.DistanceOverSurface(p1, p2) * Astronomy.Constants.EarthRadius);




            Polygon centre = getPointPolygon(LanePos.getSurfacePoint(pp1, rollAngle, 0));

            Console.Write("GEOMETRYCOLLECTION(");
            Console.WriteLine(getWKTLinePoints(pitchLine));
            Console.WriteLine(",");
            Console.WriteLine(getWKTLinePoints(rollCorrPitchLine));
            Console.WriteLine(",");
            Console.WriteLine(getWKTLinePoints(nopitchLine));
            Console.WriteLine(",");
            Console.WriteLine(getWKTTrajectory(trajectory));
            Console.WriteLine(",");
            Console.WriteLine(Polygon.getMultipolFromPolygons(new List<Polygon>() { centre }));
            Console.WriteLine(")");
        }




        public void test_plot4getRollPitchLanePolygon()
        {

            var allPols = new List<Polygon>();
                            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
            DataFetcher fetcher = new DataFetcher(managerDB);
            double rollAngle = AstronomyMath.ToRad(45);
            double pitchAngle = AstronomyMath.ToRad(30);
            double viewAngle = OptimalChain.Constants.camera_angle;

            DateTime dt1 = new DateTime(2019, 2, 2, 0, 00, 00);
            DateTime dt2 = new DateTime(2019, 2, 2, 1, 9, 00);

            DateTime segmdt1 = new DateTime(2019, 2, 2, 0, 00, 00);
            DateTime segmdt12 = new DateTime(2019, 2, 2, 0, 0, 15);
            DateTime segmdt2 = new DateTime(2019, 2, 2, 0, 2, 9);

            Trajectory trajectory = fetcher.GetTrajectorySat(segmdt1, segmdt2);

            int step = 5;

            Console.Write("GEOMETRYCOLLECTION(");
            

            Console.WriteLine(getWKTViewInterpolPolygons(trajectory, segmdt1, segmdt2, rollAngle, pitchAngle, step));
            Console.Write(",");
            Console.WriteLine(Polygon.getMultipolFromPolygons(new List<Polygon>() { SatLane.getRollPitchLanePolygon(trajectory, rollAngle, pitchAngle) }));

            Console.Write(",");
            Console.WriteLine(getWKTViewInterpolPolygons(trajectory, segmdt1, segmdt2, -rollAngle, pitchAngle, step));
            Console.Write(",");
            Console.WriteLine(Polygon.getMultipolFromPolygons(new List<Polygon>() { SatLane.getRollPitchLanePolygon(trajectory, -rollAngle, pitchAngle) }));

            Console.Write(",");
            Console.WriteLine(getWKTViewInterpolPolygons(trajectory, segmdt1, segmdt2, rollAngle, -pitchAngle, step));
            Console.Write(",");
            Console.WriteLine(Polygon.getMultipolFromPolygons(new List<Polygon>() { SatLane.getRollPitchLanePolygon(trajectory, rollAngle, -pitchAngle) }));

            Console.Write(",");
            Console.WriteLine(getWKTViewInterpolPolygons(trajectory, segmdt1, segmdt2, -rollAngle, -pitchAngle, step));
            Console.Write(",");
            Console.WriteLine(Polygon.getMultipolFromPolygons(new List<Polygon>() { SatLane.getRollPitchLanePolygon(trajectory, -rollAngle, -pitchAngle) }));

            Console.Write(")");
        }

        public void testGetSegmentStrip()
        {
            var allPols = new List<Polygon>();
                            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
            DataFetcher fetcher = new DataFetcher(managerDB);
            double rollAngle = AstronomyMath.ToRad(45);
            double pitchAngle = AstronomyMath.ToRad(30);
            double viewAngle = OptimalChain.Constants.camera_angle;

            DateTime dt1 = new DateTime(2019, 2, 2, 0, 00, 00);
            DateTime dt2 = new DateTime(2019, 2, 2, 0, 2, 00);

            DateTime segmdt1 = new DateTime(2019, 2, 2, 0, 00, 30);
            DateTime segmdt12 = new DateTime(2019, 2, 2, 0, 0, 40);
            DateTime segmdt2 = new DateTime(2019, 2, 2, 0, 00, 50);

            Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

            TrajectoryPoint pp1 = (TrajectoryPoint)fetcher.GetPositionSat(DateTime.Parse("01.02.2019 0:15:00"));

            Vector3D leftVector = LanePos.getDirectionVector(pp1, -0.793708, 0);
            Vector3D rightVector = LanePos.getDirectionVector(pp1, 0.793708, 0);

            LanePos llp1 = new LanePos(pp1, OptimalChain.Constants.camera_angle, 0);


            Char separator = System.Globalization.CultureInfo.CurrentCulture.NumberFormat.CurrencyDecimalSeparator[0];
            int step = 5;

            Console.Write("GEOMETRYCOLLECTION(");

            Console.WriteLine(getWKTInterpolateTrajectory(trajectory, segmdt1, segmdt2, step));
            Console.Write(",");
            Console.WriteLine(getWKTViewLine(trajectory, segmdt1, segmdt2, rollAngle, 0, step));
            Console.Write(",");
            Console.WriteLine(getWKTStripSegment(trajectory, segmdt1, segmdt2, rollAngle));
            Console.Write(",");
            Console.WriteLine(getWKTViewInterpolPolygons(trajectory, segmdt1, segmdt2, rollAngle, 0, step));
            //  Console.WriteLine(getWKTViewPolygon(trajectory, rollAngle, 0, segmdt1));
            //  Console.Write(",");
            //  Console.WriteLine(getWKTViewPolygon(trajectory, rollAngle, 0, segmdt2));


            Console.Write(")");
        }

        public void testTrapezium()
        {
         //   testTrajectoryInterpolationFromDat();
           // return;
            //test_plotTrajectoryAndSeveralViewPolygons();
                    
            var allPols = new List<Polygon>();
                            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
            DataFetcher fetcher = new DataFetcher(managerDB);
            double rollAngle = -AstronomyMath.ToRad(45);
            double pitchAngle = 0;// AstronomyMath.ToRad(30);
            double viewAngle = OptimalChain.Constants.camera_angle;

            DateTime dt1 = new DateTime(2019, 2, 2, 0, 00, 00);
            DateTime dt2 = new DateTime(2019, 2, 2, 0, 2, 00);
            //DateTime dt1 = new DateTime(2019, 1, 4,7,0,0);
            //DateTime dt2 = new DateTime(2019, 1, 4,8,0,0);

            DateTime segmdt1 = new DateTime(2019, 2, 2, 0, 00, 30);
            DateTime segmdt12 = new DateTime(2019, 2, 2, 0, 0, 40);
            DateTime segmdt2 = new DateTime(2019, 2, 2, 0, 00, 50);

            string testpolstr;
            if (rollAngle > 0)
                testpolstr = "POLYGON((139.94934082031247 -3.4311748572202134,139.80102539062497 -2.9677272712041542,140.21575927734375 -2.855262784366559,140.37506103515622 -3.2502085616531673,139.94934082031247 -3.4311748572202134))";
            else
                testpolstr = "POLYGON((153.0560302734375 -5.883796361755699,151.622314453125 -5.621452591118825,152.07824707031247 -3.9080988818941194,154.0887451171875 -4.340933889327545,153.76464843749997 -4.882994243904847,152.24304199218747 -4.729726554568899,152.1441650390625 -5.008866554677795,153.47900390625 -5.643319165077756,153.0560302734375 -5.883796361755699))";

            Polygon testPol = new Polygon(testpolstr);
            RequestParams rp = new RequestParams(_id : 301, _priority: 3, _timeFrom: dt1, _timeTo: dt2, _Max_SOEN_anlge: 55 * Math.PI / 180, _minCoverPerc: 50, _Max_sun_angle: 10, _Min_sun_angle: 90, _wktPolygon: testpolstr, _albedo : 0.36, _compression : 10, _shootingType : 0);
            

            Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

            SatLane lane = new SatLane(trajectory, rollAngle, OptimalChain.Constants.camera_angle);

            List<CaptureConf> res = lane.getCaptureConfs(rp);

            //POINT(30.12333691120147 -42.379585265899514),POINT(30.12430787086486 -42.379676407283156))
            GeoPoint gp1 = new GeoPoint(-42.379585265899514, 30.12333691120147);
            GeoPoint gp2 = new GeoPoint(-42.379676407283156, 30.12430787086486);
            Console.WriteLine(GeoPoint.DistanceOverSurface(gp1, gp2));

            Console.Write("GEOMETRYCOLLECTION(");

            Console.WriteLine(getWKTStrip(trajectory,  rollAngle, 5));
            Console.Write(",");
            Console.WriteLine(getWKTViewLine(trajectory, res[0].dateFrom, res[0].dateTo, rollAngle, 0, step : 5));
            Console.Write(",");
            Console.WriteLine(getWKTStripSegment(trajectory, res[0].dateFrom, res[0].dateTo, rollAngle));
            Console.Write(",");
            Console.WriteLine(testpolstr);

            


            //Console.WriteLine(getWKTStripSegment(trajectory, segmdt1, segmdt2, rollAngle));                   

            Console.Write(")");
 
        }



        public void test_Booblick2()
        {
            DateTime dt1 = DateTime.Parse("01.02.2019 00:00:00");
            DateTime dt2 = DateTime.Parse("01.02.2019 00:59:00");

                            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
            DataFetcher fetcher = new DataFetcher(managerDB);


            string wkt1 = "POLYGON ((-15.7763671875 -57.5158228655388,-15.7476677787646 -57.5143790621652,-16.5673828125 -56.2921566850765,-19.6435546875 -57.4685891920893,-16.9278227030282 -57.9963198478714,-15.7763671875 -57.5158228655388))"    ;   
            string wkt2 ="POLYGON((-16.5673828125 -59.3107679560388,-16.9498671881788 -58.8959440932022,-19.423828125 -59.6677405816496,-15.688476562499998 -60.6301017662667,-15.3803417658345 -59.2198858496667,-16.5673828125 -59.3107679560388))";
            string wkt3 ="POLYGON((-17.5341796875 -58.2401635434164,-16.9278227030282 -57.9963198478714,-19.6435546875 -57.46858919208929,-19.423828125 -59.6677405816496,-16.9498671881788 -58.89594409320231,-17.5341796875 -58.2401635434164))";


            //string wkt1 = "POLYGON ((-12.715963521347 -58.9384020443573,-11.337890625 -59.2658806282581,-12.0849609375 -56.6803737895014,-13.3968340721309 -57.3893745928461,-12.715963521347 -58.9384020443573))";
            //string wkt2 = "POLYGON ((-12.7001953125 -58.9726671545015,-15.3803417658345 -59.2198858496667,-15.6884765625 -60.6301017662667,-11.337890625 -59.2658806282581,-12.715963521347 -58.9384020443573,-12.7001953125 -58.9726671545015))";


            RequestParams prms1 = new RequestParams(0, 1, dt1, dt2, 99999, 1, 99999, 9999, wkt1);
            RequestParams prms2 = new RequestParams(0, 1, dt1, dt2, 99999, 1, 99999, 9999, wkt2);
            RequestParams prms3 = new RequestParams(0, 1, dt1, dt2, 99999, 1, 99999, 9999, wkt3);

          //  var res = Sessions.getCaptureConfArray(new List<RequestParams>() { prms1, prms2 }, dt1, dt2, managerDB, new List<Tuple<DateTime, DateTime>>(), new List<Tuple<DateTime, DateTime>>());

            double viewAngle = OptimalChain.Constants.camera_angle; // угол обзора камеры
            double angleStep = viewAngle; // шаг равен углу обзора

            double Max_SOEN_anlge = prms1.Max_SOEN_anlge;
            foreach (var req in new List<RequestParams>() { prms1, prms2, prms3 })
            {
                if (req.Max_SOEN_anlge > Max_SOEN_anlge)
                    Max_SOEN_anlge = req.Max_SOEN_anlge;

            }
            Trajectory traj = fetcher.GetTrajectorySat(dt1, dt2);
            double max_roll_angle = 0.261408;//Math.Min(Max_SOEN_anlge, OptimalChain.Constants.max_roll_angle);
            double min_roll_angle = 0.178328;//Math.Max(-Max_SOEN_anlge, -OptimalChain.Constants.max_roll_angle);

            int num_steps = (int)((max_roll_angle - min_roll_angle) / angleStep); /// @todo что делать с остатком от деления?
            List<Polygon> allpols = new List<Polygon>();                                                                 
            for (double rollAngle = min_roll_angle; rollAngle <= max_roll_angle; rollAngle += angleStep)
            { 
                SatLane viewLane = new SatLane(traj, rollAngle, viewAngle);
                int numtr = 0;
                List<CaptureConf> allconfs = new List<CaptureConf>();
                foreach (var request in new List<RequestParams>() { prms1, prms2, prms3 })
                {
                    if (Math.Abs(rollAngle) > Math.Abs(request.Max_SOEN_anlge))
                        continue;
                    List<CaptureConf> confs = viewLane.getCaptureConfs(request);

                    if (confs.Count == 0)                    
                        continue;

                    //CaptureConf.compressCConfArray(confs);
                    //CaptureConf.compressTwoCConfArrays(allconfs, confs);
                    allconfs.AddRange(confs);

                  //  allconfs.AddRange(confs);
                    numtr++;
                }

                CaptureConf.compressCConfArray(allconfs);
                List<TimePeriod> re = TimePeriod.compressTimePeriods(allconfs.Select(conf => new TimePeriod(conf.dateFrom, conf.dateTo)).ToList());
                if (allconfs.Count > 1)
                {
                    Console.WriteLine("rollAngle = {0}", rollAngle);                   
                    //Console.WriteLine("isNeedUnit 01 = {0}", CaptureConf.isNeedUnit(allconfs[0], allconfs[1]));
                    //Console.WriteLine("isNeedUnit 12 = {0}", CaptureConf.isNeedUnit(allconfs[1], allconfs[2]));
                    //Console.WriteLine();  

                    foreach (var conf in allconfs)
                    {
                        SatLane strip = new SatLane(traj, conf.rollAngle, OptimalChain.Constants.camera_angle);
                        allpols.Add(strip.getSegment(conf.dateFrom, conf.dateTo));
                    }
                }
            }


            //List<Polygon> allpols = new List<Polygon>();
            //foreach (var conf in res)
            //{
            //    SatLane strip = new SatLane(traj, conf.rollAngle, OptimalChain.Constants.camera_angle);
            //    allpols.Add(strip.getSegment(conf.dateFrom, conf.dateTo));
            //}

            Console.Write("GEOMETRYCOLLECTION(");
            Console.WriteLine(wkt1);
            Console.Write(",");
            Console.WriteLine(wkt2);
            Console.Write(",");
            Console.WriteLine(wkt3);
            Console.Write(",");
            Console.WriteLine(Polygon.getMultipolFromPolygons(allpols));
            Console.Write(")"); 

        }



        public void test_Booblick()
        {
           // test_Booblick2();
           // return;

            DateTime dt1 = DateTime.Parse("01.02.2019 00:00:00");
            DateTime dt2 = DateTime.Parse("01.02.2019 00:59:00");
            
                            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
            DataFetcher fetcher = new DataFetcher(managerDB);

            double rollAngle = AstronomyMath.ToRad(45);
            double pitchAngle = AstronomyMath.ToRad(30);
            double viewAngle = OptimalChain.Constants.camera_angle;
            Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);
 
            string outPol = "POLYGON((-16.5673828125 -56.292156685076456,-12.0849609375 -56.68037378950137,-11.337890625 -59.265880628258095,-15.688476562499998 -60.63010176626667,-19.423828124999996 -59.66774058164962,-19.6435546875 -57.46858919208933,-16.5673828125 -56.292156685076456))";
            string intPol = "POLYGON((-15.776367187500002 -57.515822865538816,-17.534179687499996 -58.24016354341643,-16.5673828125 -59.31076795603883,-12.7001953125 -58.97266715450152,-13.403320312499998 -57.3739384187141,-15.776367187500002 -57.515822865538816))";

            List<string> holes = new List<string>() { intPol };


           // List<string> wktl = new List<string>()
           // {
           //     "POLYGON ((-15.7763671875 -57.5158228655388,-15.7476677787646 -57.5143790621652,-16.5673828125 -56.2921566850765,-19.6435546875 -57.4685891920893,-16.9278227030282 -57.9963198478714,-15.7763671875 -57.5158228655388))"
           // //,
           // //"POLYGON ((-13.4033203125 -57.3739384187141,-13.3968340721309 -57.3893745928461,-12.0849609375 -56.6803737895014,-16.5673828125 -56.2921566850765,-15.7476677787646 -57.5143790621652,-13.4033203125 -57.3739384187141))"
           // //,
           // //"POLYGON ((-12.715963521347 -58.9384020443573,-11.337890625 -59.2658806282581,-12.0849609375 -56.6803737895014,-13.3968340721309 -57.3893745928461,-12.715963521347 -58.9384020443573))"
           // //,
           //// "POLYGON ((-12.7001953125 -58.9726671545015,-15.3803417658345 -59.2198858496667,-15.6884765625 -60.6301017662667,-11.337890625 -59.2658806282581,-12.715963521347 -58.9384020443573,-12.7001953125 -58.9726671545015))"
           // ,
           // "POLYGON((-16.5673828125 -59.3107679560388,-16.9498671881788 -58.8959440932022,-19.423828125 -59.6677405816496,-15.688476562499998 -60.6301017662667,-15.3803417658345 -59.2198858496667,-16.5673828125 -59.3107679560388))"
           // ,"POLYGON((-17.5341796875 -58.2401635434164,-16.9278227030282 -57.9963198478714,-19.6435546875 -57.46858919208929,-19.423828125 -59.6677405816496,-16.9498671881788 -58.89594409320231,-17.5341796875 -58.2401635434164))"

           // };
            
           List<RequestParams> lst = new List<RequestParams>() { new RequestParams(0, 1, dt1, dt2, 99999, 1, 99999, 9999, outPol, holes) };
           
           // List<RequestParams> lst = wktl.Select(str =>   new RequestParams(0, 1, dt1, dt2, 99999, 1, 99999, 9999, str)).ToList();

            DateTime start = DateTime.Now;

            var res = Sessions.getCaptureConfArray(lst, dt1, dt2, managerDB, new List<TimePeriod>(), new List<TimePeriod>());

            DateTime end = DateTime.Now;
            Console.WriteLine("total time = " + (end - start).TotalSeconds.ToString());


            Trajectory traj = fetcher.GetTrajectorySat(dt1, dt2);

            List<Polygon> allpols = new List<Polygon>();
            foreach (var conf in res)
            {
                SatLane strip = new SatLane(traj, conf.rollAngle, OptimalChain.Constants.camera_angle);
                allpols.Add(strip.getSegment(conf.dateFrom, conf.dateTo));
            }

            Console.Write("GEOMETRYCOLLECTION(");
            //Console.WriteLine(outPol);
            //Console.Write(",");
            //Console.WriteLine(intPol);
            Console.WriteLine(Polygon.getMultipolFromPolygons( lst.SelectMany(l => l.polygons).ToList() ));
            Console.Write(",");
            Console.WriteLine(Polygon.getMultipolFromPolygons(allpols));
            Console.Write(")");                         
        }


        


        public IList<CaptureConf> test_getCaptureConfArray()
        {
            test_Booblick();
            return null;

            //DateTime dt1 = DateTime.Parse("01.02.2019 1:12:30") ; 
            //DateTime dt2 = DateTime.Parse("01.02.2019 1:14:30"); 

            DateTime dt1 = new DateTime(2019, 1, 6);
            DateTime dt2 = new DateTime(2019, 1, 8);
                            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);

             //for (int i = 0; i < 10; i++)
             //{ 
             //    List<Tuple<DateTime, DateTime>> shadowPeriods;
             //    List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot;
             //    Sessions.checkIfViewLaneIsLitWithTimeSpans(managerDB, dt1, dt2, out partsLitAndNot, out shadowPeriods);

             //    Console.WriteLine(shadowPeriods.Count);
             //}


             //return null;


            #region

            //DataFetcher fetcher = new DataFetcher(managerDB);
            //var dlfpd = fetcher.GetMinimumPointsArray(DateTime.Parse("04.01.2019 0:22:16"), DateTime.Parse("04.01.2019 0:22:16"));
            //TrajectoryPoint ppp = (TrajectoryPoint)fetcher.GetPositionSat(DateTime.Parse("04.01.2019 0:22:16"));
            //Console.WriteLine("ppp = {0}", ppp);

            //return null;

            /*
            DBTables.DataFetcher fetcher = new DBTables.DataFetcher(managerDB);

            var trajectory = fetcher.GetTrajectorySat(dt1, dt2);
            SatLane strip15 = new SatLane(trajectory, _rollAngle: 0.3, _pitchAngle: 0, _viewAngle: OptimalChain.Constants.camera_angle*2, polygonStep: OptimalChain.Constants.stripPolygonStep);
            SatLane strip1 = new SatLane(trajectory, _rollAngle: 0.3, _pitchAngle: 0, _viewAngle: OptimalChain.Constants.camera_angle *2, polygonStep: 1);

            //DateTime segmentDt1 = new DateTime(2019, 2, 2, 4, 00, 00);
            //DateTime segmentDt2 = new DateTime(2019, 2, 2, 4, 01, 00);
            //var segment1 = strip1.getSegment(segmentDt1, segmentDt2);
            //var segment15 = strip15.getSegment(segmentDt1, segmentDt2);

            //Console.WriteLine("segment1 = {0}", segment1.ToWtk());
            //Console.WriteLine("segment15 = {0}", segment15.ToWtk());
            
            captureLanes.Add(strip15);
            captureLanes.Add(strip1);
              
              return null;
            */


            //      var pol1 = new Polygon("POLYGON((140.67658615905074 -30.35266808565639, 140.49224100571504 -30.85915223183219, 140.96342848938676 -31.030650450647485, 141.14777364272246 -30.524166304471677, 140.67658615905074 -30.35266808565639))");
      //      var pol2 = new Polygon("POLYGON ((140.20082574914 -30.275170650554,140.056877086177 -30.8179109896749,140.169988541764 -30.8395432221064,140.313051263494 -30.2966902815026,140.20082574914 -30.275170650554))");

      //      Tuple<IList<Polygon>, IList<Polygon>> res = Polygon.IntersectAndSubtract(pol1, pol2);

      //      Console.WriteLine("wkt = {0}", res.Item2[0].ToWtk());

      //      int i = 4;

          //  wkt = POLYGON ((140.20082574914 -30.275170650554,140.056877086177 -30.8179109896749,140.169988541764 -30.8395432221064,140.313051263494 -30.2966902815026,140.20082574914 -30.275170650554))
            // wkt = POLYGON ((140.20082574914 -30.275170650554,140.056877086177 -30.8179109896749,140.169988541764 -30.8395432221064,140.313051263494 -30.2966902815026,140.20082574914 -30.275170650554))

         //   testPMI();
           // return null;
            //Random rand = new Random((int)DateTime.Now.Ticks);
            //for (int i = 0; i < 40; i++)
            //{
            //    Polygon randpol = getRandomPolygon(rand, 3, 10, 5, 15);
            //    polygons.Add(randpol);
            //}
            //return null;

            DateTime start = DateTime.Now;
            //  if (curRequest == null)
            //      return null;

            List<RequestParams> requests = new List<RequestParams>();

            polygons.Clear();

            // polygons.Add(new Polygon("POLYGON ((-2 -4, -2 -2, -4 -2, -4 -4, -2 -4))"));

            //polygons.Add(new Polygon("POLYGON ((12 8, 12 12, 8 12, 8 8, 12 8))"));

            //polygons.Add(new Polygon("POLYGON ((6 2, 6 6, 2 6, 2 2, 6 2))"));

            // polygons.Add(new Polygon("POLYGON((-29 27,  -23 33 , -21 31, -27 25, -29 27)"));
            //polygons.Add(new Polygon("POLYGON((-315.14378163320714 -1.645382936563152, -306.1789378832072 9.73302071251409, -341.3351878832072 29.937923070513676, -351.70628163320714 8.344268391587661, -315.14378163320714 -1.645382936563152))"));
            //polygons.Add(new Polygon("POLYGON((-315.14378163320714 -1.645382936563152, -306.1789378832072 9.73302071251409, -341.3351878832072 29.937923070513676, -351.70628163320714 8.344268391587661, -315.14378163320714 -1.645382936563152))"));
            //     polygons.Add(new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))"));
   

           //  polygons.Add(new Polygon("POLYGON ((0.02 -0.02, 0.02 0.02, -0.02 0.02, -0.02 -0.02, 0.02 -0.02))"));

            //      polygons.Add(new Polygon("POLYGON ((0 0, 0 4, -4 4, -4 0, 0 0))"));
            //      polygons.Add(new Polygon("POLYGON ((-14 22, -18 26, -22 22, -18 18, -14 22))"));
            //      polygons.Add(new Polygon("POLYGON ((-24 16, -12 28, -16 32, -28 20, -24 16))"));
            //      polygons.Add(new Polygon("POLYGON ((-29 27, -27 25, -21 31, -23 33 , -29 27))"));

            ///
            //foreach (var req in Requests)
            //{       
            //    var pol = new SphericalGeom.Polygon(req.Polygon.Select(sp => GeoPoint.ToCartesian(new GeoPoint(sp.Lat, sp.Lon), 1)), new Vector3D(0, 0, 0));

            //polygons.Add(new Polygon("POLYGON ((106.95738018444 -30.609773998998,106.700176507603 -33.064421050007,110.603738797998 -36.72736549789,111.040933447857 -35.3975779951589,106.95738018444 -30.609773998998))"));
          //  polygons.Add(new Polygon("POLYGON ((133.41867554681 -11.0582796777856,134.79494602739 -16.8112932429141,141.527607361566 -7.43738453881555,133.41867554681 -11.0582796777856))"));
            // polygons.Add(new Polygon("POLYGON ((138.670930231104 -73.0085047795101,143.14991204937 -83.8960681103823,145.941798919954 -75.7762365711183,138.670930231104 -73.0085047795101))"));
            //polygons.Add(new Polygon("POLYGON ((126.708643529393 -5.18327756706429,126.739746677109 -5.39253384427269,138.433146631633 -3.22822909968528,137.855654937064 -2.76997099530497,137.340205104196 -2.46307396240683,126.708643529393 -5.18327756706429))"));
          /////  polygons.Add(new Polygon("POLYGON ((-143.260352662219 -83.2068449781505,-143.514471593982 -83.4356344480832,-143.576392802203 -90,-133.829165175185 -89.8117100894023,-132.997685724069 -87.9447040949423,-133.509387302723 -84.6908830347334,-143.260352662219 -83.2068449781505))"));
          //  polygons.Add(new Polygon("POLYGON ((68.8371301436284 64.5679742254139,78.1675312294433 65.5242184669642,74.2848812648429 66.6953759634614,68.8371301436284 64.5679742254139))"));
            //polygons.Add(new Polygon("POLYGON ((-55.8885998929106 -51.0299184009157,-59.6660285951263 -53.3592811563356,-60.998245640736 -57.1388000817364,-47.4972819949729 -59.295121154378,-49.1270533296085 -52.5489538484122,-52.6093146844346 -50.9235872173843,-55.8885998929106 -51.0299184009157))"));
            //polygons.Add(new Polygon("POLYGON ((56.5653703288219 69.7182063183484,59.1705391644263 67.225426956238,63.7461199642006 66.4286120063546,67.4258940485706 67.5465288182627,69.6882614283301 70.7278221689149,56.5653703288219 69.7182063183484))"));
            //polygons.Add(new Polygon("POLYGON ((26.8338545786005 78.2610774769097,29.7522575089587 78.4741186461335,28.3276502452736 85.8875266545802,26.8338545786005 78.2610774769097))"));
          //  polygons.Add(new Polygon("POLYGON ((-148.413059539385 60.6528298133914,-144.595795884903 59.4715669443149,-137.99680556921 59.7291907557153,-148.413059539385 60.6528298133914))"));
            //polygons.Add(new Polygon("POLYGON ((139.414808620807 42.8765727046178,137.127415045697 37.8230626726055,137.531307137287 33.8516656011559,138.848270314871 31.4934729642023,141.593130765639 31.9864673372116,142.559851317366 34.1804613176344,139.414808620807 42.8765727046178))"));
            //polygons.Add(new Polygon("POLYGON ((89.3463480833278 75.3563418684926,90.17789290387 73.507015483096,95.5213484607249 72.2117366912906,97.5895423180327 72.5056315730144,98.9744831080816 72.9929570395977,100.617063460934 75.4759080001207,89.3463480833278 75.3563418684926))"));
         //   polygons.Add(new Polygon("POLYGON ((26.8138989934746 84.5757037848601,25.7066986554876 83.2716031647492,25.460553718472 75.4250840084123,25.6987497258697 74.74736526883,26.8138989934746 84.5757037848601))"));
            //polygons.Add(new Polygon("POLYGON ((-143.509603288707 -73.6057989450235,-141.58042591158 -75.247277729637,-138.2208107487 -75.1495515529046,-137.113190686083 -63.6670184451398,-139.657280048046 -62.4161633983284,-143.509603288707 -73.6057989450235))"));
            //polygons.Add(new Polygon("POLYGON ((-15.4580806424608 -22.6141475434161,-15.2667895047222 -23.141422739191,-13.577056791552 -25.3600098170744,-9.0829252046855 -26.5270448390978,-6.23468715270856 -25.2042257864628,-4.96856810034895 -23.6315694368746,-15.4580806424608 -22.6141475434161))"));

         /*   polygons.Add(new Polygon("POLYGON ((-78.8955984362672 -4.27544586717168,-77.2982435684835 -8.80701993693518,-76.0355707731791 -9.91258542782664,-78.8955984362672 -4.27544586717168))"));
            polygons.Add(new Polygon("POLYGON ((179.579289520074 10.6889467971472,178.574485675387 8.34346200482048,178.300283947013 5.92319028653008,178.301940565539 5.79922342988971,179.509489550329 1.4148425216926,179.977243125844 0.808526713961943,-176.696585336882 11.8025603068032,179.579289520074 10.6889467971472))"));
            polygons.Add(new Polygon("POLYGON ((79.9707715668423 15.8899537981103,74.7928143619591 7.66737880195265,81.3890491389272 3.91874091741299,83.7903179867659 4.06285736651357,89.7013599496656 9.03295479607039,79.9707715668423 15.8899537981103))"));
            polygons.Add(new Polygon("POLYGON ((-22.0093787436407 -79.17611002106,-21.4058118139738 -81.0901792860746,-20.8223570204733 -81.9639274544874,-19.5501335170067 -82.8222251271947,-18.4706693302515 -82.8280389339737,-15.9157345156895 -77.5069339064041,-16.0995903951845 -76.2702049340253,-22.0093787436407 -79.17611002106))"));
            polygons.Add(new Polygon("POLYGON ((-49.4689524581043 -78.1241384677363,-37.0254353233509 -80.0039018907845,-41.6101624796887 -73.1048739844596,-49.4689524581043 -78.1241384677363))"));
            polygons.Add(new Polygon("POLYGON ((-42.1297446390768 -55.477013622913,-39.8403615537175 -65.7381830490545,-37.7368644530883 -66.1700038006077,-32.4611019569797 -55.0642804489959,-34.0793624326851 -54.2894218371821,-42.1297446390768 -55.477013622913))"));
            polygons.Add(new Polygon("POLYGON ((28.5759040376927 -33.7168641517643,25.709842398812 -34.811742911266,24.9078661116917 -35.5353804219651,24.592481814517 -35.9048727335882,27.0744859083197 -45.9426929740054,33.5847852895836 -43.8586716636478,32.9108698874432 -35.3476480800036,28.5759040376927 -33.7168641517643))"));
            polygons.Add(new Polygon("POLYGON ((54.2297736032941 -35.289707564306,53.696678024114 -42.3280880106462,53.9161727931182 -42.5144409669209,54.3611472449329 -42.7705129510599,57.4154623790318 -37.9943848731226,56.0874138352781 -35.4882563008246,54.2297736032941 -35.289707564306))"));
            polygons.Add(new Polygon("POLYGON ((-163.555204262938 67.48966029635,-160.889444105472 65.4032934420663,-159.106597228681 66.4491329584151,-157.701289193239 74.2059689990217,-163.555204262938 67.48966029635))"));
            polygons.Add(new Polygon("POLYGON ((161.724825519704 31.8117411336656,161.037548520549 20.31125254061,166.751110706636 20.9141950611686,170.281496006867 24.948348271668,170.328418350756 25.181348243781,161.724825519704 31.8117411336656))"));
            polygons.Add(new Polygon("POLYGON ((120.388684206653 -74.3544290607743,122.31536246308 -74.9778501501178,122.919787297783 -74.8060037583622,120.388684206653 -74.3544290607743))"));
            polygons.Add(new Polygon("POLYGON ((-56.8968329994112 77.1959492679278,-56.9964068867487 75.7754056775369,-55.892752496346 72.3395491882293,-49.7004571466462 78.9960875151499,-56.8968329994112 77.1959492679278))"));
            polygons.Add(new Polygon("POLYGON ((-81.860646175566 13.2379682379701,-81.2117783020112 12.0751343240872,-80.6514156350255 11.4296342760156,-75.6802674359291 9.60718527111581,-69.8451794555669 15.6507149545838,-70.5913960094611 17.6399092420469,-81.860646175566 13.2379682379701))"));
            polygons.Add(new Polygon("POLYGON ((-165.21673846612 -54.0589138057729,-165.573261002891 -54.9705389496409,-165.679803494884 -56.4150844568433,-165.655215606378 -56.6166901082156,-165.21673846612 -54.0589138057729))"));
            polygons.Add(new Polygon("POLYGON ((73.166398746769 83.3795342613812,69.1157648056298 78.3986011694295,69.1017292969888 77.8679160512509,82.572775740474 76.204628990747,82.7895030334302 76.948348271668,82.8510351925494 77.29836274115,77.2353040566382 83.8046777874499,73.166398746769 83.3795342613812))"));
            */


            //// polygons.Add(new Polygon("POLYGON ((26.0783192463844 -46.5841481916318,35.7476829726148 -50.2074973317149,38.0478959239693 -44.1392035223282,37.9091621107214 -43.3620420190943,37.7692823966422 -42.8563445227041,36.0679047735226 -40.0818489445239,32 -38.4,26.0783192463844 -46.5841481916318))"));

          //  polygons.Add(new Polygon("POLYGON((31.999998092651374 -38.40001524136964,31.949615478515618 -38.47186888757715,31.87271118164062 -38.59111377614743,31.787567138671875 -38.711232538952245,31.70379638671875 -38.83649855755878,31.604919433593754 -38.97542487543143,31.514282226562496 -39.110882537651776,31.416778564453118 -39.254588032219935,31.320648193359368 -39.395877126120325,31.19155883789062 -39.57499787247078,31.09954833984375 -39.714581756674136,30.92788696289063 -39.959227732549735,30.516811070625 -40.522957467748796,30.141675885937502 -41.05796006928006,29.818885145625 -41.53642525541324,29.583335145937497 -41.89459230713839,29.243096257499996 -42.34755224461481,28.867961072812495 -42.84206525405591,28.527722184374994 -43.30726336928021,28.161311073749996 -43.838187772879266,27.655314778125003 -44.476586348941424,27.262731445312507 -45.046470974453676,26.8788721865625 -45.53134234600353,26.6171499646875 -45.92717932066853,26.294359224375 -46.28404919936878,26.084981446875 -46.57867696533415,26.652046260937503 -46.80009584431832,27.288903667499998 -47.038446804815074,27.9083129259375 -47.27573777161199,28.5887907028125 -47.541423912593864,29.3914055165625 -47.846770617344674,30.18529625625 -48.15033025399259,30.935566625625 -48.42316553673152,31.537527735937495 -48.636925905779336,32.148212920312496 -48.86700362106729,32.715277734375 -49.078887967947956,33.25617032625001 -49.28987225636626,33.893027732812506 -49.52261556428171,34.512436991250006 -49.74861960810681,35.088225879375 -49.95674016499248,35.55932587875001 -50.1192354973219,35.751255508125006 -50.1918997652828,35.960633285625 -49.69221691198592,36.12639069281251 -49.295562051788416,36.32704439625001 -48.73483767318533,36.536422173750005 -48.15033025399259,36.7196277290625 -47.729541166814705,36.963901802812494 -47.097868896254404,37.12093513593749 -46.66256383068718,37.391381431875004 -45.89682991175613,37.6618277278125 -45.1634641743713,37.8712055053125 -44.594740493578705,38.0631351346875 -44.12694808996411,37.940998097812496 -43.69328098795961,37.85375735718751 -43.243746370599794,37.74906846843751 -42.80367277237827,37.44372587624999 -42.30240199364288,37.077314765625 -41.71250212742097,36.710903654999996 -41.1893954419583,36.43173328500001 -40.70838407759651,36.18745921125001 -40.29710512310906,36.04787402625001 -40.05046399667327,34.87884810187501 -39.6150074100132,34.250714769375 -39.35240973567654,33.63130551093751 -39.075277397772716,33.038068474687506 -38.83783639639242,32.61058884562501 -38.674560221614776,32.20055736468751 -38.46310943260255,31.999998092651374 -38.40001524136964))"));
       
            polygons.Add(new Polygon("POLYGON ((-15.8664209385787 -41.8827252308367,-14.8106463870087 -49.6164733955684,-10.9440009333287 -38.700314997375,-15.8664209385787 -41.8827252308367))"));
            polygons.Add(new Polygon("POLYGON ((163.414914670518 -23.1209193064958,171.886393790662 -27.90656213235,175.613451853244 -25.9126049895841,174.843139727864 -23.2980456916021,171.10490459146 -21.9525325763657,163.414914670518 -23.1209193064958))"));
            polygons.Add(new Polygon("POLYGON ((-13.6661621940662 61.0353110690154,-15.486750526803 55.5407719575142,-15.2674461248971 54.9466985506005,-15.0495054314605 54.5127936835052,-9.64885334199004 52.3068425359899,-6.30568820568854 56.7589992726206,-6.47127652853919 58.3107442602605,-13.6661621940662 61.0353110690154))"));
            polygons.Add(new Polygon("POLYGON ((126.603881131078 35.9319950327046,127.662062991179 34.9160144992729,141.382287221762 35.8547882578265,140.344012519996 37.0811195744369,126.603881131078 35.9319950327046))"));
            polygons.Add(new Polygon("POLYGON ((174.764524222916 82.7007470853579,176.000199701313 80.3893464601858,179.840919111031 80.2118170917372,174.764524222916 82.7007470853579))"));
            polygons.Add(new Polygon("POLYGON ((-166.653603357378 -58.1912078091185,-164.167035610375 -61.9445518787533,-163.411191899595 -62.1541595974383,-162.471593249977 -62.2840342809686,-158.036232033093 -60.8611506899791,-160.719689103894 -55.8195560419926,-166.653603357378 -58.1912078091185))"));
            polygons.Add(new Polygon("POLYGON ((-136.67983980768 15.550501185823,-134.183811899563 5.66329974762385,-129.673532121944 8.98053705130875,-129.123231511427 12.160802007753,-131.303634416966 19.9334426881156,-136.67983980768 15.550501185823))"));
            polygons.Add(new Polygon("POLYGON ((-146.863347083308 50.6703741078009,-146.29453013041 49.6998941206875,-143.533966359605 48.4357149342326,-142.96583004738 48.352969285425,-141.211637456659 48.3351748097207,-138.254788208575 52.741031677548,-141.026520279104 53.6461797601714,-146.863347083308 50.6703741078009))"));
            polygons.Add(new Polygon("POLYGON ((41.2073239717082 9.62969208953022,45.8993180646144 2.50659671317216,44.2856486340812 9.07875014448496,41.2073239717082 9.62969208953022))"));
            polygons.Add(new Polygon("POLYGON ((71.5389625706523 14.0170518331449,71.8234245352136 9.8181098353429,78.7070873188562 15.1974109916451,71.5389625706523 14.0170518331449))"));
            polygons.Add(new Polygon("POLYGON ((150.282960556559 68.7657081660049,149.25821079531 61.452448743304,157.697181819883 63.0311001920605,158.588436630892 64.7692405345148,156.688331597956 67.7449967310437,156.498815816154 67.8535865886079,155.834832059546 68.1741259809839,150.282960556559 68.7657081660049))"));
            polygons.Add(new Polygon("POLYGON ((49.3014047092355 -78.3595995644856,50.5360158774887 -88.622139401323,52.9579896767873 -89.6529419440329,61.5676417079571 -86.6018564085675,61.9964779200923 -80.1185979226873,55.9217839961247 -76.1496204126083,49.3014047092355 -78.3595995644856))"));
            polygons.Add(new Polygon("POLYGON ((115.4370418576 -42.7549470226189,121.788418586957 -47.6561713127527,123.606854030957 -46.2437912107672,115.4370418576 -42.7549470226189))"));
            polygons.Add(new Polygon("POLYGON ((-26.3279424201223 90,-22.4614324311582 78.4002733632755,-21.9625575703847 78.9816223926563,-21.1542711995688 80.5288407706938,-20.7890568565678 81.6991140163714,-20.4220738917181 85.8071593926865,-20.9903064770381 89.0054359822459,-26.3279424201223 90))"));
            polygons.Add(new Polygon("POLYGON ((145.658802799189 -39.1739519816625,148.038113653869 -44.0189088138402,152.498507610645 -37.2131565127994,145.658802799189 -39.1739519816625))"));
            polygons.Add(new Polygon("POLYGON ((123.740229273654 54.359048822482,121.767235125686 46.4638562990237,122.054237487526 44.5874639613622,122.723415090441 42.6763154224208,126.299999270743 48.0050964909487,126.117726374793 50.4969707795727,124.298158528025 54.3459961197746,123.740229273654 54.359048822482))"));
            polygons.Add(new Polygon("POLYGON ((-127.245747990213 -61.8318264250873,-127.999283766234 -62.3618253313607,-128.957052259882 -65.1342841957574,-127.511082204054 -70.0605524088338,-125.663888813167 -69.1252677346447,-125.265153548812 -63.9102015988547,-127.245747990213 -61.8318264250873))"));
            polygons.Add(new Polygon("POLYGON ((-138.671204797401 -49.1092510405956,-140.24895981677 -52.9045455466279,-137.701279553012 -55.4751646884104,-126.371829725968 -49.9990231746623,-126.544569430647 -49.8000756316276,-138.671204797401 -49.1092510405956))"));
            polygons.Add(new Polygon("POLYGON ((-133.94164053758 45.1391715789649,-135.073629559564 43.2656180213095,-132.924190243753 36.5022455286815,-133.94164053758 45.1391715789649))"));
            polygons.Add(new Polygon("POLYGON ((-153.724156900857 -62.3480680285966,-149.529173163416 -60.3092267863093,-150.173546648582 -58.5845087399254,-153.724156900857 -62.3480680285966))"));
            polygons.Add(new Polygon("POLYGON ((9.67840068444707 73.6494968613506,8.15350426603461 69.2160621950517,13.8625902929402 70.6564276773748,9.67840068444707 73.6494968613506))"));
            polygons.Add(new Polygon("POLYGON ((149.857471002546 7.03591549474189,150.07377189911 0.744272936232637,155.383769641468 9.09082275366954,149.857471002546 7.03591549474189))"));
            polygons.Add(new Polygon("POLYGON ((-85.7851409961435 67.0809050952738,-86.3739167588772 66.3276396624916,-86.7360325742873 65.4757442749821,-86.9442108328882 64.6631020679371,-85.7758260129434 58.9112814964859,-84.5589636715731 58.6981302517932,-83.1767493747642 60.8167687840936,-85.7851409961435 67.0809050952738))"));
            polygons.Add(new Polygon("POLYGON ((163.28346638757 62.7652953888517,163.231579462185 57.6473670335433,163.882830753566 54.6012600516916,168.824158129547 58.2051034519246,167.364815075696 66.97042958302,166.994403641621 67.4210444314943,163.28346638757 62.7652953888517))"));
            polygons.Add(new Polygon("POLYGON ((140.455677037969 -85.5695455685603,139.800399060667 -88.0619433173144,147.769181519289 -90,149.347335088723 -85.2565506970921,140.455677037969 -85.5695455685603))"));
            polygons.Add(new Polygon("POLYGON ((-99.3822986004557 -7.68185020470924,-100.286637675719 -7.82779410236288,-105.198883496014 -11.9225155191138,-101.567581085001 -15.8218815212475,-98.5639546409902 -16.3864819612103,-90.8600133647422 -12.5669157749003,-92.6868941280083 -9.03054665568643,-99.3822986004557 -7.68185020470924))"));
            polygons.Add(new Polygon("POLYGON ((91.4013978914636 75.484003446282,90.2166117001699 75.3542190645492,86.7879101059993 72.546551391018,90.0419629197143 70.6768644866397,95.8445372535741 71.2791399490019,96.2166885861927 74.5145503916087,93.4136766144508 75.4094272409147,91.4013978914636 75.484003446282))"));
            polygons.Add(new Polygon("POLYGON ((-38.9926737965842 20.4000032975084,-37.1596804030749 20.6171394852932,-34.8765335907028 21.8475905192692,-35.4840868896827 24.676549022824,-38.9926737965842 20.4000032975084))"));
            polygons.Add(new Polygon("POLYGON ((-171.832897057659 23.1323538592612,-167.095015306559 21.4822412183158,-162.826824113493 25.7822664441844,-171.832897057659 23.1323538592612))"));
            polygons.Add(new Polygon("POLYGON ((-140.552932977979 -26.3280375649579,-144.482740739761 -29.6762537612741,-137.641800462407 -33.1512566315003,-135.662076517314 -30.9840663397532,-136.447233171395 -27.7291407268914,-137.24097416612 -27.0770287434598,-138.886682183355 -26.4150240396706,-140.552932977979 -26.3280375649579))"));

        
         //  //  polygons.Add(new Polygon("POLYGON ((-38.9763453298122 30.0080564431448,-38.6761961185558 29.8972269350411,-35.9101429589335 32.9187079087084,-37.1682845064806 34.0511201603333,-38.9763453298122 30.0080564431448))"));
         //   // polygons.Add(new Polygon("POLYGON ((68.295061612353 33.8432044928854,70.6685350162804 30.4622223756182,72.4730436946195 30.319934864894,75.21705428202 34.526308734133,68.295061612353 33.8432044928854))"));

         //  // polygons.Add(new Polygon("POLYGON((140.67658615905074 -30.35266808565639, 140.49224100571504 -30.85915223183219, 140.96342848938676 -31.030650450647485, 141.14777364272246 -30.524166304471677, 140.67658615905074 -30.35266808565639))"));

         ////   polygons.Add(new Polygon("POLYGON((78.58566289767623 17.130947926520477, 78.74633794650434 17.252957595077618, 78.8424683175981 17.44302666622589, 78.70376592501998 17.59363047055203, 78.52249151095748 17.639441559013832, 78.32611089572309 17.61195630153543, 78.29589849337934 17.602793618503057, 78.27941900119185 17.304098970796602, 78.58566289767623 17.130947926520477))"));

         //   // с 01.02.2019  16:40 по 01.02.2019 17:00



            //int id = 310;
            //foreach (var pol in polygons)
            //{
            //    RequestParams reqparams = new RequestParams();
            //    reqparams.id = id;
            //    reqparams.timeFrom = new DateTime(2019, 2, 01, 0, 00, 00);
            //    reqparams.timeTo = new DateTime(2019, 2, 25, 0, 00, 00);
            //    reqparams.shootingType = 0;
            //    reqparams.minCoverPerc = 70;
            //    reqparams.Max_SOEN_anlge = 0.9599310885968813;//AstronomyMath.ToRad(45);
            //    reqparams.wktPolygon = pol.ToWtk();
            //    reqparams.albedo = 0;
            //    reqparams.compression = 0;
            //    requests.Add(reqparams);
            //    id++;
            //}


         //   requests[0].compression = 2;
         //   requests[1].compression = 1;
         //   requests[2].compression = 1;
         //   requests[3].compression = 1;
         //   requests[4].compression = 1;
         //   requests[5].compression = 9;
         //   requests[6].compression = 1;
         //   requests[7].compression = 8;
         //   requests[8].compression = 1;
         //   requests[9].compression = 10;
         //   requests[10].compression = 1;
         //   requests[11].compression = 10;
         //   requests[12].compression = 5;
         //   requests[13].compression = 10;
         //   requests[14].compression = 1;
         //   requests[15].compression = 10;
         //   requests[16].compression = 6;
         //   requests[17].compression = 2;
         //   requests[18].compression = 2;
         //   requests[19].compression = 1;

           



            /*  
        requests[0].timeFrom = DateTime.Parse("2019-01-31T00:00:00");
        requests[0].timeTo = DateTime.Parse("2019-02-02T00:00:00");
        requests[0].priority = 3;
        requests[0].Max_SOEN_anlge = 0.9599310885968813;

        requests[1].timeFrom = DateTime.Parse("2019-01-31T00:00:00");
        requests[1].timeTo = DateTime.Parse("2019-02-02T00:00:00");
        requests[1].priority = 2;
        requests[1].Max_SOEN_anlge = 0.9599310885968813;

        requests[2].timeFrom = DateTime.Parse("2019-02-01T00:00:00");
        requests[2].timeTo = DateTime.Parse("2019-02-03T00:00:00");
        requests[2].priority = 1;
        requests[2].Max_SOEN_anlge = 0.9599310885968813;


        requests[3].timeFrom = DateTime.Parse("2019-01-31T00:00:00");
        requests[3].timeTo = DateTime.Parse("2019-02-02T00:00:00");
        requests[3].priority = 2;
        requests[3].Max_SOEN_anlge = 0.9599310885968813;

        requests[4].timeFrom = DateTime.Parse("2019-01-31T00:00:00");
        requests[4].timeTo = DateTime.Parse("2019-02-03T00:00:00");
        requests[4].priority = 3;
        requests[4].Max_SOEN_anlge = 0.9599310885968813;

        requests[5].timeFrom = DateTime.Parse("2019-01-31T00:00:00");
        requests[5].timeTo = DateTime.Parse("2019-02-02T00:00:00");
        requests[5].priority = 2;
        requests[5].Max_SOEN_anlge = 0.9599310885968813;

        requests[6].timeFrom = DateTime.Parse("2019-01-31T00:00:00");
        requests[6].timeTo = DateTime.Parse("2019-02-05T00:00:00");
        requests[6].priority = 3;
        requests[6].shootingType = 1;
        requests[6].Max_SOEN_anlge = 0.9599310885968813;

        requests[7].timeFrom = DateTime.Parse("2019-01-31T00:00:00");
        requests[7].timeTo = DateTime.Parse("2019-02-02T00:00:00");
        requests[7].priority = 1;
        requests[7].Max_SOEN_anlge = 0.9599310885968813;

        requests[8].timeFrom = DateTime.Parse("2019-01-31T00:00:00");
        requests[8].timeTo = DateTime.Parse("2019-02-03T00:00:00");
        requests[8].priority = 1;
        requests[8].Max_SOEN_anlge = 0.9599310885968813;

        requests[9].timeFrom = DateTime.Parse("2019-01-31T00:00:00");
        requests[9].timeTo = DateTime.Parse("2019-02-02T00:00:00");
        requests[9].priority = 2;
        requests[9].Max_SOEN_anlge = 0.9599310885968813;

        requests[10].timeFrom = DateTime.Parse("2019-01-31T00:00:00");
        requests[10].timeTo = DateTime.Parse("2019-02-03T00:00:00");
        requests[10].priority = 2;
        requests[10].Max_SOEN_anlge = 0.7853981633974483;

        requests[11].timeFrom = DateTime.Parse("2019-01-31T00:00:00");
        requests[11].timeTo = DateTime.Parse("2019-02-04T00:00:00");
        requests[11].priority = 3;
        requests[11].Max_SOEN_anlge = 0.9599310885968813;

        requests[12].timeFrom = DateTime.Parse("2019-01-31T00:00:00");
        requests[12].timeTo = DateTime.Parse("2019-02-05T00:00:00");
        requests[12].priority = 3;
        requests[12].Max_SOEN_anlge = 0.9599310885968813;

        requests[13].timeFrom = DateTime.Parse("2018-03-01T13:27:00");
        requests[13].timeTo = DateTime.Parse("2018-03-03T13:27:00");
        requests[13].priority = 2;
        requests[13].Max_SOEN_anlge = 0.8726646259971648;

        requests[14].timeFrom = DateTime.Parse("2018-02-26T19:22:00");
        requests[14].timeTo = DateTime.Parse("2018-03-13T19:22:00");
        requests[14].priority = 3;
        requests[14].Max_SOEN_anlge = 0.5235987755982988;
       
            requests.Add(new RequestParams(i: 310, p: 3, d1: DateTime.Parse("2019-01-31T00:00:00"), d2: DateTime.Parse("2019-02-02T00:00:00"), max_a: 0.95993108859688125, min_p: 70.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((126.35281885204118 -15.644335352003495, 126.32209465981855 -15.728749376366125, 126.40978114795878 -15.760664647996506, 126.44050534018143 -15.676250623633877, 126.35281885204118 -15.644335352003495))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 311, p: 2, d1: DateTime.Parse("2019-01-31T00:00:00"), d2: DateTime.Parse("2019-02-02T00:00:00"), max_a: 0.95993108859688125, min_p: 80.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((121.83117292297347 -33.45740031865339, 121.64682776963774 -33.96388446482916, 122.13442233635772 -34.141354373505145, 122.31876748969344 -33.63487022732936, 121.83117292297347 -33.45740031865339))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 313, p: 1, d1: DateTime.Parse("2019-02-01T00:00:00"), d2: DateTime.Parse("2019-02-03T00:00:00"), max_a: 0.95993108859688125, min_p: 60.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((109.48626654494377 19.5594462504705, 109.36336977605328 19.22179015301998, 109.45285099845553 19.18922165153981, 109.57574776734603 19.52687774899033, 109.48626654494377 19.5594462504705))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 314, p: 2, d1: DateTime.Parse("2019-01-31T00:00:00"), d2: DateTime.Parse("2019-02-02T00:00:00"), max_a: 0.95993108859688125, min_p: 75.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((104.68623366692623 -2.84175930698035, 104.62478528248099 -3.0105873557055958, 104.79383841043744 -3.0721176622913617, 104.85528679488269 -2.903289613566102, 104.68623366692623 -2.84175930698035))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 315, p: 3, d1: DateTime.Parse("2019-01-31T00:00:00"), d2: DateTime.Parse("2019-02-03T00:00:00"), max_a: 0.95993108859688125, min_p: 80.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((85.28167344568055 27.77346555952296, 85.25094925345793 27.689051535160345, 85.34630207406599 27.65434594670569, 85.37702626628858 27.738759971068305, 85.28167344568055 27.77346555952296))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 316, p: 2, d1: DateTime.Parse("2019-01-31T00:00:00"), d2: DateTime.Parse("2019-02-02T00:00:00"), max_a: 0.95993108859688125, min_p: 60.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((85.62309186053487 20.819531710129908, 85.43874670719914 20.313047563954115, 85.87129312860758 20.155613541623083, 86.05563828194332 20.66209768779889, 85.62309186053487 20.819531710129908))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 317, p: 3, d1: DateTime.Parse("2019-01-31T00:00:00"), d2: DateTime.Parse("2019-02-05T00:00:00"), max_a: 0.95993108859688125, min_p: 70.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((80.52680145298038 6.062894530438157, 80.48993242231323 5.961597701203004, 80.59178604790101 5.924526013236985, 80.62865507856816 6.02582284247211, 80.52680145298038 6.062894530438157))", sT: 1, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 318, p: 1, d1: DateTime.Parse("2019-01-31T00:00:00"), d2: DateTime.Parse("2019-02-02T00:00:00"), max_a: 0.95993108859688125, min_p: 50.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((28.150485999492147 -14.948406577516423, 27.90469246171119 -15.623718772417476, 28.07973905691767 -15.687430522682277, 28.325532594698632 -15.012118327781238, 28.150485999492147 -14.948406577516423))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 319, p: 1, d1: DateTime.Parse("2019-01-31T00:00:00"), d2: DateTime.Parse("2019-02-03T00:00:00"), max_a: 0.95993108859688125, min_p: 60.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((8.30301122730751 5.128774112168699, 8.24156284286227 4.959946063443454, 8.411039289178655 4.898261681575065, 8.472487673623894 5.067089730300339, 8.30301122730751 5.128774112168699))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 320, p: 2, d1: DateTime.Parse("2019-01-31T00:00:00"), d2: DateTime.Parse("2019-02-02T00:00:00"), max_a: 0.95993108859688125, min_p: 50.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((-2.5414649965255296 53.86834241621756, -2.7433738935640104 53.368600359569825, -2.183205190113302 53.142277512483645, -1.9812962930748217 53.64201956913138, -2.5414649965255296 53.86834241621756))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 321, p: 2, d1: DateTime.Parse("2019-01-31T00:00:00"), d2: DateTime.Parse("2019-02-03T00:00:00"), max_a: 0.78539816339744828, min_p: 60.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((-19.227843372656842 65.35817995896684, -19.320015949324713 65.10493788587891, -18.91815182702349 64.95867130714174, -18.82597925035562 65.2119133802297, -19.227843372656842 65.35817995896684))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 322, p: 3, d1: DateTime.Parse("2019-01-31T00:00:00"), d2: DateTime.Parse("2019-02-04T00:00:00"), max_a: 0.95993108859688125, min_p: 60.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((-46.73357579241549 -23.369637743068665, -46.82574836908334 -23.62287981615657, -46.549505262271985 -23.723424084457108, -46.45733268560412 -23.470182011369232, -46.73357579241549 -23.369637743068665))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 323, p: 3, d1: DateTime.Parse("2019-01-31T00:00:00"), d2: DateTime.Parse("2019-02-05T00:00:00"), max_a: 0.95993108859688125, min_p: 70.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((-53.38412914720887 48.58173644618182, -53.44557753165411 48.41290839745656, -53.31824817212707 48.36656430064056, -53.25679978768183 48.53539234936582, -53.38412914720887 48.58173644618182))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 330, p: 2, d1: DateTime.Parse("2018-03-01T13:27:00"), d2: DateTime.Parse("2018-03-03T13:27:00"), max_a: 0.87266462599716477, min_p: 10.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((126.62480592378415 -14.31823303125367, 126.64700746187006 -14.305272850284837, 126.63353204377925 -14.294959615999502, 126.61570787080565 -14.288369433364153, 126.61297170945132 -14.307982784865686, 126.62480592378415 -14.31823303125367))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 331, p: 3, d1: DateTime.Parse("2018-02-26T19:22:00"), d2: DateTime.Parse("2018-03-13T19:22:00"), max_a: 0.52359877559829882, min_p: 10.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((9.13324843996645 5.912918992576621, 9.11764935876068 5.824452206931596, 9.206581004918789 5.808771158275093, 9.22218008612456 5.897237943920118, 9.13324843996645 5.912918992576621))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 321, p: 2, d1: DateTime.Parse("2019-01-31T00:00:00"), d2: DateTime.Parse("2019-02-03T00:00:00"), max_a: 0.78539816339744828, min_p: 60.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((73.166398746769 83.3795342613812,69.1157648056298 78.3986011694295,69.1017292969888 77.8679160512509,82.572775740474 76.204628990747,82.7895030334302 76.948348271668,82.8510351925494 77.29836274115,77.2353040566382 83.8046777874499,73.166398746769 83.3795342613812))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 322, p: 3, d1: DateTime.Parse("2019-01-31T00:00:00"), d2: DateTime.Parse("2019-02-04T00:00:00"), max_a: 0.95993108859688125, min_p: 60.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((-165.21673846612 -54.0589138057729,-165.573261002891 -54.9705389496409,-165.679803494884 -56.4150844568433,-165.655215606378 -56.6166901082156,-165.21673846612 -54.0589138057729))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 323, p: 3, d1: DateTime.Parse("2019-01-31T00:00:00"), d2: DateTime.Parse("2019-02-05T00:00:00"), max_a: 0.95993108859688125, min_p: 70.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((106.95738018444 -30.609773998998,106.700176507603 -33.064421050007,110.603738797998 -36.72736549789,111.040933447857 -35.3975779951589,106.95738018444 -30.609773998998))", sT: 0, comp: 10, alb: 0.0));
            requests.Add(new RequestParams(i: 330, p: 2, d1: DateTime.Parse("2018-03-01T13:27:00"), d2: DateTime.Parse("2018-03-03T13:27:00"), max_a: 0.87266462599716477, min_p: 10.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((126.708643529393 -5.18327756706429,126.739746677109 -5.39253384427269,138.433146631633 -3.22822909968528,137.855654937064 -2.76997099530497,137.340205104196 -2.46307396240683,126.708643529393 -5.18327756706429))", sT: 0, comp: 10, alb: 0.0));          
            */


          //  requests.Add(new RequestParams(i: 331, p: 3, d1: DateTime.Parse("2018-02-26T19:22:00"), d2: DateTime.Parse("2019-03-13T19:22:00"), max_a: 0.52359877559829882, min_p: 10.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((32.185089 29.690301, 32.185089 33.690301, 28.185089 33.690301, 28.185089 29.690301, 32.185089 29.690301))", sT: 0, comp: 10, alb: 0.0));
            //requests.Add(new RequestParams(i: 331, p: 3, d1: DateTime.Parse("2018-02-26T19:22:00"), d2: DateTime.Parse("2019-03-13T19:22:00"), max_a: 0.52359877559829882, min_p: 10.0, max_s_a: 90, min_s_a: 10, polygon: "POLYGON((33.690301 28.185089, 33.690301 32.185089, 29.690301 32.185089, 29.690301 28.185089, 33.690301 28.185089))", sT: 0, comp: 10, alb: 0.0));


            //var res = Sessions.getCaptureConfArray(requests, dt1, dt2, managerDB, new List<Tuple<DateTime, DateTime>>(), new List<Tuple<DateTime, DateTime>>());
            
            //var rnd = new Random();
            //var result = res.OrderBy(item => rnd.Next());
            //res = result.ToList<CaptureConf>();
            //DateTime end = DateTime.Now;
            // Console.WriteLine("total time = " + (end - start).TotalSeconds.ToString() + " ceк" );

            //foreach (var conf in res)
            //{
            //    captureIntervals.Add(new Polygon(conf.wktPolygon));
            //}
            //Console.WriteLine("res.Count = {0}", res.Count());
            
            //return res;
          

            #endregion

            List<Tuple<DateTime, DateTime>> silenceRanges = new List<Tuple<DateTime, DateTime>>();
            List<Tuple<DateTime, DateTime>> inactivityRanges = new List<Tuple<DateTime, DateTime>>();
            
            //DateTime dt1 = new DateTime(2019, 1, 4);
            //DateTime dt2 = new DateTime(2019, 1, 8);

            List<MPZ> mpzArray;
            List<CommunicationSession> sessions;

            Order order = new Order();
            order.captured = new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))");
            order.intersection_coeff = 0.1;
            order.request = new RequestParams(1, 1, dt1, dt2, AstronomyMath.ToRad(45), 0.4, 1, 1, "POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))");             
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

            requests = new List<RequestParams>();
            int id = 0;
            foreach (var pol in polygons)
            {
                RequestParams reqparams = new RequestParams(id, 1, dt1, dt2, AstronomyMath.ToRad(45), 0.4, 1, 1, pol.ToWtk());
                requests.Add(reqparams);
                id++;
            }
            var res = Sessions.getCaptureConfArray(requests, dt1, dt2, managerDB, new List<TimePeriod>(), new List<TimePeriod>());

            return null;

            DataFetcher fetcher = new DataFetcher(managerDB);
            Trajectory traj = fetcher.GetTrajectorySat(dt1,dt2);

            List<Polygon> allpols = new List<Polygon>();
            foreach(var conf in res)
            {
                SatLane strip = new SatLane(traj, conf.rollAngle, OptimalChain.Constants.camera_angle);
                allpols.Add(strip.getSegment(conf.dateFrom, conf.dateTo));
             }


            Console.Write("GEOMETRYCOLLECTION(");

            Console.WriteLine( requests[0].wktPolygon );
            
            Console.Write(",");

            Console.WriteLine(Polygon.getMultipolFromPolygons(allpols));
            Console.Write(")");


            return null;

            //string s = "POLYGON((126.62833380556265 -14.265775772684364, 126.61297170945132 -14.307982784865686, 126.6565272839827 -14.323835717531452, 126.671889380094 -14.281628705350116, 126.62833380556265 -14.265775772684364))";
            //RequestParams rp = new RequestParams(i : 301, p: 3, d1: dt1, d2: dt2, max_a: 55 * Math.PI / 180, min_p: 50, max_s_a: 10, min_s_a: 90, polygon: s, alb : 0.36, comp : 10, sT : 0);
            
            
            //            DataFetcher fetcher = new DataFetcher(managerDB);
            //Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

            //SatLane sl = new SatLane(trajectory,   0.012167999999999603, OptimalChain.Constants.camera_angle);
            //CaptureConf cccc = sl.getCaptureConfs(rp)[0];


            //List<Polygon> allPols = new List<Polygon>();
            ////  allPols.Add(sl.Sectors[0].polygon);
            
            ////  allPols.Add(sl.getSegment(cccc.dateFrom, cccc.dateTo));

            //Trajectory trajectory2 = fetcher.GetTrajectorySat(cccc.dateFrom, cccc.dateTo);
            //SatLane sl2 = new SatLane(trajectory2, 0.012167999999999603, OptimalChain.Constants.camera_angle);
            //allPols.Add(sl2.Sectors[0].polygon);


            //Console.Write("GEOMETRYCOLLECTION(");
            //Console.WriteLine(Polygon.getMultipolFromPolygons(allPols));
            //Console.Write(",");
            //Console.WriteLine(getWKTTrajectory(trajectory2));
            //Console.Write(",");
            //Console.Write("POLYGON((126.62833380556265 -14.265775772684364,126.61297170945132 -14.307982784865686,126.6565272839827 -14.323835717531452,126.671889380094 -14.281628705350116,126.62833380556265 -14.265775772684364))");
            //Console.Write(")");
           
            //Console.WriteLine();

            ////IList<Polygon> intersections = Polygon.Intersect(sl.Sectors[0].polygon, new Polygon(s));

            //return null;
                        
          //   Sessions.getMPZArray(new List<RequestParams> {rp}, dt1, dt2
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
