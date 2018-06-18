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
using Constants = OptimalChain.Constants;

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
            Console.WriteLine("angle_12 = {0}; ", Vector3D.AngleBetween(pp1, pp2));
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
            TrajectoryPoint? point = fetcher.GetSinglePoint<SatTableFacade>(dtx);

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
            Vector3D sun = new Vector3D(-125889950.019308, 50789227.1698602, -56664005.2550829);
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
            Polygon sector = new Polygon(Verts, Apexes);
            var LitAndNot = SphericalGeom.Polygon.IntersectAndSubtract(sector, SphericalGeom.Polygon.Hemisphere(sun));
            //Console.WriteLine(sector.ToWtk());


            foreach (SphericalGeom.Polygon p in LitAndNot.Item1)
                foreach (Polygon piece in p.BreakIntoLobes())
                    Console.WriteLine(piece.ToWtk());
        }




        void tedfdfsdfdfst()
        {
            Polygon test = new Polygon("POLYGON((-30.7177734375 -62.99515845212052,-29.443359375 -61.037012232401864,-27.4658203125 -59.445075099047145,-25.576171875 -58.90464570301999,-22.939453125 -58.63121664342478,-18.984375 -58.85922354706658,-15.2490234375 -59.7120971733229,-11.0302734375 -60.17430626192603,-7.5585937500000036 -59.7120971733229,-6.459960937499999 -58.539594766640484,-5.844726562499999 -56.944974180851595,-6.152343749999999 -55.70235509327092,-6.459960937499999 -55.22902305740633,-7.03125 -54.826007999094955,-6.943359374999999 -54.79435160392048,-6.361083984375 -55.200818422433024,-6.064453124999999 -55.68377855290112,-5.77056884765625 -56.93298739609703,-6.306152343749999 -58.58543569119917,-7.426757812499998 -59.833775202184206,-11.0302734375 -60.31606836555202,-15.314941406249998 -59.86688319521021,-19.00634765625 -58.97266715450152,-22.91748046875 -58.722598828043374,-25.510253906250007 -58.995311187950925,-27.312011718750004 -59.51202938650269,-29.24560546875 -61.06891658627741,-30.454101562499993 -63.0450010154201,-30.7177734375 -62.99515845212052))");
            var line = test.getCenterLine();

            Console.WriteLine("GEOMETRYCOLLECTION(");
            Console.WriteLine(WktTestingTools.getLineSringStr(line));
            Console.WriteLine(",");
            Console.WriteLine(test.ToWtk());
            Console.WriteLine(")");
        }

        void tedfdfssdfsdfsdfdfdfst()
        {
            ////Polygon test = new Polygon("POLYGON((-30.7177734375 -62.99515845212052,-29.443359375 -61.037012232401864,-27.4658203125 -59.445075099047145,-25.576171875 -58.90464570301999,-22.939453125 -58.63121664342478,-18.984375 -58.85922354706658,-15.2490234375 -59.7120971733229,-11.0302734375 -60.17430626192603,-7.5585937500000036 -59.7120971733229,-6.459960937499999 -58.539594766640484,-5.844726562499999 -56.944974180851595,-6.152343749999999 -55.70235509327092,-6.459960937499999 -55.22902305740633,-7.03125 -54.826007999094955,-6.943359374999999 -54.79435160392048,-6.361083984375 -55.200818422433024,-6.064453124999999 -55.68377855290112,-5.77056884765625 -56.93298739609703,-6.306152343749999 -58.58543569119917,-7.426757812499998 -59.833775202184206,-11.0302734375 -60.31606836555202,-15.314941406249998 -59.86688319521021,-19.00634765625 -58.97266715450152,-22.91748046875 -58.722598828043374,-25.510253906250007 -58.995311187950925,-27.312011718750004 -59.51202938650269,-29.24560546875 -61.06891658627741,-30.454101562499993 -63.0450010154201,-30.7177734375 -62.99515845212052))");
            //Polygon test = new Polygon("POLYGON((157.36679077148438 -6.858258512427739,157.54600524902347 -6.805762241482228,157.8865814208984 -6.931198262659578,157.80487060546872 -7.07568095750247,157.7080535888672 -7.091353264512179,157.47390747070312 -7.0927160486314165,157.4677276611328 -7.03683860125858,157.48558044433594 -7.032068262865209,157.50274658203128 -7.035475652433021,157.51441955566406 -7.039564486902762,157.54188537597656 -7.042971821437291,157.56385803222656 -7.047742047725649,157.58651733398438 -7.048423504616636,157.6373291015625 -7.049786415392134,157.6558685302734 -7.048423504616636,157.67852783203125 -7.046379130937709,157.6984405517578 -7.041608890626904,157.71835327148438 -7.0327497427788614,157.7355194091797 -7.021845944232467,157.74032592773435 -7.008897351515117,157.7423858642578 -6.993222257973443,157.7362060546875 -6.980272870827221,157.7197265625 -6.965278395972945,157.70118713378906 -6.96050732611333,157.67166137695312 -6.955736207749922,157.64144897460938 -6.946875430788481,157.5933837890625 -6.935288009015181,157.5652313232422 -6.930516634816186,157.54188537597656 -6.90461404723807,157.52197265624997 -6.895070630903007,157.49313354492188 -6.887572097488075,157.45811462402347 -6.8889354760209045,157.43064880371094 -6.89166222132711,157.40180969238278 -6.89166222132711,157.38052368164062 -6.8889354760209045,157.3681640625 -6.880073445529206,157.36473083496094 -6.8705295354242395,157.36679077148438 -6.858258512427739))");


            //GeoPoint[] gPoints = test.Vertices.Select(vert => GeoPoint.FromCartesian(vert)).ToArray();

            ////TriangleNet.Geometry.Contour contour = new TriangleNet.Geometry.Contour(gPoints.Select(gp => new TriangleNet.Geometry.Vertex(gp.Latitude, gp.Longitude)));
            ////TriangleNet.Geometry.IPolygon rtiPol = new TriangleNet.Geometry.Polygon();
            ////rtiPol.Add(contour);
            ////var options = new TriangleNet.Meshing.ConstraintOptions();
            ////var quality = new TriangleNet.Meshing.QualityOptions();
            ////quality.MaximumAngle = 20;
            ////TriangleNet.Mesh mesh = (TriangleNet.Mesh)TriangleNet.Geometry.ExtensionMethods.Triangulate(rtiPol, options, quality);

            //List<GeoPoint> centres = new List<GeoPoint>(mesh.NumberOfEdges);

            //foreach (var edge in mesh.Edges)
            //{
            //    if (Math.Abs(edge.P0 - edge.P1) == 1)
            //        continue;
            //    GeoPoint a = gPoints[edge.P0];
            //    GeoPoint b = gPoints[edge.P1];
            //    double dlat = b.Latitude - a.Latitude;
            //    double dlon = b.Longitude - a.Longitude;
            //    GeoPoint c = new GeoPoint(a.Latitude + dlat / 2, a.Longitude + dlon / 2);
            //    centres.Add(c);
            //}


            //Console.WriteLine("GEOMETRYCOLLECTION(");
            //Console.WriteLine(getPointsStr(centres));
            //Console.WriteLine(",");
            //Console.WriteLine(test.ToWtk());
            //Console.WriteLine(")");

        }

        public void test_isRequestFeasible()
        {
            //string wktPol = "POLYGON((-375.9521484375 40.44694705960049,-374.80957031249994 40.71395582628605,-374.06249999999994 41.27780646738182,-373.359375 42.09822241118974,-372.48046875 42.58544425738492,-370.546875 42.61779143282345,-368.2177734375 42.13082130188812,-367.119140625 41.0130657870063,-367.470703125 39.707186656826565,-369.755859375 39.06184913429155,-372.26074218749994 38.95940879245421,-374.2822265625 39.30029918615028,-375.64453124999994 39.09596293630548,-376.78710937499994 37.96152331396614,-376.83105468749994 36.914764288955936,-376.3916015625 35.63944106897394,-375.16113281249994 34.488447837809304,-372.52441406249994 33.54139466898276,-369.88769531249994 33.28461996888768,-367.1630859375 33.247875947924385,-363.9990234375 33.651208299204995,-360.65917968749994 34.27083595165,-358.76953125 35.38904996691166,-357.9345703125 36.914764288955936,-357.9345703125 38.134556577054155,-358.330078125 39.23225314171492,-360.263671875 39.97712009843963,-362.548828125 40.17887331434699,-363.20800781249994 39.707186656826565,-361.7138671875 39.23225314171492,-360.52734375 38.685509760012025,-359.47265624999994 38.09998264736481,-359.51660156249994 36.844460740795625,-360.703125 35.46066995149532,-363.25195312499994 34.6332079113796,-367.3828125 34.01624188966704,-370.8544921875 34.30714385628805,-374.501953125 35.42486791930558,-375.64453124999994 36.703659597194545,-375.24902343749994 38.134556577054155,-373.1396484375 38.44498466889473,-370.107421875 38.169114135560875,-367.734375 38.27268853598099,-366.15234374999994 39.1300602421351,-365.8447265625 40.27952566881291,-365.75683593749994 41.244772343082076,-366.7236328125 42.22851735620853,-368.04199218749994 43.06888777416961,-370.98632812499994 43.2612061247998,-372.65625 43.357138222110535,-374.06249999999994 43.10098287618854,-374.677734375 42.22851735620853,-375.42480468749994 41.541477666790286,-376.5234375 41.57436130598913,-376.25976562499994 42.22851735620853,-377.97363281249994 42.293564192170095,-377.62207031249994 41.73852846935915,-379.2919921875 41.80407814427238,-380.39062499999994 42.391008609205045,-381.00585937499994 42.650121813680244,-381.97265625 42.293564192170095,-380.39062499999994 41.672911819602064,-379.4677734375 40.88029480552822,-377.88574218749994 40.74725696280419,-375.9521484375 40.44694705960049))";
            string wktPol = "POLYGON((-329.710693359375 30.63791202834112,-329.600830078125 30.85507928696859,-329.32617187500006 30.958768570779867,-328.95263671875 30.96818929679425,-328.67797851562506 30.883369321692243,-328.480224609375 30.770159115784196,-328.22753906250006 30.533876572997627,-327.96386718750006 30.25906720321302,-327.689208984375 29.94541533710442,-327.44750976562506 29.602118211647337,-327.27172851562506 29.276816328368568,-327.095947265625 28.940861769405572,-326.84326171874994 28.652030630362262,-326.5576171875 28.497660832963476,-325.887451171875 28.343064904825496,-325.48095703125 28.21728975595707,-325.404052734375 28.101057958669443,-325.4397583007812 28.09378928277097,-325.5180358886718 28.202767685948402,-325.7638549804687 28.271729925419976,-326.0893249511719 28.35998482403741,-326.42715454101557 28.424014366515777,-326.60293579101557 28.48196998767908,-326.75262451171875 28.555576049185973,-326.8954467773437 28.647210004919998,-327.06848144531244 28.837455983116214,-327.15225219726557 28.948072282131648,-327.2044372558593 29.048966726566917,-327.3143005371093 29.284003336047178,-327.42004394531244 29.440793523459092,-327.52441406249994 29.660609413340282,-327.6274108886718 29.79417590643662,-327.7619934082032 29.972780616663897,-327.8828430175782 30.113057804059494,-328.04351806640625 30.292274851024246,-328.2536315917969 30.50666709259758,-328.3950805664063 30.64145672280776,-328.4706115722657 30.714684645247488,-328.5639953613282 30.780778241596465,-328.6381530761719 30.82560162443589,-328.7123107910157 30.86333140957521,-328.82080078125 30.89868960357603,-328.92105102539057 30.929322813120706,-329.03778076171864 30.942280064232108,-329.21218872070307 30.930500817607793,-329.29458618164057 30.929322813120706,-329.36187744140625 30.915185627408306,-329.4525146484375 30.884547891921983,-329.54315185546875 30.838572911522647,-329.58984374999994 30.79375558121771,-329.5994567871093 30.74537659824557,-329.64752197265625 30.668628397433352,-329.6818542480468 30.61427741282776,-329.710693359375 30.63791202834112))";

            Polygon pol = new Polygon(wktPol);

            List<GeoPoint> line = pol.getCenterLine();

            Console.WriteLine("GEOMETRYCOLLECTION(");
            Console.WriteLine(WktTestingTools.getLineSringStr(line));
            Console.WriteLine(",");
            Console.WriteLine(wktPol);
            Console.WriteLine(")");

            return;

            //tedfdfssdfsdfsdfdfdfst();
            //return;

            //for (int testi = 0; testi < 2; testi++)
            //{

            //    List<Polygon> polygons = new List<Polygon>();
            //    Random rand = new Random((int)DateTime.Now.Ticks);
            //    for (int i = 0; i < 20; i++)
            //    {
            //        Polygon randpol = getRandomPolygon(rand, 3, 12, 2, 8);
            //        polygons.Add(randpol);
            //    }

            //    string cs = System.IO.File.ReadLines("DBstring.conf").First();
            //    DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);

            //    DateTime dt1 = new DateTime(2019, 1, 5);
            //    DateTime dt2 = new DateTime(2019, 1, 6);

            //    DataFetcher fetcher = new DataFetcher(manager);
            //    Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

            //    if (trajectory.Count == 0)
            //        throw new Exception("На эти даты нет траектории в БД, тест некорректный");


            //    foreach (var pol in polygons)
            //    {
            //        //try
            //        //{
            //        RequestParams reqparams = new RequestParams(0, 1, dt1, dt2, AstronomyMath.ToRad(45), 0.4, 1, 1, pol.ToWtk());

            //        double cover;
            //        List<CaptureConf> output;
            //        Sessions.isRequestFeasible(reqparams, dt1, dt2, manager, out cover, out output);
            //        //}
            //        //catch (Exception ex)
            //        //{
            //        //    List<string> lines = new List<string>();
            //        //    Console.WriteLine("Ошибка обнаружена на следующем полигонt:");
            //        //    Console.WriteLine(pol.ToWtk());
            //        //    lines.Add(pol.ToWtk());                        
            //        //    System.IO.File.WriteAllLines(@"badPolygons.txt", lines);
            //        //    throw ex;
            //        //}

            //    }
            //}


            //// if (curRequest == null)
            // return;

            ///12.03.2015
            //  по 
            //14.03.2015

            //DateTime dt1 = new DateTime(2018, 4, 23);
            //DateTime dt2 = new DateTime(2018, 4, 24);



            //23-24 04 2018

            DateTime dt1 = new DateTime(2019, 2, 4);
            DateTime dt2 = new DateTime(2019, 2, 6);


            //Polygon pol = new Polygon("POLYGON ((-126.734018422276 -22.0749545024952,-130.817004196723 -24.9951369749654,-127.160780596832 -29.8248985732365,-120.731159883255 -28.8811236147716,-119.107074097174 -26.7711150394608,-119.002845016367 -25.8737550177619,-119.34506718841 -24.6296253187895,-119.70825315869 -24.0675537068877,-122.608344032093 -22.2398046264519,-126.734018422276 -22.0749545024952))");

            RequestParams reqparams = new RequestParams(0, 1, dt1, dt2, AstronomyMath.ToRad(45), 0.4, 1, 1,
                "POLYGON((20.390624999999996 24.766784522874445,20.566406249999993 26.824070780470166,19.33593749999999 27.137368359795587,18.281249999999993 25.72073513441211,20.390624999999996 24.766784522874445))"
                );


            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);

            DataFetcher fetcher = new DataFetcher(managerDB);
            Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);
            double isfes;
            List<CaptureConf> possibleConfs;
            Sessions.isRequestFeasible(reqparams, dt1, dt2, managerDB, out isfes, out possibleConfs);
            Console.WriteLine(isfes);
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
            string wkt2 = Sessions.getSOENViewPolygon(dt1, rollAngle: Math.PI / 6, pitchAngle: 0, duration: 260, managerDB: managerDB, isCoridor: true);


            polygons.Add(new Polygon(wkt1));
            polygons.Add(new Polygon(wkt2));


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

            Console.WriteLine(WktTestingTools.getWKTInterpolateTrajectory(trajectory, segmdt1, segmdt2, step));

            Console.Write(",");

            Console.WriteLine(WktTestingTools.getWKTViewInterpolPolygons(trajectory, segmdt1, segmdt2, rollAngle, pitchAngle, step));

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
            for (int i = 0; i < count; i++)
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

            for (int i = 0; i < count; i += 2)
            {
                int j = i / 2;
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
            TrajectoryPoint pp1 = (TrajectoryPoint)fetcher.GetSinglePoint<SatTableFacade>(dtMid);

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
                double rollCor = CaptureConf.getRollCorrection(height, velo, AstronomyMath.ToRad(kaGeoPoint.Latitude), pitch);

                Vector3D surfPoint = LanePos.getSurfacePoint(pp1, rollAngle + rollCor, pitch);
                rollCorrPitchLine.Add(surfPoint);
                Console.WriteLine(rollCor);
            }

            //GeoPoint p1 = new GeoPoint(-49.01265386395501, -14.562377929687498);
            //GeoPoint p2 = new GeoPoint(-49.11343354595957, -14.974365234374996);
            //Console.WriteLine("err = {0}", GeoPoint.DistanceOverSurface(p1, p2) * Astronomy.Constants.EarthRadius);




            Polygon centre = getPointPolygon(LanePos.getSurfacePoint(pp1, rollAngle, 0));

            Console.Write("GEOMETRYCOLLECTION(");
            Console.WriteLine(WktTestingTools.getWKTLinePoints(pitchLine));
            Console.WriteLine(",");
            Console.WriteLine(WktTestingTools.getWKTLinePoints(rollCorrPitchLine));
            Console.WriteLine(",");
            Console.WriteLine(WktTestingTools.getWKTLinePoints(nopitchLine));
            Console.WriteLine(",");
            Console.WriteLine(WktTestingTools.getWKTTrajectory(trajectory));
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


            Console.WriteLine(WktTestingTools.getWKTViewInterpolPolygons(trajectory, segmdt1, segmdt2, rollAngle, pitchAngle, step));
            Console.Write(",");
            Console.WriteLine(Polygon.getMultipolFromPolygons(new List<Polygon>() { SatLane.getRollPitchLanePolygon(trajectory, rollAngle, pitchAngle) }));

            Console.Write(",");
            Console.WriteLine(WktTestingTools.getWKTViewInterpolPolygons(trajectory, segmdt1, segmdt2, -rollAngle, pitchAngle, step));
            Console.Write(",");
            Console.WriteLine(Polygon.getMultipolFromPolygons(new List<Polygon>() { SatLane.getRollPitchLanePolygon(trajectory, -rollAngle, pitchAngle) }));

            Console.Write(",");
            Console.WriteLine(WktTestingTools.getWKTViewInterpolPolygons(trajectory, segmdt1, segmdt2, rollAngle, -pitchAngle, step));
            Console.Write(",");
            Console.WriteLine(Polygon.getMultipolFromPolygons(new List<Polygon>() { SatLane.getRollPitchLanePolygon(trajectory, rollAngle, -pitchAngle) }));

            Console.Write(",");
            Console.WriteLine(WktTestingTools.getWKTViewInterpolPolygons(trajectory, segmdt1, segmdt2, -rollAngle, -pitchAngle, step));
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

            TrajectoryPoint pp1 = (TrajectoryPoint)fetcher.GetSinglePoint<SatTableFacade>(DateTime.Parse("01.02.2019 0:15:00"));

            Vector3D leftVector = LanePos.getDirectionVector(pp1, -0.793708, 0);
            Vector3D rightVector = LanePos.getDirectionVector(pp1, 0.793708, 0);

            LanePos llp1 = new LanePos(pp1, OptimalChain.Constants.camera_angle, 0);


            Char separator = System.Globalization.CultureInfo.CurrentCulture.NumberFormat.CurrencyDecimalSeparator[0];
            int step = 5;

            Console.Write("GEOMETRYCOLLECTION(");

            Console.WriteLine(WktTestingTools.getWKTInterpolateTrajectory(trajectory, segmdt1, segmdt2, step));
            Console.Write(",");
            Console.WriteLine(WktTestingTools.getWKTViewLine(trajectory, segmdt1, segmdt2, rollAngle, 0, step));
            Console.Write(",");
            Console.WriteLine(WktTestingTools.getWKTStripSegment(trajectory, segmdt1, segmdt2, rollAngle));
            Console.Write(",");
            Console.WriteLine(WktTestingTools.getWKTViewInterpolPolygons(trajectory, segmdt1, segmdt2, rollAngle, 0, step));
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
            RequestParams rp = new RequestParams(_id: 301, _priority: 3, _timeFrom: dt1, _timeTo: dt2, _Max_SOEN_anlge: 55 * Math.PI / 180, _minCoverPerc: 50, _Max_sun_angle: 10, _Min_sun_angle: 90, _wktPolygon: testpolstr, _albedo: 0.36, _compression: 10, _shootingType: 0);


            Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

            SatLane lane = new SatLane(trajectory, rollAngle, OptimalChain.Constants.camera_angle);

            List<CaptureConf> res = lane.getCaptureConfs(rp);

            //POINT(30.12333691120147 -42.379585265899514),POINT(30.12430787086486 -42.379676407283156))
            GeoPoint gp1 = new GeoPoint(-42.379585265899514, 30.12333691120147);
            GeoPoint gp2 = new GeoPoint(-42.379676407283156, 30.12430787086486);
            Console.WriteLine(GeoPoint.DistanceOverSurface(gp1, gp2));

            Console.Write("GEOMETRYCOLLECTION(");

            Console.WriteLine(WktTestingTools.getWKTStrip(trajectory, rollAngle));
            Console.Write(",");
            Console.WriteLine(WktTestingTools.getWKTViewLine(trajectory, res[0].dateFrom, res[0].dateTo, rollAngle, 0, step: 5));
            Console.Write(",");
            Console.WriteLine(WktTestingTools.getWKTStripSegment(trajectory, res[0].dateFrom, res[0].dateTo, rollAngle));
            Console.Write(",");
            Console.WriteLine(testpolstr);




            //Console.WriteLine(WktTestingTools.WktTestingTools.getWKTStripSegment(trajectory, segmdt1, segmdt2, rollAngle));                   

            Console.Write(")");

        }



        public void test_Booblick2()
        {
            DateTime dt1 = DateTime.Parse("01.02.2019 00:00:00");
            DateTime dt2 = DateTime.Parse("01.02.2019 00:59:00");

            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
            DataFetcher fetcher = new DataFetcher(managerDB);


            string wkt1 = "POLYGON ((-15.7763671875 -57.5158228655388,-15.7476677787646 -57.5143790621652,-16.5673828125 -56.2921566850765,-19.6435546875 -57.4685891920893,-16.9278227030282 -57.9963198478714,-15.7763671875 -57.5158228655388))";
            string wkt2 = "POLYGON((-16.5673828125 -59.3107679560388,-16.9498671881788 -58.8959440932022,-19.423828125 -59.6677405816496,-15.688476562499998 -60.6301017662667,-15.3803417658345 -59.2198858496667,-16.5673828125 -59.3107679560388))";
            string wkt3 = "POLYGON((-17.5341796875 -58.2401635434164,-16.9278227030282 -57.9963198478714,-19.6435546875 -57.46858919208929,-19.423828125 -59.6677405816496,-16.9498671881788 -58.89594409320231,-17.5341796875 -58.2401635434164))";


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

            DateTime dt1 = DateTime.Parse("01.02.2019 06:00:00");
            DateTime dt2 = DateTime.Parse("01.02.2019 08:00:00");

            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
            DataFetcher fetcher = new DataFetcher(managerDB);

            double rollAngle = AstronomyMath.ToRad(45);
            double pitchAngle = AstronomyMath.ToRad(30);
            double viewAngle = OptimalChain.Constants.camera_angle;
            Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

          //  string outPol = "POLYGON((-16.5673828125 -56.292156685076456,-12.0849609375 -56.68037378950137,-11.337890625 -59.265880628258095,-15.688476562499998 -60.63010176626667,-19.423828124999996 -59.66774058164962,-19.6435546875 -57.46858919208933,-16.5673828125 -56.292156685076456))";
          //  string intPol = "POLYGON((-15.776367187500002 -57.515822865538816,-17.534179687499996 -58.24016354341643,-16.5673828125 -59.31076795603883,-12.7001953125 -58.97266715450152,-13.403320312499998 -57.3739384187141,-15.776367187500002 -57.515822865538816))";

            string outPol =  "POLYGON((35.760498046875 56.662264768247184,34.573974609375 54.68018309710001,42.242431640625 54.47642158429295,42.71484375 56.67433841615883,35.760498046875 56.662264768247184))" ;

            List<string> holes = new List<string>() {
             

"POLYGON((36.5185546875 56.1333069123757,36.683349609375 56.492827145026666,36.92504882812501 56.365250136856105,37.430419921875 56.492827145026666,37.177734375 56.035225783698735,37.02392578125 56.06590296330043,37.06787109375001 56.26776108757582,36.815185546875 56.243349924105246,36.705322265625 56.07203547180089,36.5185546875 56.1333069123757))" 

,"POLYGON((37.89184570312501 56.42605447604976,37.628173828125 56.004524201154,37.84790039062501 55.973798205076605,38.04565429687501 56.365250136856105,38.56201171875 56.3409012041991,38.583984375 56.4199784113855,37.89184570312501 56.42605447604976))" 

,"POLYGON((38.91357421875 56.365250136856105,39.166259765625 56.12718415613108,38.792724609375 55.930740775711854,39.02343750000001 55.85681658243854,39.72656249999999 56.32872090717996,39.517822265625 56.32872090717996,39.29809570312501 56.243349924105246,38.91357421875 56.365250136856105))" 

,"POLYGON((40.92407226562501 56.32262930069561,40.836181640625 56.188367864753076,41.24267578125 56.19448087726974,41.275634765625 56.31044317134601,40.92407226562501 56.32262930069561))" 

,"POLYGON((36.35375976562501 55.652798033189555,36.309814453125 55.10980079314379,37.6611328125 55.36038057233307,37.6611328125 55.578344672182055,36.35375976562501 55.652798033189555))" 

,"POLYGON((41.49536132812499 56.79486226140054,43.01147460937501 55.96765007530669,44.176025390625 56.54737205307899,41.49536132812499 56.79486226140054))" 

,"POLYGON((35.419921875 55.63419794625841,35.343017578125 55.460170838618154,33.85986328124999 55.578344672182055,34.266357421875 55.986091533808406,35.419921875 55.63419794625841))" 

,"POLYGON((35.859375 56.07203547180089,35.66162109375 55.92458580482949,35.958251953125 55.83831352210822,36.49658203125 55.94919982336745,35.859375 56.07203547180089))" 

,"POLYGON((35.892333984375 56.517079019323745,35.79345703125 56.27996083172846,36.2109375 56.237244700410315,36.287841796875 56.492827145026666,35.892333984375 56.517079019323745))" 

,"POLYGON((35.2001953125 55.05320258537114,35.826416015625 54.977613670696286,35.716552734375 54.8386636129751,35.079345703125 54.85131525968609,35.2001953125 55.05320258537114))" 

,"POLYGON((38.95751953125001 55.05320258537114,39.17724609375 55.36038057233307,39.61669921875001 55.19768334019969,39.462890625 54.90188218738501,38.95751953125001 55.05320258537114))" 

,"POLYGON((40.83618164062499 55.64039895668736,40.616455078125 55.17886766328198,41.17675781249999 55.09723033442452,40.83618164062499 55.64039895668736))" 

,"POLYGON((39.517822265625 55.64659898563684,39.61669921874999 55.42901345240739,40.2099609375 55.410307210052196,40.53955078125 55.813629071199585,40.067138671875 55.94919982336745,39.75952148437499 55.677584411089526,39.517822265625 55.64659898563684))" 

,"POLYGON((38.265380859375 55.66519318443602,37.979736328125 55.522411831398216,38.48510742187499 55.4165436085801,39.056396484374986 55.547280698640805,38.265380859375 55.66519318443602))" 

,"POLYGON((38.12255859374999 54.79435160392052,37.716064453125 54.832336301970344,37.96875 55.09094362227856,38.46313476562499 55.01542594056298,38.12255859374999 54.79435160392052))" 

,"POLYGON((36.60644531249999 54.807017138462555,36.96899414062499 54.75633118164467,36.97998046875 54.457266680933856,36.397705078125 54.49556752187411,36.60644531249999 54.807017138462555))" 

,"POLYGON((42.76977539062499 55.03431871502809,41.80297851562499 54.876606654108684,42.07763671875 54.68653423452969,42.56103515625 54.699233528481386,42.76977539062499 55.03431871502809))" 

,"POLYGON((42.69287109374999 55.64039895668736,41.84692382812499 55.770393581620056,41.77001953125 55.441479359140686,43.121337890625 55.34788906283774,42.69287109374999 55.64039895668736))" 

,"POLYGON((40.24291992187499 56.72259433299854,40.18798828124999 56.53525774684846,40.3857421875 56.4806953901963,40.462646484375 56.72259433299854,40.24291992187499 56.72259433299854))" 

,"POLYGON((39.803466796875 54.88924640307587,40.67138671874999 54.85763959554899,40.58349609374999 54.80068486732233,39.83642578125 54.79435160392052,39.803466796875 54.88924640307587))" 

,"POLYGON((38.583984375 54.844989932187616,39.17724609375 54.82600799909497,39.17724609375 54.72462019492448,38.638916015625 54.74999097022689,38.583984375 54.844989932187616))" 

,"POLYGON((37.37548828125 54.69288437829769,37.76000976562499 54.64205540129177,37.705078125 54.48918653875083,37.265625 54.48918653875083,37.37548828125 54.69288437829769))" 

,"POLYGON((37.03491210937499 55.103516058019665,37.13378906249999 54.99652425983251,37.705078125 54.990221720048964,37.73803710937499 55.13492985052767,37.03491210937499 55.103516058019665))" 

,"POLYGON((41.660156249999986 56.145549500679095,40.83618164062499 55.986091533808406,40.91308593749999 55.795105452236925,41.69311523437499 56.004524201154,41.660156249999986 56.145549500679095))" 

,"POLYGON((38.869628906249986 56.752722872057376,38.67187499999999 56.53525774684846,39.37499999999999 56.517079019323745,39.517822265624986 56.81290751870026,38.869628906249986 56.752722872057376))" 

,"POLYGON((41.17675781249999 54.85131525968609,40.96801757812499 54.69288437829769,41.28662109374999 54.64205540129177,41.31958007812499 54.80068486732236,41.17675781249999 54.85131525968609))" 

,"POLYGON((35.58471679687499 55.36038057233307,35.93627929687499 55.65899609942838,36.123046875 55.59697126798508,35.848388671875 55.178867663282006,35.628662109375 55.22275708802212,35.58471679687499 55.36038057233307))" 

,"POLYGON((38.353271484375 56.01680776320322,37.96875 55.91227293006361,38.18847656249999 55.83831352210822,38.353271484375 56.01680776320322))" 

,"POLYGON((37.02392578125 55.95535088453653,38.089599609375 55.90611502601163,38.309326171875 55.832143877813024,37.144775390625 55.825973254619015,37.02392578125 55.95535088453653))" 

,"POLYGON((38.287353515625 55.97379820507658,39.19921875 55.72092280778696,38.968505859375 55.71473455012688,38.287353515625 55.97379820507658))" 

,"POLYGON((40.07809037249999 55.31661025761744,39.83639136257812 55.128616289259156,40.49557048054688 55.128616289259156,40.5175431178125 55.541032161833186,40.07809037249999 55.31661025761744))" 

 
};


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

            Console.WriteLine(Polygon.getMultipolFromPolygons(lst.SelectMany(l => l.polygons).ToList()));
            
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
                if (conf.dateFrom == conf.dateTo)
                    allpols.Add(new SatelliteCoordinates(traj.GetPoint(conf.dateFrom), conf.rollAngle, 0).ViewPolygon);
                else
                    allpols.Add(strip.getSegment(conf.dateFrom, conf.dateTo));
            }
            Console.WriteLine();
            Console.WriteLine();
            Console.Write("GEOMETRYCOLLECTION(");
            Console.WriteLine(outPol);
            Console.Write(",");
            //Console.WriteLine(intPol);
            //Console.WriteLine(Polygon.getMultipolFromPolygons(lst.SelectMany(l => l.polygons).ToList()));
            Console.WriteLine(Polygon.getMultipolFromPolygons( holes.Select(h => new Polygon(h)).ToList()));
            Console.Write(",");
            Console.WriteLine(Polygon.getMultipolFromPolygons(allpols));
            Console.Write(")");
            Console.WriteLine();
            Console.WriteLine();
        }

        public void test_getStereoTripletMpzArray()
        {
            DateTime dt1 = new DateTime(2019, 2, 6, 0, 0, 0);
            DateTime dt2 = new DateTime(2019, 2, 6, 1, 0, 0);


            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);


            //DataFetcher fetcher = new DataFetcher(managerDB);
            //Trajectory traj = fetcher.GetTrajectorySat(dt1, dt2);
            //Console.WriteLine(WktTestingTools.getWKTStrip(traj, 0));

            string polwtk = "POLYGON((137.894051953125 -26.38104068955903,137.95725568359373 -26.376116965436545,137.96962163085936 -26.410578626363836,137.8954259472656 -26.420422925429968,137.894051953125 -26.38104068955903))";

            List<string> holes = new List<string>();

            RequestParams req = new RequestParams(0, 1, dt1, dt2,
                _Max_SOEN_anlge: AstronomyMath.ToRad(50),
                _minCoverPerc: 0.12,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: polwtk,
                _polygonToSubtract: holes, _requestChannel: ShootingChannel.ePK,
                _shootingType: ShootingType.ePlain);

            double coverage;
            List<CaptureConf> possibleConfs;
            Sessions.isRequestFeasible(req, dt1, dt2, managerDB, out coverage, out possibleConfs);

            return;
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
    , managerDB
    , 356
               , out mpzArray
               , out sessions);


            Console.WriteLine("res.Count = {0}", mpzArray.Count());


            Console.Write("GEOMETRYCOLLECTION(");
            Console.Write(polwtk);
            Console.Write(",");
            Console.Write(Polygon.getMultipolFromPolygons(mpzArray.SelectMany(mpz => mpz.Routes.Select(r => new Polygon(r.Parameters.ShootingConf.wktPolygon))).ToList()));
            Console.Write(")");
        }

        public void test_getPlainMpzArray()
        {
            DateTime dt1 = new DateTime(2019, 2, 6, 0, 0, 0);
            DateTime dt2 = new DateTime(2019, 2, 6, 21, 0, 0);
             
            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);


            //Sessions.getSunBlindingPeriods(dt1, dt2, managerDB);
            //return;
            string polwtk = "POLYGON((140.47668457031253 -17.623081791311762,139.603271484375 -17.30606566309359,139.43023681640625 -18.145851771694467,140.5865478515625 -18.19282519773317,140.47668457031253 -17.623081791311762))";

            List<string> holes = new List<string>();

            RequestParams req = new RequestParams(0, 1, dt1, dt2,
                _Max_SOEN_anlge: AstronomyMath.ToRad(50),
                _minCoverPerc: 0.12,
                _Max_sun_angle: 90,
                _Min_sun_angle: 10,
                _wktPolygon: polwtk,
                _polygonToSubtract: holes, _requestChannel: ShootingChannel.ePK,
                _shootingType: ShootingType.ePlain);

            //double coverage;
            //List<CaptureConf> possibleConfs;
            //Sessions.isRequestFeasible(req, dt1, dt2, managerDB, out coverage, out possibleConfs);
            //return;

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
            , managerDB
            , 356
            , out mpzArray
            , out sessions);

            Console.WriteLine("res.Count = {0}", mpzArray.Count());

            Console.Write("GEOMETRYCOLLECTION(");
            Console.Write(polwtk);
            Console.Write(",");
            Console.Write(  Polygon.getMultipolFromPolygons(mpzArray.SelectMany(mpz => mpz.Routes.Select(r => new Polygon(r.Parameters.ShootingConf.wktPolygon) )).ToList()   ));
            Console.Write(")");

            double roll = mpzArray.First().Routes.First().Parameters.ShootingConf.roll;
            double pitch = mpzArray.First().Routes.First().Parameters.ShootingConf.pitch;
            DateTime start = mpzArray.First().Routes.First().startTime;

            int duration = mpzArray.First().Routes.First().Troute * 200;
            string viewPol = Sessions.getSOENViewPolygon(start, roll, pitch, duration, managerDB, false);

            Console.Write(viewPol);



        }


        public void test_getCoridorMpzArray()
        {
            //DateTime dt1 = DateTime.Parse("07/01/2019 03:00:00");
            //DateTime dt2 = DateTime.Parse("17/01/2019 04:00:00");
            DateTime dt1 = DateTime.Parse("13.01.2019 8:0:41");
            DateTime dt2 = DateTime.Parse("13.01.2019 8:59:41");
            

            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
            DBTables.DataFetcher fetcher = new DBTables.DataFetcher(managerDB);

            
            //List<TimePeriod> shadowPeriods;
            //List<Tuple<int, List<wktPolygonLit>>> partsLitAndNot;
            //  Sessions.checkIfViewLaneIsLitWithTimeSpans(managerDB, dt1, dt2, out partsLitAndNot, out shadowPeriods);

            //return;

            string corydor_pol = "POLYGON ((31.4346313476563 31.7491897749571,31.2835693359375 31.7491897749571,31.1407470703125 31.7305032292844,31.0226440429687 31.6674083170809,30.948486328125 31.4989456779629,30.948486328125 31.3630184912912,30.9841918945312 31.2198482262012,31.0693359375 31.1211457091563,31.234130859375 31.0388151041287,31.4456176757813 30.9658342024211,31.5362548828125 30.8409311390299,31.4895629882813 30.6780776242053,31.3906860351562 30.6000938735501,31.234130859375 30.5504351350953,31.080322265625 30.545704405481,31.0665893554687 30.4652467507532,31.1434936523437 30.4652467507532,31.278076171875 30.4818170028273,31.4593505859375 30.5362422547349,31.5966796875 30.588272671027,31.640625 30.713503990355,31.6433715820313 30.8126285847737,31.6543579101562 30.9045813670442,31.4813232421875 31.0552869290102,31.3604736328125 31.0999817937494,31.2368774414063 31.1446556363416,31.1517333984375 31.1963569577351,31.0830688476563 31.2527261968368,31.0391235351563 31.3348710339506,31.0308837890625 31.5012875211967,31.0693359375 31.6042705179912,31.2533569335937 31.6674083170809,31.4346313476563 31.6767584187955,31.4346313476563 31.7491897749571))";


      //      Console.WriteLine("GEOMETRYCOLLECTION(");
            //Console.WriteLine(new Polygon(corydor_pol).ToWtk());
            //Console.Write(",");
            //Console.WriteLine(WktTestingTools.getLineSringStr(new Polygon(corydor_pol).getCenterLine()));
            //Console.WriteLine(")");
            //return;

            RequestParams coridor_rp = new RequestParams(_id: 0, _priority: 3, _timeFrom: dt1, _timeTo: dt2, _Max_SOEN_anlge: AstronomyMath.ToRad(45), _minCoverPerc: 0.4, _Max_sun_angle: 1, _Min_sun_angle:  1,
                _wktPolygon: corydor_pol, _shootingType: ShootingType.eCorridor, _compression: 6);

            //double coverage;
            //List<CaptureConf> possibleConfs;
            //Sessions.isRequestFeasible(coridor_rp, dt1, dt2, managerDB, out coverage, out possibleConfs);

            //List<Polygon> allpols3 = possibleConfs.Select(cconf => cconf.orders.First().captured).ToList();

            //Console.Write("GEOMETRYCOLLECTION(");
            //Console.WriteLine(corydor_pol);
            //Console.Write(",");
            //Console.WriteLine(WktTestingTools.getLineSringStr(new Polygon(corydor_pol).getCenterLine()));
            //Console.Write(",");
            //Console.WriteLine(Polygon.getMultipolFromPolygons(allpols3));            
            //Console.Write(")");

          //  return;
     ///////////////////////////////////////////


            //var res = Sessions.getCaptureConfArray(new List<RequestParams> { coridor_rp }, dt1, dt2, managerDB, new List<TimePeriod>(), new List<TimePeriod>());

            //List<Polygon> allpols = res.Select(cconf => cconf.orders.First().captured).ToList();

            //Console.Write("GEOMETRYCOLLECTION(");
            //Console.WriteLine(corydor_pol);
            //Console.Write(",");
            //Console.WriteLine(WktTestingTools.getLineSringStr(new Polygon(corydor_pol).getCenterLine()));
            //Console.Write(",");
            //Console.WriteLine(Polygon.getMultipolFromPolygons(allpols));
            //Console.Write(")");

        //    return;

            List<Tuple<DateTime, DateTime>> silenceRanges = new List<Tuple<DateTime, DateTime>>();
            List<Tuple<DateTime, DateTime>> inactivityRanges = new List<Tuple<DateTime, DateTime>>();
            List<RouteMPZ> routesToDrop = new List<RouteMPZ>();
            List<RouteMPZ> routesToDelete = new List<RouteMPZ>();
            List<MPZ> mpzArray;
            List<CommunicationSession> sessions;


            Sessions.getMPZArray(new List<RequestParams> { coridor_rp }, dt1, dt2 
    , silenceRanges
    , inactivityRanges
    , routesToDrop
    , routesToDelete
    , managerDB
    , 356
    , out mpzArray
    , out sessions);


            Console.Write("GEOMETRYCOLLECTION(");
            Console.Write(corydor_pol);
            Console.Write(",");
            Console.Write(Polygon.getMultipolFromPolygons(mpzArray.SelectMany(mpz => mpz.Routes.SelectMany(r => r.Parameters.ShootingConf.orders.Select(ord => ord.captured))).ToList()));
            Console.Write(")");



        }



        public IList<CaptureConf> test_getCaptureConfArray()
        {
            //string str1 = "POLYGON((37.68310546875 55.95688849713528,37.21343994140624 55.808998992704886,37.4798583984375 55.51774716789876,38.111572265625 55.63109707296326,38.00445556640624 55.9722612642278,37.68310546875 55.95688849713528))";
            //string str2 = "POLYGON((37.68859863281249 55.890715987422226,37.57873535156249 55.85989956952267,37.71881103515624 55.78892895389265,37.91107177734374 55.84294011297763,37.72155761718749 55.90765459369871,37.68859863281249 55.890715987422226))";
            //SqlGeography geom1 = SqlGeography.STGeomFromText(new SqlChars(str1), 4326);
            //SqlGeography geom2 = SqlGeography.STGeomFromText(new SqlChars(str1), 4326);
            //SqlGeography geom3 = SqlGeography.ST

            DateTime start = DateTime.Now;
            
            test_TestSessionsSequenses();
            //test_getPlainMpzArray();
           
            DateTime endd = DateTime.Now;
            Console.WriteLine("total time = " + (endd - start).TotalSeconds.ToString());

            return null;
        }
         

        public void test_TestSessionsSequenses()
        {
           // tttt();
          ///  return;
            DateTime fromDt = DateTime.Parse("20.02.2019 0:0:0"); 
            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
 
            //new SatelliteTrajectory.LanePos(p, 2 * OptimalChain.Constants.max_roll_angle + OptimalChain.Constants.camera_angle, 0)
            //DataFetcher fetcher = new DataFetcher(managerDB);
            //Trajectory trajectory = fetcher.GetTrajectorySat(fromDt,  DateTime.Parse(" 01.03.2019 1:39:12")    );
            //var roll = 2 * OptimalChain.Constants.max_roll_angle + OptimalChain.Constants.camera_angle;
            //foreach (var point in trajectory)
            //{
            //    Console.WriteLine(point.Position.ToVector().Length - Astronomy.Constants.EarthRadius);
            //}
            //new SatLane(trajectory, roll, 0);
            //return;
            List<MPZ> mpzArray;
            SessionsPlanning.TestSessionsSequenses.get20AreaShooting5Turn(fromDt, managerDB, out mpzArray);
             
            var orderPolsList = mpzArray.SelectMany(mpz => mpz.Routes.SelectMany(r => r.Parameters.ShootingConf.orders.Select(order => order.captured))).ToList();
            var shootingPolsList = mpzArray.SelectMany(mpz => mpz.Routes.Select(r =>  new Polygon(r.Parameters.ShootingConf.wktPolygon))).ToList();

            Console.Write("GEOMETRYCOLLECTION(");            
            Console.Write(Polygon.getMultipolFromPolygons(orderPolsList));
            Console.Write(",");

            Console.Write(Polygon.getMultipolFromPolygons(shootingPolsList));
            Console.Write(")");
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
