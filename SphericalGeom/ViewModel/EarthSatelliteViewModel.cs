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
            return curRequest.Contains(new Vector3D(x, y, z));
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

        public void q_CreateCaptureIntervals()
        {

            DateTime begDt = new DateTime(2000, 1, 1);
            DateTime endDt = new DateTime(2020, 1, 1);

            string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "\\" + "trajectory_5hours.dat";
            Astronomy.Trajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, begDt, endDt); // @todo временно

            double viewAngle = AstronomyMath.ToRad(45);
            double rollAngle = AstronomyMath.ToRad(0);

            // максимально детализированная полоса
            SatLane viewLaneFull = new SatLane(trajectory, rollAngle, viewAngle, 1);

            // полоса, в которой точек меньше в 16 раз
            SatLane viewLane = new SatLane(trajectory, rollAngle, viewAngle, 15);

            foreach (var sector in viewLaneFull.Sectors)
            {
                Console.WriteLine("");
            }
        }


        public void adadasdasdasdCreateCaptureIntervals()
        {



            DateTime begDt = new DateTime(2000, 1, 1);
            DateTime endDt = new DateTime(2020, 1, 1);

            string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "\\" + "trajectory_5hours.dat";
            Astronomy.Trajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, begDt, endDt); // @todo временно

            double viewAngle = AstronomyMath.ToRad(45);
            double rollAngle = AstronomyMath.ToRad(0);

            // максимально детализированная полоса
            SatLane viewLaneFull = new SatLane(trajectory, rollAngle, viewAngle, 1);

            // полоса, в которой точек меньше в 16 раз
            SatLane viewLane = new SatLane(trajectory, rollAngle, viewAngle, 15);


            // Console.WriteLine("viewLaneFull.size = {0}, viewLane.size = {1}", viewLaneFull.lanePoints.Count, viewLane.lanePoints.Count);

            int i = 0;
            foreach (var point in trajectory.Points)
            {
                DateTime calc_time = new DateTime();
                foreach (var sector in viewLaneFull.Sectors)
                {
                    if (sector.fromDT <= point.Time && point.Time <= sector.toDT)
                    {
                        calc_time = sector.getPointTime(point.Position.ToVector());
                    }
                }
                var diff_msecs = (point.Time - calc_time).TotalMilliseconds;
                if (Math.Abs(diff_msecs) > 9)
                {
                    foreach (var sector in viewLaneFull.Sectors)
                    {
                        if (sector.fromDT <= point.Time && point.Time <= sector.toDT)
                        {
                            calc_time = sector.getPointTime(point.Position.ToVector());
                        }
                    }
                }
                i++;
            }
        }
        /*
            DateTime begDt = new DateTime(2000, 1, 1);
            DateTime endDt = new DateTime(2020, 1, 1);

            string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "\\" + "trajectory_5hours.dat";
            SatTrajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, begDt, endDt); // @todo временно

            double viewAngle = AstronomyMath.ToRad(45);
            double rollAngle = AstronomyMath.ToRad(0);

            // максимально детализированная полоса
            SatLane viewLaneFull = trajectory.getCaptureLane(rollAngle, viewAngle, 1);
  
            LanePos lanePos1 = viewLaneFull.Sectors[0].sectorPoints[0];
            LanePos lanePos2 = viewLaneFull.Sectors[0].sectorPoints[1];
            LanePos lanePos3 = viewLaneFull.Sectors[0].sectorPoints[2];

            double dist13 = AstronomyMath.ToDegrees(GeoPoint.DistanceOverSurface(lanePos1.MiddleGeoPoint, lanePos3.MiddleGeoPoint));
            double dist12 = AstronomyMath.ToDegrees(GeoPoint.DistanceOverSurface(lanePos1.MiddleGeoPoint, lanePos2.MiddleGeoPoint));
            double dist23 = AstronomyMath.ToDegrees(GeoPoint.DistanceOverSurface(lanePos2.MiddleGeoPoint, lanePos3.MiddleGeoPoint));
            double test = dist13 - dist12 - dist23;

            double dist2 = AstronomyMath.ToDegrees(lanePos1.getDistToPoint(lanePos2.MiddleGeoPoint));
            

            return;
        }*/


        DateTime getInterpolTime(DateTime dt1, DateTime dt2, double dist, double fullDist)
        {
            double fullTime13 = Math.Abs((dt1 - dt2).TotalMilliseconds);
            double diffMiliSecs = fullTime13 * dist / fullDist;
            var newTime = dt1.AddMilliseconds(diffMiliSecs);
            return newTime;
        }

        public void test()
        {

            DateTime begDt = new DateTime(2000, 1, 1);
            DateTime endDt = new DateTime(2020, 1, 1);

            string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "\\" + "trajectory_5hours.dat";
            Trajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, begDt, endDt); // @todo временно

            double viewAngle = AstronomyMath.ToRad(5);
            double rollAngle = AstronomyMath.ToRad(0);

            // максимально детализированная полоса
            SatLane viewLaneFull = new SatLane(trajectory, rollAngle, viewAngle, 1);

            // полоса, в которой точек меньше в 16 раз
            SatLane viewLane = new SatLane(trajectory, rollAngle, viewAngle, 2);

            double maxError = 0;
            int countErrors = 0;
            int errorLimit = 3;

            int i = 0;
            foreach (var orig_sector in viewLaneFull.Sectors)
            {
                foreach (var orig_point in orig_sector.sectorPoints)
                {
                    var point = orig_point.MiddleCartPoint;

                    DateTime calc_time = new DateTime();
                    foreach (var sector in viewLane.Sectors)
                    {
                        if (sector.fromDT <= orig_point.Time && orig_point.Time <= sector.toDT)
                        {
                            calc_time = sector.getPointTime(point);
                            i++;
                        }
                    }

                    var diff_msecs = (orig_point.Time - calc_time).TotalMilliseconds;
                    if (maxError < diff_msecs)
                        maxError = diff_msecs;
                    if (Math.Abs(diff_msecs) >= errorLimit)
                    {
                        countErrors++;
                    }
                }
            }
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

        public void testt()
        {
            List<Vector3D> verts = new List<Vector3D>()
            {
            new Vector3D(-0.852034459185435, 0.120130037485442, -0.509515509532664 ),
            new Vector3D(-0.972504632989056, -0.156405344939802, -0.172545955875771 ),
            new Vector3D(-0.549501530291284, -0.609902051020679, 0.571023253789464 ),
            new Vector3D(-0.455742357633653, -0.633482054296441, 0.625299440542594 ),
            new Vector3D(-0.469679375963483, -0.56027574121427, 0.68227001810233 ),
            new Vector3D(-0.564279557617797, -0.536416675779779, 0.627571295392033 ),
            new Vector3D(-0.989735755317796, -0.0808817092973809, -0.11781885989368 ),
            new Vector3D(-0.869933396998213, 0.194610736332144, -0.453147377893121 ),
            };

            Polygon lanepol = new Polygon(verts);
            Polygon pp = new Polygon(lanepol.ToWtk());

        }

        public void getCameraView()
        {
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="I"></param>
        /// <param name="h"></param>
        /// <param name="v"></param>
        /// <param name="w"></param>
        /// <param name="b"></param>
        /// <param name="pitch"></param>
        /// <returns></returns>
        public double getRollCorrection(double h, double v, double w, double b, double pitch)
        {
            double I = OptimalChain.Constants.orbital_inclination;
            double R = Astronomy.Constants.EarthRadius;
            double bm = b + Math.Sin(I) * (Math.Acos(Math.Sqrt(1 - (R + h) * (R + h) / R / R * Math.Sin(pitch) * Math.Sin(pitch))) - pitch); 
            double d = Math.Cos(bm) * w / v * pitch * Math.Sin(I); 
            double sinRoll = R * Math.Sin(d) / Math.Sqrt(R * R + (R + h) * (R + h) - 2 * R * (R + h) * Math.Cos(d)); 
            return Math.Asin(sinRoll); 
        }

        public Polygon getPointPolygon(Vector3D point)
        {
            GeoPoint gp = GeoPoint.FromCartesian(point);
            Console.WriteLine(gp.ToString());
            double delta = 0.1;

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
            double rollDelta = -AstronomyMath.ToRad( 0.730159124782707 );//getRollCorrection(h, velo, w, b, pitchAngle);
            Console.WriteLine("{0} рад - рассчитаная поправка по крену. В градусах - {1}", rollDelta, AstronomyMath.ToDegrees(rollDelta));
            var resTestPoint = LanePos.getSurfacePoint(point, 0 + rollDelta, pitchAngle);

            // расстояние между точкой в надир и точкой с тангажом
            double dist = GeoPoint.DistanceOverSurface(nadirPoint, pointPitch);
            double deltaTime = Math.Sign(pitchAngle) * dist / velo; // время, которое потребуется для преодаления dist с текущей скоростью.
            Console.WriteLine("Упреждение по времени = {0} секунд ", deltaTime);
            Console.WriteLine("Упреждение по расстоянию = {0} км ", dist * Astronomy.Constants.EarthRadius);
            DateTime time_2 = point.Time.AddSeconds(deltaTime);
            TrajectoryPoint point_2  = trajectory.GetPoint(time_2); // получим точку траектории, отстающую от начальной на deltaTime
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
            //  SatLane strip = new SatLane(trajectory, rollAngle, viewAngle, readStep: 1, polygonStep: 15);

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

        public void testViewArea()//Trajectory trajectory)
        { 
         //   for (double AngleDegree = 0; AngleDegree <= 45; AngleDegree += 5)
            {
                double rollAngleDegree = 30;
                // double rollAngle = AstronomyMath.ToRad(45); // -0.78539816339744828;
                double rollAngle = AstronomyMath.ToRad(rollAngleDegree);
                double pitchAngle = AstronomyMath.ToRad(45);
                double viewAngle = AstronomyMath.ToRad(1);
                double minAngle = rollAngle - viewAngle / 2;
                double maxAngle = rollAngle + viewAngle / 2;

               // double h = 725;
               // GeoPoint point = new GeoPoint(21.1834626801572, -15.6786539720217);
                Vector3D position = new Vector3D( 6363.5108478, -1786.1444021, 2561.4350056);//GeoPoint.ToCartesian(point, Astronomy.Constants.EarthRadius + h);

                Vector3D eDirVect = new Vector3D(-position.X, -position.Y, -position.Z);
                Vector3D velo = new Vector3D(3.1737652626, 3.7964467971, -5.2171244942);
                
                //Vector3D rightRotAxis = -rotAxis;// Vector3D.CrossProduct(eDirVect, velo);

                List<Vector3D> verts = new List<Vector3D>();

                Vector3D rollAxis = Vector3D.CrossProduct(velo, eDirVect);
                RotateTransform3D rollTransfrom = new RotateTransform3D(new AxisAngleRotation3D(velo, AstronomyMath.ToDegrees(rollAngle)));
                
                Vector3D pitchAxis = Vector3D.CrossProduct(velo, eDirVect);
                RotateTransform3D pitchTransfrom = new RotateTransform3D(new AxisAngleRotation3D(pitchAxis, AstronomyMath.ToDegrees(pitchAngle)));
            
                Vector3D dirVector = rollTransfrom.Transform(pitchTransfrom.Transform(eDirVect));
                rollAxis = rollTransfrom.Transform(pitchTransfrom.Transform(rollAxis));     
                
                Vector3D rightLeftAxis = Vector3D.CrossProduct(dirVector, rollAxis);     
                RotateTransform3D rightHorizTransfrom = new RotateTransform3D(new AxisAngleRotation3D(rightLeftAxis, AstronomyMath.ToDegrees(+viewAngle / 2)));
                RotateTransform3D leftHorizTransfrom = new RotateTransform3D(new AxisAngleRotation3D(rightLeftAxis, AstronomyMath.ToDegrees(-viewAngle / 2)));
                
                Vector3D leftVector = leftHorizTransfrom.Transform(dirVector);     
                Vector3D rightVector = rightHorizTransfrom.Transform(dirVector);

                RotateTransform3D topVertTransfrom = new RotateTransform3D(new AxisAngleRotation3D(rollAxis, AstronomyMath.ToDegrees(+viewAngle / 2)));
                RotateTransform3D botVertTransfrom = new RotateTransform3D(new AxisAngleRotation3D(rollAxis, AstronomyMath.ToDegrees(-viewAngle / 2)));

                Vector3D topVector = topVertTransfrom.Transform(dirVector);  
                Vector3D botVector = botVertTransfrom.Transform(dirVector);

                test_vectors(leftVector, topVector, rightVector, botVector);

                TrajectoryPoint point = new TrajectoryPoint(new DateTime(), position.ToPoint(), velo);
                Vector3D rollDirVect = LanePos.getDirectionVector(point, AstronomyMath.ToRad(45), 0);
                Vector3D rollPitchDirVect = LanePos.getDirectionVector(point, AstronomyMath.ToRad(45), AstronomyMath.ToRad(45));

                Vector3D rollPoint = Routines.SphereVectIntersect(rollDirVect, position.ToPoint(), Astronomy.Constants.EarthRadius);
                Vector3D rollPithPoint = Routines.SphereVectIntersect(rollPitchDirVect, position.ToPoint(), Astronomy.Constants.EarthRadius);
                double dist = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(rollPoint), GeoPoint.FromCartesian(rollPithPoint)) * Astronomy.Constants.EarthRadius;
                double speed = velo.Length;
                double t =  dist / speed;          
                Console.WriteLine("t = {0}", t);
                Console.WriteLine("deltaRoll  = {0}", AstronomyMath.ToDegrees(t * OptimalChain.Constants.earthRotSpeed));  
                 
                //      double speed = OptimalChain.Constants.earthRotSpeed; // скорость вращения земли 7.2921158553e-5

                //trajectory.GetPosition()

                int i = 0, jj = 0;
                int pos_ind = 0;
                int sect_ind = 0;
                double min_dist = 10000000000;

                foreach (var sect in captureLanes[0].Sectors)
                {
                    pos_ind = 0;
                    jj = 0;
                    foreach (var pos in sect.sectorPoints)
                    {
                        var distt = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(rollPithPoint), pos.LeftGeoPoint);
                        if (min_dist > distt)
                        {
                            min_dist = distt;
                            sect_ind = i;
                            pos_ind = jj;
                        }
                        jj++;
                    }
                    i++;
                }

                var mp = captureLanes[0].Sectors[sect_ind].sectorPoints[pos_ind];

                Console.WriteLine("min_dist = {0} ",AstronomyMath.ToDegrees( min_dist) );



                //Console.WriteLine("leftVector.Length = {0}", leftVector.Length);
                //Console.WriteLine("rightVector.Length = {0}", rightVector.Length);
                //Console.WriteLine("topVector.Length = {0}", topVector.Length);
                //Console.WriteLine("botVector.Length = {0}", botVector.Length);

          //      verts.Add(SatTrajectory.SphereVectIntersect(leftVector, position.ToPoint(), Astronomy.Constants.EarthRadius));
         //       verts.Add(SatTrajectory.SphereVectIntersect(rightVector, position.ToPoint(), Astronomy.Constants.EarthRadius));
          //      verts.Add(SatTrajectory.SphereVectIntersect(topVector, position.ToPoint(), Astronomy.Constants.EarthRadius));
          //      verts.Add(SatTrajectory.SphereVectIntersect(botVector, position.ToPoint(), Astronomy.Constants.EarthRadius));

                var tanHalfVa = Math.Tan(viewAngle / 2);
                var angle_rad = Math.Atan(tanHalfVa / (Math.Sqrt(tanHalfVa * tanHalfVa + 1)));
                var angle_degr = AstronomyMath.ToDegrees(angle_rad);

           /*
                Console.WriteLine("angle = {0}", angle_degr);
                var rrAxis = Vector3D.CrossProduct(topVector, rollAxis);
                RotateTransform3D horizTransfrom_1 = new RotateTransform3D(new AxisAngleRotation3D(rrAxis, angle_degr));
                Vector3D crossVector_1 = horizTransfrom_1.Transform(topVector);
                Vector3D crossPoint_1 = SatTrajectory.SphereVectIntersect(crossVector_1, position.ToPoint(), Astronomy.Constants.EarthRadius);
              //  verts.Add(crossPoint_1);
                Console.WriteLine("crossPoint_1 = {0}; {1}; {2}", crossPoint_1.X, crossPoint_1.Y, crossPoint_1.Z);
             
                RotateTransform3D horizTransfrom_2 = new RotateTransform3D(new AxisAngleRotation3D(rrAxis, -angle_degr));
                Vector3D crossVector_2 = horizTransfrom_2.Transform(topVector);
                Vector3D crossPoint_2 = SatTrajectory.SphereVectIntersect(crossVector_2, position.ToPoint(), Astronomy.Constants.EarthRadius);
              //  verts.Add(crossPoint_2);
                Console.WriteLine("crossPoint_2 = {0}; {1}; {2}", crossPoint_2.X, crossPoint_2.Y, crossPoint_2.Z);

                rrAxis = Vector3D.CrossProduct(botVector, rollAxis);
                RotateTransform3D horizTransfrom_3 = new RotateTransform3D(new AxisAngleRotation3D(rrAxis, -angle_degr));
                Vector3D crossVector_3 = horizTransfrom_3.Transform(botVector);
                Vector3D crossPoint_3 = SatTrajectory.SphereVectIntersect(crossVector_3, position.ToPoint(), Astronomy.Constants.EarthRadius);
              //  verts.Add(crossPoint_3);
                Console.WriteLine("crossPoint_3 = {0}; {1}; {2}", crossPoint_3.X, crossPoint_3.Y, crossPoint_3.Z);
                           
                RotateTransform3D horizTransfrom_4 = new RotateTransform3D(new AxisAngleRotation3D(rrAxis, angle_degr));
                Vector3D crossVector_4 = horizTransfrom_4.Transform(botVector);
                Vector3D crossPoint_4 = SatTrajectory.SphereVectIntersect(crossVector_4, position.ToPoint(), Astronomy.Constants.EarthRadius);
              //  verts.Add(crossPoint_4);
              //  Console.WriteLine("crossPoint_4 = {0}; {1}; {2}", crossPoint_4.X, crossPoint_4.Y, crossPoint_4.Z);

              //  test_vectors(topVector, botVector, topVector, botVector);
             //   test_vectors(crossVector_1, crossVector_2, crossVector_3, crossVector_4);
           */
                
           /*
                Console.WriteLine("angle = {0}", angle_degr);
                rotAxis = Vector3D.CrossProduct(rightVector, rightLeftAxis);
                RotateTransform3D horizTransfrom_1 = new RotateTransform3D(new AxisAngleRotation3D(rotAxis, angle_degr));
                Vector3D crossVector_1 = horizTransfrom_1.Transform(rightVector);
                Vector3D crossPoint_1 = SatTrajectory.SphereVectIntersect(crossVector_1, position.ToPoint(), Astronomy.Constants.EarthRadius);
                verts.Add(crossPoint_1);
                Console.WriteLine("crossPoint_1 = {0}; {1}; {2}", crossPoint_1.X, crossPoint_1.Y, crossPoint_1.Z);

                rotAxis = Vector3D.CrossProduct(rightVector, rightLeftAxis);
                RotateTransform3D horizTransfrom_2 = new RotateTransform3D(new AxisAngleRotation3D(rotAxis, -angle_degr));
                Vector3D crossVector_2 = horizTransfrom_2.Transform(rightVector);
                Vector3D crossPoint_2 = SatTrajectory.SphereVectIntersect(crossVector_2, position.ToPoint(), Astronomy.Constants.EarthRadius);
                verts.Add(crossPoint_2);
                Console.WriteLine("crossPoint_2 = {0}; {1}; {2}", crossPoint_2.X, crossPoint_2.Y, crossPoint_2.Z);

                rotAxis = Vector3D.CrossProduct(leftVector, rightLeftAxis);
                RotateTransform3D horizTransfrom_3 = new RotateTransform3D(new AxisAngleRotation3D(rotAxis, -angle_degr));
                Vector3D crossVector_3 = horizTransfrom_3.Transform(leftVector);
                Vector3D crossPoint_3 = SatTrajectory.SphereVectIntersect(crossVector_3, position.ToPoint(), Astronomy.Constants.EarthRadius);
                verts.Add(crossPoint_3);
                Console.WriteLine("crossPoint_3 = {0}; {1}; {2}", crossPoint_3.X, crossPoint_3.Y, crossPoint_3.Z);

                rotAxis = Vector3D.CrossProduct(leftVector, rightLeftAxis);
                RotateTransform3D horizTransfrom_4 = new RotateTransform3D(new AxisAngleRotation3D(rotAxis, angle_degr));
                Vector3D crossVector_4 = horizTransfrom_4.Transform(leftVector);
                Vector3D crossPoint_4 = SatTrajectory.SphereVectIntersect(crossVector_4, position.ToPoint(), Astronomy.Constants.EarthRadius);
                verts.Add(crossPoint_4);
                Console.WriteLine("crossPoint_4 = {0}; {1}; {2}", crossPoint_4.X, crossPoint_4.Y, crossPoint_4.Z);
            
                test_vectors(crossVector_1, crossVector_2, crossVector_3, crossVector_4);
               */
               // test_vectors(leftVector, topVector, rightVector, botVector);
               // test_vectors(leftVector, rightVector, topVector, botVector);
            

                //verts.Add(crossPoint_1);

               // Vector3D rightCrossPoint = SatTrajectory.SphereVectIntersect(leftVector, position.ToPoint(), Astronomy.Constants.EarthRadius);
               //   Vector3D leftCrossPoint = SatTrajectory.SphereVectIntersect(rightVector, position.ToPoint(), Astronomy.Constants.EarthRadius);

                //Console.WriteLine("botVector = {0}; {1}; {2}", botVector.X, botVector.Y, botVector.Z);  
                //Console.WriteLine("topVector = {0}; {1}; {2}", topVector.X, topVector.Y, topVector.Z);

                /*
                Console.WriteLine("rightCrossPoint = {0}; {1}; {2}", rightCrossPoint.X, rightCrossPoint.Y, rightCrossPoint.Z);
                Console.WriteLine("leftCrossPoint = {0}; {1}; {2}", leftCrossPoint.X, leftCrossPoint.Y, leftCrossPoint.Z);
              
                Console.WriteLine();
                
                
                Vector3D crossPoint = SatTrajectory.SphereVectIntersect(topVector, position.ToPoint(), Astronomy.Constants.EarthRadius);
                verts.Add(crossPoint);
                crossPoint = SatTrajectory.SphereVectIntersect(leftVector, position.ToPoint(), Astronomy.Constants.EarthRadius);
                verts.Add(crossPoint);
                crossPoint = SatTrajectory.SphereVectIntersect(botVector, position.ToPoint(), Astronomy.Constants.EarthRadius);
                verts.Add(crossPoint);
                crossPoint = SatTrajectory.SphereVectIntersect(rightVector, position.ToPoint(), Astronomy.Constants.EarthRadius);
                verts.Add(crossPoint);
                
                */
         

                // verts.Add(GeoPoint.ToCartesian(point,1));
                // Console.WriteLine("angle = {0}", angle);
               
                    int n = 5;
                    double angle_h = angle_rad * 2 / n;

                    for (int j = 1; j <= n - 1; j++)
                    {
                        Vector3D rAxis = Vector3D.CrossProduct(topVector, rollAxis);
                        double angle = AstronomyMath.ToDegrees(-angle_rad + angle_h * j);
                        RotateTransform3D vertTransfrom = new RotateTransform3D(new AxisAngleRotation3D(rAxis, angle));
                        Vector3D crossVector = vertTransfrom.Transform(topVector);
                        Vector3D crossPoint = Routines.SphereVectIntersect(crossVector, position.ToPoint(), Astronomy.Constants.EarthRadius);
                        verts.Add(crossPoint);
                    }

                 //   Console.WriteLine("angle_degr = {0}\n", angle_degr);

                    for (int j = 0; j <= n; j++)
                    {
                        Vector3D rAxis = Vector3D.CrossProduct(rightVector, velo);
                        double angle = AstronomyMath.ToDegrees(-angle_rad + angle_h * j);
                      //  Console.WriteLine("angle = {0}", angle);
                        RotateTransform3D horizTransfrom = new RotateTransform3D(new AxisAngleRotation3D(rAxis, angle));
                        Vector3D crossVector = horizTransfrom.Transform(rightVector);
                        Vector3D crossPoint = Routines.SphereVectIntersect(crossVector, position.ToPoint(), Astronomy.Constants.EarthRadius);
                        verts.Add(crossPoint);
                    }


                    for (int j = 1; j <= n - 1; j++)
                    {
                        Vector3D rAxis = Vector3D.CrossProduct(botVector, rollAxis);
                        double angle = AstronomyMath.ToDegrees(-angle_rad + angle_h * j);
                        RotateTransform3D vertTransfrom = new RotateTransform3D(new AxisAngleRotation3D(rAxis, angle));
                        Vector3D crossVector = vertTransfrom.Transform(botVector);
                        Vector3D crossPoint = Routines.SphereVectIntersect(crossVector, position.ToPoint(), Astronomy.Constants.EarthRadius);
                        verts.Add(crossPoint);
                    }

                    for (int j = 0; j <= n; j++)
                    {
                        Vector3D rAxis = Vector3D.CrossProduct(leftVector, velo);
                        double angle = AstronomyMath.ToDegrees(-angle_rad + angle_h * j);
                        RotateTransform3D horizTransfrom = new RotateTransform3D(new AxisAngleRotation3D(-rAxis, angle));
                        Vector3D crossVector = horizTransfrom.Transform(leftVector);
                        Vector3D crossPoint = Routines.SphereVectIntersect(crossVector, position.ToPoint(), Astronomy.Constants.EarthRadius);
                        verts.Add(crossPoint);
                    }
               
                    

                //verts.Add(GeoPoint.ToCartesian(point,1));

                //vertTransfrom = new RotateTransform3D(new AxisAngleRotation3D(velo, AstronomyMath.ToDegrees(minAngle)));

                polygons.Add(new Polygon(verts));
          }
           /*
            Console.Write("x=[");
            foreach (var pp in verts)
            {
                Console.Write("{0};  ", pp.X);
            }
            Console.WriteLine("]"); 

            Console.Write("y=[");
            foreach (var pp in verts)
            {
                Console.Write("{0};  ", pp.Y);
            }
            Console.WriteLine("]"); 
            
            Console.Write("z=[");
            foreach (var pp in verts)
            {
                Console.Write("{0};  ", pp.Z);
            }
            Console.WriteLine("]"); 
           */
            //}
            //curRequest = new Polygon(verts);
        }


        public void CreateCaptureIntervals()
        {
            test_roll_correction();
            return;

            polygons.Add(new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))"));
            polygons.Add(new Polygon("POLYGON ((0 0, 0 4, -4 4, -4 0, 0 0))"));

            polygons.Add(new Polygon("POLYGON ((-14 22, -18 26, -22 22, -18 18, -14 22))"));

            polygons.Add(new Polygon("POLYGON ((-24 16, -12 28, -16 32, -28 20, -24 16))"));

            polygons.Add(new Polygon("POLYGON ((-29 27, -27 25, -21 31, -23 33 , -29 27))"));
           
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


            SatLane strip15 = new SatLane(trajectory, rollAngle, viewAngle, polygonStep: 15);

            captureLanes.Add(strip15);

            return;

            int i = 0, j = 0;
            int pos_ind = 0;
            int sect_ind = 0;
            double min_dist = 10000000000;

            foreach (var sect in strip15.Sectors)
            {
                pos_ind = 0;
                j = 0;
                foreach (var pos in sect.sectorPoints)
                {
                    var dist = GeoPoint.DistanceOverSurface(pos.LeftGeoPoint, new GeoPoint(0, 0));
                    if (min_dist > dist)
                    {
                        min_dist = dist;
                        sect_ind = i;
                        pos_ind = j;
                    }
                    j++;
                }
                i++;
            }

            var mp = strip15.Sectors[sect_ind].sectorPoints[pos_ind];

            Console.WriteLine("mp.Velo = {0}, \nmp.position = {1} ", mp.TrajPoint.Velocity, mp.TrajPoint.Position);


            testViewArea();
            return;

            if (0 == Requests.Count)
                return;

            RequestParams reqparams = new RequestParams();
            reqparams.id = 1;
            reqparams.dateFrom = new DateTime(2000, 1, 1);
            reqparams.dateTo = new DateTime(2020, 1, 1);
            reqparams.wktPolygon = curRequest.ToWtk();

            List<CaptureConf> comconfs = new List<CaptureConf>();

            List<TargetWorkInterval> intervals = new List<TargetWorkInterval>();
            foreach (var lane in captureLanes)
            {
                //foreach (var sect in lane.Sectors)
                //{
                //    IList<Polygon> intersections = Polygon.Intersect(sect.polygon, curRequest);

                //    foreach (var inters in intersections)
                //    {
                //        captureIntervals.Add(inters); 
                //    }
                //}

                //continue;

                List<CaptureConf> confs = lane.getCaptureConfs(reqparams);
                // List<TargetWorkInterval> wints = lane.getTargetIntervals(curRequest, 0);
                foreach (var conf in confs)
                {
                    captureIntervals.Add(lane.getSegment(conf.dateFrom, conf.dateTo).Item1);
                }
                comconfs.AddRange(confs);
            }


        }

        public void test_isRequestFeasible()
        {
            if (curRequest == null)
                return;
            RequestParams reqparams = new RequestParams();
            reqparams.id = 1;
            reqparams.dateFrom = new DateTime(2000, 1, 1);
            reqparams.dateTo = new DateTime(2020, 1, 1);
            reqparams.wktPolygon = curRequest.ToWtk();
            reqparams.minCoverPerc = 0.4;
            Console.WriteLine(Sessions.isRequestFeasible(reqparams, new DateTime(2000, 1, 1), new DateTime(2020, 1, 1)));
        }


        public IList<CaptureConf> test_getCaptureConfArray()
        {
            DateTime start = DateTime.Now;
            //  if (curRequest == null)
            //      return null;

            List<RequestParams> requests = new List<RequestParams>();

            polygons.Add(new Polygon("POLYGON ((-2 -6, -2 -2, -6 -2, -6 -6, -2 -6))"));
            
            polygons.Add(new Polygon("POLYGON ((12 8, 12 12, 8 12, 8 8, 12 8))"));
            
            polygons.Add(new Polygon("POLYGON ((6 2, 6 6, 2 6, 2 2, 6 2))"));


            polygons.Add(new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))"));
            polygons.Add(new Polygon("POLYGON ((0 0, 0 4, -4 4, -4 0, 0 0))"));
            //polygons.Add(new Polygon("POLYGON ((-14 22, -18 26, -22 22, -18 18, -14 22))"));
            //polygons.Add(new Polygon("POLYGON ((-24 16, -12 28, -16 32, -28 20, -24 16))"));
            polygons.Add(new Polygon("POLYGON ((-29 27, -27 25, -21 31, -23 33 , -29 27))"));
            // polygons.Add(new Polygon("POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))"));

            //foreach (var req in Requests)
            //{
            //    var pol = new SphericalGeom.Polygon(req.Polygon.Select(sp => GeoPoint.ToCartesian(new GeoPoint(sp.Lat, sp.Lon), 1)), new Vector3D(0, 0, 0));
            int id = 0;
            foreach (var pol in polygons)
            {
                RequestParams reqparams = new RequestParams();
                reqparams.id = id;
                reqparams.dateFrom = new DateTime(2000, 1, 1);
                reqparams.dateTo = new DateTime(2020, 1, 1);
                reqparams.priority = 1;
                reqparams.minCoverPerc = 0.4;
                reqparams.Max_SOEN_anlge = AstronomyMath.ToRad(45);
                reqparams.wktPolygon = pol.ToWtk();
                requests.Add(reqparams);
                id++;
            }

            var res = Sessions.getCaptureConfs(requests, new DateTime(2000, 03, 13, 4, 0, 0), new DateTime(2115, 03, 13, 4, 4, 0));

            DateTime end = DateTime.Now;

            Console.WriteLine("total time = " + (end - start).TotalSeconds.ToString());


            foreach (var conf in res)
            {
                captureIntervals.Add(new Polygon(conf.wktPolygon));
            }
            Console.WriteLine("res.Count = {0}", res.Count());
            return res;
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
