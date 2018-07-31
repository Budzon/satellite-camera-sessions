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


namespace ConsoleExecutor
{
    class WktTestingTools
    {

        public static string getLineSringStr(List<GeoPoint> line)
        {
            Char separator = System.Globalization.CultureInfo.CurrentCulture.NumberFormat.CurrencyDecimalSeparator[0];
            string res = "LINESTRING (";
            for (int i = 0; i < line.Count; i++)
            {
                var p = line[i];
                res += string.Format("{0}  {1}", p.Longitude.ToString().Replace(separator, '.'), p.Latitude.ToString().Replace(separator, '.'));
                if (i < line.Count - 1)
                    res += string.Format(" , ");
            }
            res += string.Format(")");
            return res;
        }

        public static string getPointsStr(List<GeoPoint> points)
        {
            Char separator = System.Globalization.CultureInfo.CurrentCulture.NumberFormat.CurrencyDecimalSeparator[0];
            string res = "GEOMETRYCOLLECTION(";
            for (int i = 0; i < points.Count; i++)
            {
                var p = points[i];
                res += string.Format("POINT ({0}  {1})", p.Longitude.ToString().Replace(separator, '.'), p.Latitude.ToString().Replace(separator, '.'));
                if (i < points.Count - 1)
                    res += string.Format(" , ");
            }
            res += string.Format(")");
            return res;
        }

        public static string getWKTStrip(DateTime dt1, DateTime dt2)
        {            
            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
            DataFetcher fetcher = new DataFetcher(managerDB);
            Trajectory traj = fetcher.GetTrajectorySat(dt1, dt2);
            SatLane strip = new SatLane(traj, 0, AstronomyMath.ToRad(90));
            return Polygon.getMultipolFromPolygons(strip.Sectors.Select(sect => sect.polygon).ToList()) ;
        }

        public static string getWKTInterpolateTrajectory(Trajectory trajectory, DateTime segmdt1, DateTime segmdt2, int step)
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

        public static string getWKTTrajectory(Trajectory trajectory)
        {
            string res = "";
            Char separator = System.Globalization.CultureInfo.CurrentCulture.NumberFormat.CurrencyDecimalSeparator[0];
            res += "LINESTRING(\n";
            for (int i = 0; i < trajectory.Count; i++)
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

        public static string getWKTLinePoints(List<Vector3D> points)
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

        public static string getWKTViewInterpolPolygons(Trajectory trajectory, DateTime segmdt1, DateTime segmdt2, double rollAngle, double pitchAngle, int step)
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

        public static string getWKTViewPolygons(Trajectory trajectory, DateTime segmdt1, DateTime segmdt2, double rollAngle, double pitchAngle)
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

        public static string getWKTViewLine(Trajectory trajectory, DateTime segmdt1, DateTime segmdt2, double rollAngle, double pitchAngle, int step)
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

        public static string getWKTStrip(Trajectory trajectory, double rollAngle)
        {
            SatLane lane = new SatLane(trajectory, rollAngle, OptimalChain.Constants.camera_angle);
            string res = Polygon.getMultipolFromPolygons(lane.Sectors.Select(sect => sect.polygon).ToList());
            return res;
        }

        public static string getWKTStripSegment(Trajectory trajectory, DateTime segmdt1, DateTime segmdt2, double rollAngle)
        {
            SatLane lane = new SatLane(trajectory, rollAngle, OptimalChain.Constants.camera_angle);
            return lane.getSegment(segmdt1, segmdt2).ToWtk();
        }


        public static string getWKTViewPolygon(Trajectory trajectory, double rollAngle, double pitchAngle, DateTime dtime)
        {
            TrajectoryPoint p1 = trajectory.GetPoint(dtime);
            SatelliteTrajectory.SatelliteCoordinates kp = new SatelliteTrajectory.SatelliteCoordinates(p1, rollAngle, pitchAngle);
            return kp.ViewPolygon.ToWtk();
        }




    }
}
