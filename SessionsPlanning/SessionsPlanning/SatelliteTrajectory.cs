using System;
using System.Collections.Generic;
using SatelliteRequests;
using Astronomy;
using Common;
using System.Windows.Media.Media3D;
using System.Spatial;

namespace SatelliteTrajectory
{
    public class SatTrajectory
    {
        public List<TrajectoryPoint> points { get; set; }
        public double step { get; set; }
        public double duration { get; set; }
        public DateTime startDateTime { get; set; }

        /// <summary>
        /// Get strip shooting by trajectory and max roll angle  
        /// </summary>
        /// <param name="latitude">roll angle.</param>          
        public List<Tuple<GeoPoint, GeoPoint>> getTrajectoryStrip(double angle)
        {
            double halfWidth = 3;  ///  TODO @todo get halfWidth by angle
            
            List<Tuple<GeoPoint, GeoPoint>> strip = new List<Tuple<GeoPoint, GeoPoint>>();
            
            for (int i = 0; i < points.Count - 1; i++)
            {
                GeoPoint point = GeoPoint.FromCartesian(points[i].Position.X, points[i].Position.Y, points[i].Position.Z);
                GeoPoint nextPoint = GeoPoint.FromCartesian(points[i+1].Position.X, points[i+1].Position.Y, points[i+1].Position.Z);

                double x = nextPoint.Latitude - point.Latitude;
                double y = nextPoint.Longitude - point.Longitude;
                Vector3D dirVect = new Vector3D(x,y,0);
                Vector3D axis = new Vector3D(0, 0, 1);     
           
                Matrix3D leftRotMatr = Matrix3D.Identity;
                leftRotMatr.Rotate(new Quaternion(axis, -90));
                Vector3D leftVect = leftRotMatr.Transform(dirVect);
                leftVect.Normalize();
                leftVect = leftVect * halfWidth;

                Matrix3D rightRotMatr = Matrix3D.Identity;
                rightRotMatr.Rotate(new Quaternion(axis, 90));
                Vector3D rightVect = rightRotMatr.Transform(dirVect);
                rightVect.Normalize();
                rightVect = rightVect * halfWidth;

                GeoPoint leftPoint = new GeoPoint(point.Latitude + leftVect.X, point.Longitude + leftVect.Y, false);
                GeoPoint rightPoint = new GeoPoint(point.Latitude + rightVect.X, point.Longitude + rightVect.Y, false);

                strip.Add(new Tuple<GeoPoint, GeoPoint>(leftPoint, rightPoint));
            }                       
            return strip;
        }
    }


}

