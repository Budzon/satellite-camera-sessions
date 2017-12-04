using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using System.Spatial;


using SatelliteRequests;
using Common;
using Astronomy;

namespace SatelliteTrajectory
{
    public class SatTrajectory
    {
        public List<TrajectoryPoint> points { get; set; }
        public double step { get; set; }
        public double duration { get; set; }
        public DateTime startDateTime { get; set; }

        /// <summary>
        /// Get shooting lane by trajectory and max roll angle  
        /// </summary>
        /// <param name="latitude">roll angle in radians.</param>          
        public List<Tuple<GeoPoint, GeoPoint>> getTrajectoryLane(double angle)
        {
            List<Tuple<GeoPoint, GeoPoint>> lane = new List<Tuple<GeoPoint, GeoPoint>>();
            
            for (int i = 0; i < points.Count - 1; i++)
            {
                Vector3D vectToCent = new Vector3D(points[i].Position.X, points[i].Position.Y, points[i].Position.Z);
                double altitude = vectToCent.Length - Constants.EarthRadius;

                double halfWidth = altitude * Math.Tan(angle);  /// halfWidth of the lane 

                GeoPoint point = AstronomyMath.GreenwichToSpherical(points[i].Position); 
                GeoPoint nextPoint = AstronomyMath.GreenwichToSpherical(points[i].Position); 

                double dirx = nextPoint.Latitude - point.Latitude;
                double diry = nextPoint.Longitude - point.Longitude;
                Vector3D dirVect = new Vector3D(dirx, diry, 0);

                RotateTransform3D leftTransform = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), -90));
                RotateTransform3D rightTransform = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), 90));
                
                Vector3D leftVect = leftTransform.Transform(dirVect);
                leftVect.Normalize();
                leftVect = leftVect * halfWidth;

                Vector3D rightVect = rightTransform.Transform(dirVect);
                rightVect.Normalize();
                rightVect = rightVect * halfWidth;

                GeoPoint leftPoint = new GeoPoint(point.Latitude + leftVect.X, point.Longitude + leftVect.Y, false);
                GeoPoint rightPoint = new GeoPoint(point.Latitude + rightVect.X, point.Longitude + rightVect.Y, false);

                lane.Add(new Tuple<GeoPoint, GeoPoint>(leftPoint, rightPoint));
            }
            return lane;
        }
    }


}

