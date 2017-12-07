using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using System.Spatial;

using SphericalGeom;
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
        /// <param name="minAngle">min roll angle in radians.</param>            
        /// /// <param name="maxAngle">max roll angle in radians.</param>          
        public List<Polygon> getCaptureLane(double minAngle, double maxAngle)
        {
            if (minAngle >= maxAngle)
                throw new System.ArgumentException("Wrong arguments. minAngle >= maxAngle", "original");

            List<Polygon> lane = new List<Polygon>();

            List<Vector3D> lanePoints = new List<Vector3D>();
            List<Vector3D> rightLanePoints = new List<Vector3D>();

            GeoPoint prevPoint = AstronomyMath.GreenwichToSpherical(points[0].Position);
            for (int p_ind = 0; p_ind < points.Count - 1; p_ind++)
            {
                GeoPoint point = AstronomyMath.GreenwichToSpherical(points[p_ind].Position);
                GeoPoint nextPoint = AstronomyMath.GreenwichToSpherical(points[p_ind + 1].Position);

                Vector3D vectToCent = new Vector3D(points[p_ind].Position.X, points[p_ind].Position.Y, points[p_ind].Position.Z);
                // double altitude = vectToCent.Length - Constants.EarthRadius;

                double minBeta = Math.PI - Math.Asin(vectToCent.Length * Math.Sin(minAngle) / Constants.EarthRadius);
                double maxBeta = Math.PI - Math.Asin(vectToCent.Length * Math.Sin(maxAngle) / Constants.EarthRadius);

                double leftBound = AstronomyMath.ToDegrees(Math.PI - minAngle - minBeta);
                double rightBound = AstronomyMath.ToDegrees(Math.PI - maxAngle - maxBeta); 

                double dirx = nextPoint.Latitude - point.Latitude;
                double diry = nextPoint.Longitude - point.Longitude;
                Vector3D dirVect = new Vector3D(dirx, diry, 0);
                
                RotateTransform3D rightTransform = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), 90));

                Vector3D leftVect = rightTransform.Transform(dirVect);
                leftVect.Normalize();
                leftVect = leftVect * leftBound;

                Vector3D rightVect = rightTransform.Transform(dirVect);
                rightVect.Normalize();
                rightVect = rightVect * rightBound;

                GeoPoint leftGeoPoint = new GeoPoint(point.Latitude + leftVect.X, point.Longitude + leftVect.Y, false);
                GeoPoint rightGeoPoint = new GeoPoint(point.Latitude + rightVect.X, point.Longitude + rightVect.Y, false);

                Vector3D leftPoint = GeoPoint.ToCartesian(leftGeoPoint, 1.0);
                Vector3D rightPoint = GeoPoint.ToCartesian(rightGeoPoint, 1.0);

                lanePoints.Add(leftPoint);
                rightLanePoints.Add(rightPoint);
                
                // проверим, не последняя ли это точка в секции
                double latDist, lonDist;
                double nextLatDist, nextLonDist;
                GeoPoint.CircleDistance(point, prevPoint, out latDist, out lonDist);
                GeoPoint.CircleDistance(nextPoint, prevPoint, out nextLatDist, out nextLonDist);

                if (p_ind != 0 && (p_ind == points.Count - 2 || nextLatDist < latDist || nextLonDist < lonDist))
                {
                    for (int i = rightLanePoints.Count - 1; i >= 0; i--)                    
                        lanePoints.Add(rightLanePoints[i]);                    

                    Polygon sector = new Polygon(lanePoints, new Vector3D(0, 0, 0));
                    lane.Add(sector);

                    rightLanePoints.Clear();
                    lanePoints.Clear();

                }
                prevPoint = point;
            }

            return lane;
        }
    }


}

