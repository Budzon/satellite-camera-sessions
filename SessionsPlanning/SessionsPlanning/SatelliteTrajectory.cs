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
        /// <param name="angle">roll angle in radians.</param>          
        public List<Polygon> getTrajectoryLane(double angle)
        {
            List<Polygon> lane = new List<Polygon>();

            List<vector3> lanePoints = new List<vector3>();
            List<vector3> rightLanePoints = new List<vector3>();

            GeoPoint prevPoint = AstronomyMath.GreenwichToSpherical(points[0].Position);
            for (int p_ind = 0; p_ind < points.Count - 1; p_ind++)
            {
                GeoPoint point = AstronomyMath.GreenwichToSpherical(points[p_ind].Position);
                GeoPoint nextPoint = AstronomyMath.GreenwichToSpherical(points[p_ind + 1].Position);

                Vector3D vectToCent = new Vector3D(points[p_ind].Position.X, points[p_ind].Position.Y, points[p_ind].Position.Z);
                // double altitude = vectToCent.Length - Constants.EarthRadius;

                double beta = Math.PI - Math.Asin(vectToCent.Length * Math.Sin(angle) / Constants.EarthRadius);
                double halfWidth = AstronomyMath.ToDegrees(Math.PI - angle - beta); 

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

                GeoPoint leftGeoPoint = new GeoPoint(point.Latitude + leftVect.X, point.Longitude + leftVect.Y, false);
                GeoPoint rightGeoPoint = new GeoPoint(point.Latitude + rightVect.X, point.Longitude + rightVect.Y, false);

                vector3 leftPoint = new vector3(new direction3(AstronomyMath.ToRad(leftGeoPoint.Latitude), AstronomyMath.ToRad(leftGeoPoint.Longitude)), 1);
                vector3 rightPoint = new vector3(new direction3(AstronomyMath.ToRad(rightGeoPoint.Latitude), AstronomyMath.ToRad(rightGeoPoint.Longitude)), 1);

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

                    Polygon sector = new Polygon(lanePoints, new vector3(0, 0, 0));
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

