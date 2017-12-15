using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;

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
        public SatLane getCaptureLane(double rollAngle, double viewAngle)
        {
            double minAngle = rollAngle - viewAngle / 2;
            double maxAngle = rollAngle + viewAngle / 2;

            if (minAngle >= maxAngle)
                throw new System.ArgumentException("Wrong arguments. minAngle >= maxAngle", "original");

            List<LanePos> lanePoints = new List<LanePos>();
             
            for (int p_ind = 0; p_ind < points.Count - 1; p_ind++)
            {                                
                Vector3D eDirVect = new Vector3D(-points[p_ind].Position.X, -points[p_ind].Position.Y, -points[p_ind].Position.Z);

                RotateTransform3D leftTransform = new RotateTransform3D(new AxisAngleRotation3D(points[p_ind].Velocity, AstronomyMath.ToDegrees(minAngle)));
                RotateTransform3D rightTransform = new RotateTransform3D(new AxisAngleRotation3D(points[p_ind].Velocity, AstronomyMath.ToDegrees(maxAngle)));

                Vector3D leftVector = leftTransform.Transform(eDirVect);
                Vector3D rightVector = rightTransform.Transform(eDirVect);

                Vector3D leftCrossPoint = SphereVectIntersect(leftVector, points[p_ind].Position, Constants.EarthRadius).ToVector();
                Vector3D rightCrossPoint = SphereVectIntersect(rightVector, points[p_ind].Position, Constants.EarthRadius).ToVector();

                leftCrossPoint.Normalize();
                rightCrossPoint.Normalize();

                LanePos newLanePos = new LanePos(leftCrossPoint, rightCrossPoint, points[p_ind].Time);
                lanePoints.Add(newLanePos);                
            }

            return new SatLane(minAngle, maxAngle,lanePoints);
        }
         

        /// <summary>
        /// just get crossings point of line (by point and vector) and sphere (radius R with the center (0,0,0) )
        /// </summary>
        /// <param name="vect"> directing vector</param>
        /// <param name="point"> initial point </param>
        /// <param name="R">sphere radius</param>
        /// <returns></returns>
        private Point3D SphereVectIntersect(Vector3D vect, Point3D point, double R)
        {
            double vx = vect.X;
            double vy = vect.Y;
            double vz = vect.Z;
            
            double px = point.X;
            double py = point.Y;
            double pz = point.Z;
 
            double vx2 = Math.Pow(vx, 2);
            double vy2 = Math.Pow(vy, 2);
            double vz2 = Math.Pow(vz, 2);
            
            double px2 = Math.Pow(px, 2);
            double py2 = Math.Pow(py, 2);
            double pz2 = Math.Pow(pz, 2);

            double R2 = Math.Pow(R, 2);

            double a = 1 + (vy2) / vx2 + (vz2) / vx2;
            double b = (2 * py * vy) / vx + (2 * pz * vz) / vx 
                     - (2 * px * vy2) / vx2 - (2 * px * vz2) / vx2;
            double c = py2 + pz2 + (px2 * vy2) / vx2 + (px2 * vz2) / vx2
                     - (2 * px * py * vy) / vx - (2 * px * pz * vz) / vx - R2;
 
            double D = Math.Pow(b, 2) - 4 * a * c;
            double x1 = (-b + Math.Sqrt(D)) / (2 * a);
            double x2 = (-b - Math.Sqrt(D)) / (2 * a);
            
            double y1 = (x1 - px) / vx * vy + py;
            double z1 = (x1 - px) / vx * vz + pz;

            double y2 = (x2 - px) / vx * vy + py;
            double z2 = (x2 - px) / vx * vz + pz;

            Vector3D vect1 = new Vector3D(x1 - px, y1 - py, z1 - pz);
            Vector3D vect2 = new Vector3D(x2 - px, y2 - py, z2 - pz);

            Point3D res = (vect1.Length < vect2.Length) ? new Point3D(x1, y1, z1) : new Point3D(x2, y2, z2);

            return res;
        }               
    }

    /// <summary>
    /// Полоса захвата
    /// </summary>
    public class SatLane
    {
        /// <summary>
        /// Угол крена <min,max>
        /// </summary>
        public double minRollAngle;
        public double maxRollAngle;

        public List<LanePos> lanePoints;
                
        public List<Polygon> polygons;

        public SatLane(double minAngle, double maxAngle, List<LanePos> points)
        {
            polygons = new List<Polygon>();
            lanePoints = points;
            List<Vector3D> leftLanePoints = new List<Vector3D>();
            List<Vector3D> rightLanePoints = new List<Vector3D>();
            GeoPoint prevPoint = AstronomyMath.GreenwichToSpherical(points[0].leftPoint.ToPoint());
            for (int p_ind = 0; p_ind < points.Count - 1; p_ind++)
            {
                leftLanePoints.Add(points[p_ind].leftPoint);
                rightLanePoints.Add(points[p_ind].rightPoint); 

                GeoPoint point = AstronomyMath.GreenwichToSpherical(points[p_ind].leftPoint.ToPoint());
                GeoPoint nextPoint = AstronomyMath.GreenwichToSpherical(points[p_ind + 1].leftPoint.ToPoint());

                double latDist, lonDist;
                double nextLatDist, nextLonDist;
                GeoPoint.CircleDistance(point, prevPoint, out latDist, out lonDist);
                GeoPoint.CircleDistance(nextPoint, prevPoint, out nextLatDist, out nextLonDist);

                if (p_ind != 0 && (p_ind == points.Count - 2 || nextLatDist < latDist || nextLonDist < lonDist))
                {
                    for (int i = rightLanePoints.Count - 1; i >= 0; i--)
                        leftLanePoints.Add(rightLanePoints[i]);


                    Polygon sector = new Polygon(leftLanePoints, new Vector3D(0, 0, 0));
                    polygons.Add(sector);
                    //break;

                    rightLanePoints.Clear();
                    leftLanePoints.Clear();
                }
                prevPoint = point;
            }
        }

        public List<TargetWorkInterval> getTargetIntervals(Polygon region, double minTimeDist)
        {
            List<TargetWorkInterval> resIntervals = new List<TargetWorkInterval>();

            foreach (Polygon sector in polygons)
            {
                Tuple<IList<Polygon>, IList<Polygon>> reTuple = Polygon.IntersectAndSubtract(sector, region);
                               
                foreach (var int_pol in reTuple.Item1)
                {
                    var verts = int_pol.Vertices;
                    var en = verts.GetEnumerator();
                    en.MoveNext();
                    DateTime tmin = getPointTime(en.Current);
                    DateTime tmax = tmin;
                    Vector3D minPoint, maxPoint;
                    foreach (var point in verts)
                    {
                        DateTime curTime = getPointTime(point);
                        if (curTime < tmin)
                        {
                            minPoint = point;
                            tmin = curTime;
                        } 
                        else if (curTime > tmax)
                        {
                            maxPoint = point;
                            tmax = curTime;
                        }
                    }
                    TargetWorkInterval newInt = new TargetWorkInterval();
                    newInt.fromDt = tmin;
                    newInt.toDt = tmax;
                    resIntervals.Add(newInt);
                }
            }

            for (int i = 0; i < resIntervals.Count - 1; i++)
            {
                if ((resIntervals[i + 1].fromDt - resIntervals[i].toDt).TotalSeconds < minTimeDist)
                {
                    resIntervals[i].toDt = resIntervals[i + 1].toDt;
                    resIntervals.RemoveAt(i + 1);
                }
            }

            return resIntervals;
        }

        public DateTime getPointTime(Vector3D point)
        {
            /* /// @todo такая проверка нужна, но отрабатывает не всегда корректно, проблема скорее всего в функции Contains
            bool is_contain = false;
            foreach(var sector in polygons)
            {
                if (sector.Contains(point))
                {
                    is_contain = true;
                    break;
                }
            }

              if (!is_contain)
                throw new System.ArgumentException("Point doesn't belong to this lane.");
            */

            double minDist = lanePoints[0].directDistToPoint(point);
            int minInd = 0;

            for (int i = 1; i < lanePoints.Count - 1; i++)
            {
                double dist = lanePoints[i].directDistToPoint(point);
                if (dist < minDist)
                {
                    minDist = dist;
                    minInd = i;
                }
            }

            LanePos curPos = lanePoints[minInd];
            LanePos nextPos = (minInd == lanePoints.Count - 1 ) ? lanePoints[minInd - 1] : lanePoints[minInd + 1];
            
            double distCur = curPos.directDistToPoint(point);
            double distNext = nextPos.directDistToPoint(point);

            double difTime = (curPos.time - nextPos.time).TotalSeconds;
            double diff = difTime * distCur / (distCur + distNext);
            DateTime res = (minInd == lanePoints.Count - 1) ? nextPos.time.AddSeconds(diff) : curPos.time.AddSeconds(diff);             

            return res;
        }
        
        /// <summary>
        /// only for tests
        /// </summary>
        public Polygon getSegment(DateTime begTime, DateTime endTime)
        {
            if (lanePoints[0].time > begTime || lanePoints[lanePoints.Count - 1].time < endTime)
                throw new System.ArgumentException("Incorrect time interval.");

            List<Vector3D> polygonPoints = new List<Vector3D>();
            List<Vector3D> rightPolygonPoints = new List<Vector3D>();

            for (int i = 0; i < lanePoints.Count; i++)
            {                
                if (lanePoints[i+1].time > begTime)
                {
                    if (0 == polygonPoints.Count)
                    {
                        // начался сегмент                    
                        // получаем точку входа
                        // получаем первую точку по времени    
                        // @todo 
                        ///if (lanePoints[i].time == begTime)
                    }

                    // мы внутри сегмента
                    polygonPoints.Add(lanePoints[i].leftPoint);
                    rightPolygonPoints.Add(lanePoints[i].rightPoint);
                }

                if (endTime < lanePoints[i].time)
                {
                    // закончился сегмент
                    // получаем последнюю точку по времени
                    break;
                }
                // @todo полигон может быть длиннее 180, что вызовет ошибки. Надо разрезать на полигоны или вставить проверку. 
            }

            for (int i = rightPolygonPoints.Count - 1; i >= 0; i--)
                polygonPoints.Add(rightPolygonPoints[i]);

            return new Polygon(polygonPoints, new Vector3D(0, 0, 0));
        }
    }
         
    public class LanePos
    {
        public Vector3D leftPoint;
        public Vector3D rightPoint;
        public DateTime time;

        public LanePos(Vector3D _leftPoint, Vector3D _rightPoint, DateTime _time)
        {
            leftPoint = _leftPoint;
            rightPoint = _rightPoint;
            time = _time;
        }
        /// <summary>
        /// distance in a straight line (not by sphere surface)
        /// </summary>
        /// <param name="point">checking point</param>
        /// <returns>distance</returns>
        public double directDistToPoint(Vector3D point)
        {
            ///@todo переделать на DistanceOverSurface
            double a = (rightPoint - leftPoint).Length;
            double b = (rightPoint - point).Length;
            double c = (leftPoint - point).Length;
            double halfPer = 0.5 * (a + b + c);
            double S = Math.Sqrt(halfPer * (halfPer - a) * (halfPer - b) * (halfPer - c));
            return 2 * S / halfPer;
        }
    }

    public class TargetWorkInterval
    {
        public DateTime fromDt;
        public DateTime toDt;
        public double minAngle;
        public double maxAngle; 
    }


}

