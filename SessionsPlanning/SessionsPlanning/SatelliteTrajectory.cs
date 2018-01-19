using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;

using SphericalGeom;
using SatelliteRequests;
using SatelliteSessions;
using Common;
using Astronomy;
using OptimalChain;
 
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
                throw new System.ArgumentException("Wrong arguments. minAngle >= maxAngle");

            List<LanePos> lanePoints = new List<LanePos>();
            
            for (int p_ind = 0; p_ind < points.Count; p_ind += 16)
            {
                Vector3D eDirVect = new Vector3D(-points[p_ind].Position.X, -points[p_ind].Position.Y, -points[p_ind].Position.Z);

                RotateTransform3D leftTransform = new RotateTransform3D(new AxisAngleRotation3D(points[p_ind].Velocity, AstronomyMath.ToDegrees(minAngle)));
                RotateTransform3D rightTransform = new RotateTransform3D(new AxisAngleRotation3D(points[p_ind].Velocity, AstronomyMath.ToDegrees(maxAngle)));

                Vector3D leftVector = leftTransform.Transform(eDirVect);
                Vector3D rightVector = rightTransform.Transform(eDirVect);
                
                Vector3D leftCrossPoint = SphereVectIntersect(leftVector, points[p_ind].Position, Astronomy.Constants.EarthRadius);
                Vector3D rightCrossPoint = SphereVectIntersect(rightVector, points[p_ind].Position, Astronomy.Constants.EarthRadius);
                Vector3D middlePoint = points[p_ind].Position.ToVector();

                leftCrossPoint.Normalize();
                rightCrossPoint.Normalize();
                middlePoint.Normalize();

                LanePos newLanePos = new LanePos(leftCrossPoint, middlePoint, rightCrossPoint, points[p_ind].Time);
                lanePoints.Add(newLanePos);
            }

            return new SatLane(minAngle, maxAngle, lanePoints);
        }
         

        /// <summary>
        /// just get crossings point of line (by point and vector) and sphere (radius R with the center (0,0,0) )
        /// </summary>
        /// <param name="vect"> directing vector</param>
        /// <param name="point"> initial point </param>
        /// <param name="R"> sphere radius</param>
        /// <returns></returns>
        private Vector3D SphereVectIntersect(Vector3D vect, Point3D point, double R)
        {
            Vector3D dilatedPoint = new Vector3D(point.X/R, point.Y/R, point.Z/R);
            List<Vector3D> intersection = Routines.IntersectLineUnitSphere(dilatedPoint, vect);

            Vector3D closest;
            if (intersection.Count == 2)
                closest = ((intersection[0] - dilatedPoint).Length < (intersection[1] - dilatedPoint).Length)
                    ? intersection[0] : intersection[1];
            else if (intersection.Count == 1)
                closest = intersection[0];
            else
                throw new ArgumentException("Line and sphere do not intersect.");

            return closest * R;
        }

        public static Vector3D getInterpolPoint(Vector3D firstPoint, Vector3D secondPoint, DateTime dt1, DateTime dt2, DateTime targetDt)
        {
            double fullDist = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(firstPoint), GeoPoint.FromCartesian(secondPoint));
            double fullTimeDiff = (dt2 - dt1).TotalSeconds;
            double timePart = (targetDt - dt1).TotalSeconds;
            double patrDist = fullDist * timePart / fullTimeDiff;

            Vector3D rotAxis = Vector3D.CrossProduct(firstPoint, secondPoint);
            RotateTransform3D rotTransform = new RotateTransform3D(new AxisAngleRotation3D(rotAxis, AstronomyMath.ToDegrees(patrDist)));
            Vector3D newMiddlePoint = rotTransform.Transform(firstPoint);

            return newMiddlePoint;
        }
    }

    /// <summary>
    /// Полоса захвата
    /// </summary>
    public class SatLane
    { 
        public List<LanePos> lanePoints;

        public List<LaneSector> Sectors;

        public SatLane(double minAngle, double maxAngle, List<LanePos> points)
        {
            Sectors = new List<LaneSector>();
            lanePoints = points;

            List<Vector3D> leftLanePoints = new List<Vector3D>();
            List<Vector3D> rightLanePoints = new List<Vector3D>();

            List<Vector3D> leftControlPoints = new List<Vector3D>();
            List<Vector3D> rightControlPoints = new List<Vector3D>();

            if (points.Count == 0)
                return;

            double width = (points[0].LeftCartPoint - points[0].RightCartPoint).Length / 2;
                 
            DateTime sectorFromDT = points[0].time;
            DateTime sectorToDT;

            GeoPoint prevPoint = AstronomyMath.GreenwichToSpherical(points[0].LeftCartPoint.ToPoint());
            for (int p_ind = 0; p_ind < points.Count; p_ind++)
            {
                leftLanePoints.Add(points[p_ind].LeftCartPoint);
                rightLanePoints.Add(points[p_ind].RightCartPoint);
                 
                leftControlPoints.Add(points[p_ind].leftControlPoint);
                rightControlPoints.Add(points[p_ind].rightControlPoint);

                if (p_ind == 0)
                    continue;

                GeoPoint point = GeoPoint.FromCartesian(points[p_ind].LeftCartPoint);

                bool needNewSector = false;

                if (p_ind == points.Count - 1)
                {
                    needNewSector = true;
                }                    
                else
                {
                    //GeoPoint nextPoint = GeoPoint.FromCartesian(points[p_ind + 1].LeftCartPoint);
                    double curDist = GeoPoint.DistanceOverSurface(prevPoint, point);
                    //double distNext = GeoPoint.DistanceOverSurface(prevPoint, nextPoint);
                    needNewSector = curDist > AstronomyMath.ToRad(90); // @todo плохой способ
                } 
                                                
                if (needNewSector)
                {
                    for (int i = rightLanePoints.Count - 1; i >= 0; i--)
                        leftLanePoints.Add(rightLanePoints[i]);

                    rightControlPoints[0] = new Vector3D(0, 0, 0); // торец по большому кругу
                    leftControlPoints[leftControlPoints.Count - 1] = new Vector3D(0, 0, 0); // торец по большому кругу
                    
                    for (int i = rightControlPoints.Count - 1; i >= 0; i--)
                        leftControlPoints.Add(rightControlPoints[i]);

                    sectorToDT = points[p_ind].time;
                    Polygon pol = new Polygon(leftLanePoints, leftControlPoints);
                    LaneSector newSector = new LaneSector();
                    newSector.polygon = pol;
                    newSector.fromDT = sectorFromDT;
                    newSector.toDT = sectorToDT;
                    Sectors.Add(newSector);

                    rightLanePoints.Clear();
                    leftLanePoints.Clear();

                    leftControlPoints.Clear();
                    rightControlPoints.Clear();

                    leftLanePoints.Add(points[p_ind].LeftCartPoint);
                    rightLanePoints.Add(points[p_ind].RightCartPoint);

                    leftControlPoints.Add(points[p_ind].leftControlPoint);
                    rightControlPoints.Add(points[p_ind].rightControlPoint);

                    sectorFromDT = sectorToDT;
                    prevPoint = point;
                    // break; ////
                }                                
            }
        }

        public List<CaptureConf> getCaptureConfs(RequestParams request)
        {
            Polygon region = new Polygon(request.wktPolygon);
            List<CaptureConf> res = new List<CaptureConf>();
            double square = region.Area;
            foreach(LaneSector sector in Sectors)
            {
                IList<Polygon> intersections = Polygon.Intersect(sector.polygon, region); 
                foreach (var int_pol in intersections)
                {
                    var verts = int_pol.Vertices;
                    var en = verts.GetEnumerator();
                    en.MoveNext();
                    DateTime tmin = getPointTime(en.Current, sector.fromDT, sector.toDT);
                    DateTime tmax = tmin; 
                    bool outOfRange = false;
                    foreach (var point in verts)
                    {
                        DateTime curTime = getPointTime(point, sector.fromDT, sector.toDT);
                        if (curTime < request.dateFrom || request.dateTo < curTime)
                        {
                            outOfRange = true;
                            break;
                        }                            
                        else if (curTime < tmin)
                        { 
                            tmin = curTime;
                        }
                        else if (curTime > tmax)
                        { 
                            tmax = curTime;
                        }
                    }                    
                    if (outOfRange)
                        break;
                    CaptureConf newcc = new CaptureConf();
                    Order order = new Order();
                    order.request = request;
                    double subsquare = int_pol.Area; 
                    order.intersection_coeff = subsquare / square;
                    newcc.orders.Add(order);
                    newcc.dateFrom = tmin;
                    newcc.dateTo = tmax;
                    res.Add(newcc);
                }
            }            
            return res;
        }

        // @todo убрать fromDt и toDt, функцию перевести в LaneSector
        public DateTime getPointTime(Vector3D point, DateTime fromDt, DateTime toDt)
        {
            GeoPoint geoPoint = GeoPoint.FromCartesian(point);
            double firstDist = lanePoints[0].getDistToPoint(geoPoint);        
            int first = 0;
            for (int i = 1; i < lanePoints.Count; i++)
            {
                if (lanePoints[i].time < fromDt)
                    continue;
                else if (lanePoints[i].time > toDt)
                    break;

                double dist = lanePoints[i].getDistToPoint(geoPoint);
                if (dist <= firstDist) 
                {
                    firstDist = dist;
                    first = i; 
                }
            }
            
            int second; 
            int sign;

            if (first == lanePoints.Count - 1)
            { 
                second = first - 1;  
            }
            else if (first == 0)
            {
                second = first + 1; 
            }
            else 
            { 
                 if (lanePoints[first - 1].getDistToPoint(geoPoint) < lanePoints[first + 1].getDistToPoint(geoPoint))                 
                     second = first - 1;
                 else
                     second = first + 1;                
            }
            
            double fullDist = Math.Abs(GeoPoint.DistanceOverSurface(lanePoints[first].MiddleGeoPoint, lanePoints[second].MiddleGeoPoint));
            double fullTime = Math.Abs((lanePoints[first].time - lanePoints[second].time).TotalSeconds);          
            double distPoint = Math.Abs(lanePoints[first].getDistToPoint(geoPoint));
            double diffSecs = fullTime * distPoint / fullDist;

            int outside = Math.Abs(lanePoints[second].getDistToPoint(geoPoint)) > fullDist ? 1 : -1;
            sign = (first < second) ? -outside : outside; 
             
            return lanePoints[first].time.AddSeconds(sign * diffSecs);    
        }
        

        public Polygon getSegment(DateTime begTime, DateTime endTime)
        {
            if (lanePoints[0].time > begTime || lanePoints[lanePoints.Count - 1].time < endTime)
                throw new System.ArgumentException("Incorrect time interval.");

            List<Vector3D> polygonPoints = new List<Vector3D>();
            List<Vector3D> rightPolygonPoints = new List<Vector3D>();

            for (int i = 0; i < lanePoints.Count; i++)
            { 

                if (begTime <= lanePoints[i].time && lanePoints[i].time <= endTime)
                {
                    polygonPoints.Add(lanePoints[i].LeftCartPoint);
                    rightPolygonPoints.Add(lanePoints[i].RightCartPoint);
                }

                if (lanePoints.Count - 1 == i)  
                    break;

                if (lanePoints[i + 1].time < begTime)
                    continue;

                if (lanePoints[i].time < begTime && begTime < lanePoints[i + 1].time)
                {
                    var pos = getlanePosByTime(lanePoints[i], lanePoints[i + 1], begTime);
                    polygonPoints.Add(pos.Item1);
                    rightPolygonPoints.Add(pos.Item2);
                }

                if (lanePoints[i].time < endTime && endTime < lanePoints[i + 1].time)
                {
                    var pos = getlanePosByTime(lanePoints[i], lanePoints[i + 1], endTime);
                    polygonPoints.Add(pos.Item1);
                    rightPolygonPoints.Add(pos.Item2);
                }
                 
                if (lanePoints[i + 1].time > endTime)
                    break;

                // @todo полигон может быть длиннее 180 (upd - по исходным данным вроде не должен), что вызовет ошибки. Надо разрезать на полигоны или вставить проверку. 
            }

            for (int i = rightPolygonPoints.Count - 1; i >= 0; i--)
                polygonPoints.Add(rightPolygonPoints[i]);

            return new Polygon(polygonPoints, new Vector3D(0, 0, 0));
        }

        private Tuple<Vector3D,Vector3D> getlanePosByTime(LanePos first, LanePos second, DateTime time)
        {
            Vector3D newMiddlePoint = SatTrajectory.getInterpolPoint(first.MiddleCartPoint, second.MiddleCartPoint, first.time, second.time, time);

            Vector3D newLeft = newMiddlePoint + (first.LeftCartPoint - first.MiddleCartPoint);
            Vector3D newRight = newMiddlePoint + (first.RightCartPoint - first.MiddleCartPoint);

            return new Tuple<Vector3D, Vector3D>(newLeft, newRight);
        }
        
    }
         
    public class LanePos
    {
        private Vector3D leftCartPoint;
        private Vector3D rightCartPoint;
        private Vector3D middleCartPoint;
        public Vector3D leftControlPoint;
        public Vector3D rightControlPoint;
        
        private GeoPoint leftGeoPoint;
        private GeoPoint rightGeoPoint;
        private GeoPoint middleGeoPoint;

        private double width; 

        public DateTime time;

        public LanePos(Vector3D _leftPoint, Vector3D _middlePoint, Vector3D _rightPoint, DateTime _time)
        {
            leftCartPoint = _leftPoint;
            rightCartPoint = _rightPoint;
            middleCartPoint = _middlePoint;
            middleGeoPoint = GeoPoint.FromCartesian(_middlePoint);
            time = _time;

            leftGeoPoint = GeoPoint.FromCartesian(leftCartPoint);
            rightGeoPoint = GeoPoint.FromCartesian(rightCartPoint);
            width = GeoPoint.DistanceOverSurface(leftGeoPoint, rightGeoPoint);

            // leftControlPoint = leftCartPoint - middleCartPoint;
            // rightControlPoint = rightCartPoint - middleCartPoint;

            leftControlPoint = getControlPoint(middleCartPoint, leftCartPoint);
            rightControlPoint = getControlPoint(middleCartPoint, rightCartPoint);
 
            //// неверно для полос захвата
            /*
            leftControlPoint = leftCartPoint - rightCartPoint;
            leftControlPoint.Normalize();
            leftControlPoint = leftControlPoint * (leftCartPoint - rightCartPoint).Length / 2;

            rightControlPoint = rightCartPoint - leftCartPoint; 
            rightControlPoint.Normalize();
            rightControlPoint = rightControlPoint * (rightCartPoint - leftCartPoint).Length / 2;
             */
        }

        private Vector3D getControlPoint(Vector3D middlePoint, Vector3D sidePoint)
        {
            Vector3D rotAx = Vector3D.CrossProduct(middlePoint, sidePoint);
            RotateTransform3D leftTransform = new RotateTransform3D(new AxisAngleRotation3D(rotAx, 90));
            Vector3D controlPoint = leftTransform.Transform(middlePoint);
            double lenth = Vector3D.DotProduct(sidePoint, controlPoint) / controlPoint.Length; 
            controlPoint.Normalize();
            controlPoint = controlPoint * lenth;
            return controlPoint;
        }    

        public Vector3D LeftCartPoint
        {
            get
            {
                return leftCartPoint;
            } 
        }

        public Vector3D RightCartPoint
        {
            get
            {
                return rightCartPoint;
            } 
        }

        public Vector3D MiddleCartPoint
        {
            get
            {
                return middleCartPoint;
            }
        }        

        public GeoPoint LeftGeoPoint
        {
            get
            {
                return leftGeoPoint;
            }
        }

        public GeoPoint RightGeoPoint
        {
            get
            {
                return rightGeoPoint;
            }
        }

        public GeoPoint MiddleGeoPoint
        {
            get
            {
                return middleGeoPoint;
            }
        }
         
        /// <summary>
        /// Return distance from gpoint to line [leftGeoPoint - rightGeoPoint] in radians
        /// </summary>
        /// <param name="gpoint"></param>
        /// <returns> distance in  in radians</returns>
        public double getDistToPoint(GeoPoint gpoint)
        {
            if (gpoint == leftGeoPoint || gpoint == rightGeoPoint)
            {
                return 0;
            }             
            double A = GeoPoint.DistanceOverSurface(gpoint, leftGeoPoint);
            double B = GeoPoint.DistanceOverSurface(gpoint, rightGeoPoint);
            double C = width;
            if (A + B == C)
                return 0;
            double top = (Math.Cos(A) - Math.Cos(B) * Math.Cos(C));
            double low = (Math.Sin(B) * Math.Sin(C));
            double beta = Math.Acos(top / low);
            double h = Math.Asin(Math.Sin(B) * Math.Sin(beta));  
            return h;
        }

    }

    public class TargetWorkInterval
    {
        public DateTime fromDt;
        public DateTime toDt;
        public double minAngle;
        public double maxAngle; 
    }

    public struct LaneSector
    {
        public DateTime fromDT;
        public DateTime toDT;
        public Polygon polygon;
    }


}

