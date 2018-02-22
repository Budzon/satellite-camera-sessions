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
    /// <summary>
    /// Полоса захвата
    /// </summary>
    public class SatLane
    {
        // public List<LanePos> lanePoints;

        public List<LaneSector> Sectors { get; set; }
        public double RollAngle { get { return rollAngle; } }
        public double ViewAngle { get { return viewAngle; } }

        private Astronomy.Trajectory trajectory;
        private double rollAngle;
        private double viewAngle;
 
        public SatLane(Astronomy.Trajectory _trajectory, double _rollAngle, double _viewAngle, int polygonStep = 15)
        {
            trajectory = _trajectory;
            rollAngle = _rollAngle;
            viewAngle = _viewAngle;

            double minAngle = rollAngle - viewAngle / 2;
            double maxAngle = rollAngle + viewAngle / 2;
             
            var points = trajectory.Points;
            var count = trajectory.Count;

            createPolygons(minAngle, maxAngle, polygonStep);
        }

        
        private void createPolygons(double minAngle, double maxAngle, int polygonStep)
        {
            Sectors = new List<LaneSector>();
             
            List<Vector3D> leftLanePoints = new List<Vector3D>();
            List<Vector3D> rightLanePoints = new List<Vector3D>();

            List<Vector3D> leftControlPoints = new List<Vector3D>();
            List<Vector3D> rightControlPoints = new List<Vector3D>();

            if (trajectory.Count == 0)
                return;

            TrajectoryPoint[] points = trajectory.Points;
            int points_count = trajectory.Count;

            LanePos firstPos = new LanePos(points[0], viewAngle, rollAngle);
            //double width = (firstPos.LeftCartPoint - firstPos.RightCartPoint).Length / 2;

            DateTime sectorFromDT = firstPos.Time;
            DateTime sectorToDT;
            List<LanePos> sectorPoints = new List<LanePos>();

            GeoPoint prevPoint = AstronomyMath.GreenwichToSpherical(firstPos.LeftCartPoint.ToPoint());

            for (int p_ind = 0; p_ind < points_count ; p_ind++)
            {
                LanePos pos = new LanePos(points[p_ind], viewAngle, rollAngle); 
                sectorPoints.Add(pos);

                if (p_ind == 0)
                {
                    leftLanePoints.Add(pos.LeftCartPoint);
                    rightLanePoints.Add(pos.RightCartPoint);

                    leftControlPoints.Add(pos.LeftControlPoint);
                    rightControlPoints.Add(pos.RightControlPoint);
                    continue;
                }

                GeoPoint point = GeoPoint.FromCartesian(pos.LeftCartPoint);
                bool needNewSector = false;

                if (p_ind == points_count - 1)
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

                if (p_ind % polygonStep == 0 || needNewSector)
                {
                    leftLanePoints.Add(pos.LeftCartPoint);
                    rightLanePoints.Add(pos.RightCartPoint);

                    leftControlPoints.Add(pos.LeftControlPoint);
                    rightControlPoints.Add(pos.RightControlPoint);
                }

                if (needNewSector)
                {
                    for (int i = rightLanePoints.Count - 1; i >= 0; i--)
                        leftLanePoints.Add(rightLanePoints[i]);

                    rightControlPoints[0] = new Vector3D(0, 0, 0); // торец по большому кругу
                    leftControlPoints[leftControlPoints.Count - 1] = new Vector3D(0, 0, 0); // торец по большому кругу

                    for (int i = rightControlPoints.Count - 1; i >= 0; i--)
                        leftControlPoints.Add(rightControlPoints[i]);

                    sectorToDT = points[p_ind].Time;
                    Polygon pol = new Polygon(leftLanePoints, leftControlPoints);
                    //var testpol = new Polygon(pol.ToWtk());
                    LaneSector newSector = new LaneSector();
                    newSector.polygon = pol;
                    newSector.fromDT = sectorFromDT;
                    newSector.toDT = sectorToDT;
                    newSector.sectorPoints = sectorPoints;
                    Sectors.Add(newSector);

                    sectorPoints = new List<LanePos>();//.Clear();

                    rightLanePoints.Clear();
                    leftLanePoints.Clear();

                    leftControlPoints.Clear();
                    rightControlPoints.Clear();

                    sectorPoints.Add(pos);

                    leftLanePoints.Add(pos.LeftCartPoint);
                    rightLanePoints.Add(pos.RightCartPoint);

                    leftControlPoints.Add(pos.LeftControlPoint);
                    rightControlPoints.Add(pos.RightControlPoint);

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
            foreach (LaneSector sector in Sectors)
            {
                IList<Polygon> intersections = Polygon.Intersect(sector.polygon, region);
                foreach (var int_pol in intersections)
                {
                    var verts = int_pol.Vertices;
                    var en = verts.GetEnumerator();
                    en.MoveNext();
                    DateTime tmin = sector.getPointTime(en.Current);
                    DateTime tmax = tmin;
                    bool outOfRange = false;
                    foreach (var point in verts)
                    {
                        DateTime curTime = sector.getPointTime(point);
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
                    order.captured = int_pol;
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

        public Tuple<Polygon, TrajectoryPoint, TrajectoryPoint>  getSegment(DateTime begTime, DateTime endTime)
        {
            if (Sectors.Count < 1)
                return null;

            var lastSector = Sectors[Sectors.Count - 1];
            var lastPoint = lastSector.sectorPoints[lastSector.sectorPoints.Count - 1];

            if (Sectors[0].sectorPoints[0].Time > begTime || lastPoint.Time < endTime)
                throw new System.ArgumentException("Incorrect time interval.");

            List<Vector3D> polygonPoints = new List<Vector3D>();
            List<Vector3D> rightPolygonPoints = new List<Vector3D>();

            bool found_beg = false, found_end = false;
            foreach (var sector in Sectors)
            {
                var sectorPoints = sector.sectorPoints;
                for (int i = 0; i < sectorPoints.Count; i++)
                {
                    if (sectorPoints[i].Time < begTime)
                        continue;

                    if (sectorPoints[i].Time > endTime)
                        break;

                    if (begTime < sectorPoints[i].Time && sectorPoints[i].Time < endTime)
                    {
                        polygonPoints.Add(sectorPoints[i].LeftCartPoint);
                        rightPolygonPoints.Add(sectorPoints[i].RightCartPoint);
                        continue;
                    }

                    if (begTime == sectorPoints[i].Time && sectorPoints[i].Time == endTime)
                    {
                        polygonPoints.Add(sectorPoints[i].LeftCartPoint);
                        rightPolygonPoints.Add(sectorPoints[i].RightCartPoint);

                        if (begTime == sectorPoints[i].Time)
                            found_beg = true;
                        if (endTime == sectorPoints[i].Time)
                            found_end = true;
                    }
                }
            }

            if (!found_beg)
            {
                LanePos posFrom = interpolatelanePosByTime(begTime);
                polygonPoints.Insert(0, posFrom.LeftCartPoint);
                rightPolygonPoints.Insert(0, posFrom.RightCartPoint);
            }

            if (!found_end)
            {
                LanePos posTo = interpolatelanePosByTime(endTime);
                polygonPoints.Add(posTo.LeftCartPoint);
                rightPolygonPoints.Add(posTo.RightCartPoint);
            }

            for (int ind = rightPolygonPoints.Count - 1; ind >= 0; ind--)
                polygonPoints.Add(rightPolygonPoints[ind]);

            TrajectoryPoint pointFrom = trajectory.GetPoint(begTime);
            TrajectoryPoint pointTo = trajectory.GetPoint(endTime); 
            Polygon segmenPol = new Polygon(polygonPoints, new Vector3D(0, 0, 0));
            return  Tuple.Create(segmenPol, pointFrom, pointTo);
        }
        
        /*
        private Tuple<Vector3D, Vector3D> getlanePosByTime(LanePos first, LanePos second, DateTime time)
        {
            Vector3D newMiddlePoint = SatTrajectory.getInterpolPoint(first.MiddleCartPoint, second.MiddleCartPoint, first.Time, second.Time, time);

            Vector3D newLeft = newMiddlePoint + (first.LeftCartPoint - first.MiddleCartPoint);
            Vector3D newRight = newMiddlePoint + (first.RightCartPoint - first.MiddleCartPoint);

            return new Tuple<Vector3D, Vector3D>(newLeft, newRight);
        }
        */

        private LanePos interpolatelanePosByTime(DateTime time)
        { 
            TrajectoryPoint trajPoint = trajectory.GetPoint(time); 
            return new LanePos(trajPoint, viewAngle, rollAngle);
        }

        /*
        public LanePos this[int ind]
        {
            get
            {
                int global_ind = 0;
                foreach (var sector in Sectors)
                {
                    int scount = sector.sectorPoints.Count;
                    global_ind += scount;
                    int sect_ind = ind + scount - global_ind;
                    if (sect_ind >= 0)
                    {
                        return sector.sectorPoints[sect_ind];
                    }
                }
                return null;
            }
        }
        */
    }


    public class LaneSector
    {
        public DateTime fromDT { get; set; }
        public DateTime toDT { get; set; }
        public Polygon polygon { get; set; }
        public List<LanePos> sectorPoints { get; set; }

        public DateTime getPointTime(Vector3D point)
        {
            GeoPoint geoPoint = GeoPoint.FromCartesian(point);
            double firstDist = sectorPoints[0].getDistToPoint(geoPoint);
            int first = 0;
            for (int i = 1; i < sectorPoints.Count; i++)
            {
                double dist = sectorPoints[i].getDistToPoint(geoPoint);
                if (dist <= firstDist)
                {
                    firstDist = dist;
                    first = i;
                }
            }

            int second;
            int sign;

            if (first == sectorPoints.Count - 1)
            {
                second = first - 1;
            }
            else if (first == 0)
            {
                second = first + 1;
            }
            else
            {
                if (sectorPoints[first - 1].getDistToPoint(geoPoint) < sectorPoints[first + 1].getDistToPoint(geoPoint))
                    second = first - 1;
                else
                    second = first + 1;
            }

            double distToFirst = Math.Abs(sectorPoints[first].getDistToPoint(geoPoint));
            double distToSecond = Math.Abs(sectorPoints[second].getDistToPoint(geoPoint));
            double fullDist = distToFirst + distToSecond; // Math.Abs(sectorPoints[first].getDistToPoint(sectorPoints[second].LeftGeoPoint));   //  Math.Abs(GeoPoint.DistanceOverSurface(sectorPoints[first].MiddleGeoPoint, sectorPoints[second].MiddleGeoPoint));
            double fullTime = Math.Abs((sectorPoints[first].Time - sectorPoints[second].Time).TotalMilliseconds);
            double diffMiliSecs = fullTime * distToFirst / fullDist;

            int outside = distToSecond > fullDist ? 1 : -1;
            sign = (first < second) ? -outside : outside;

            return sectorPoints[first].Time.AddMilliseconds(sign * diffMiliSecs);
            /*
            double fullDist = Math.Abs(sectorPoints[first].getDistToPoint(sectorPoints[second].LeftGeoPoint));   //  Math.Abs(GeoPoint.DistanceOverSurface(sectorPoints[first].MiddleGeoPoint, sectorPoints[second].MiddleGeoPoint));
            double fullTime = Math.Abs((sectorPoints[first].time - sectorPoints[second].time).TotalMilliseconds);
            double distPoint = Math.Abs(sectorPoints[first].getDistToPoint(geoPoint));
            double diffMiliSecs = fullTime * distPoint / fullDist;

            int outside = Math.Abs(sectorPoints[second].getDistToPoint(geoPoint)) > fullDist ? 1 : -1;
            sign = (first < second) ? -outside : outside;

            return sectorPoints[first].time.AddMilliseconds(sign * diffMiliSecs);
             */
        }
    }

    /// <summary>
    /// класс, содержащий в себе положение в полосе в момент времени, то есть "правую" и "левую" точку и время этого положения
    /// </summary>
    public class LanePos
    {
        enum RollOrientation
        { // в какую сторону от надир осуществлен наклон съемки
            left, right
        };

        private Vector3D leftCartPoint;
        private Vector3D rightCartPoint;
        private Vector3D middleCartPoint;
        private Vector3D leftControlPoint;
        private Vector3D rightControlPoint;
        private bool knowLeftConrol;
        private bool knowRightConrol;

        private GeoPoint leftGeoPoint;
        private GeoPoint rightGeoPoint;
        private GeoPoint middleGeoPoint;
        private TrajectoryPoint trajPoint;

        private double width;
        private DateTime time;

        private RollOrientation roll;


        public static Vector3D applyPitchlRotation(TrajectoryPoint point, Vector3D dirVect, double pitchAngle)
        {
            Vector3D position = point.Position.ToVector();
            Vector3D velo = point.Velocity;
            Vector3D eDirVect = -position; 
            Vector3D pitchAxis = Vector3D.CrossProduct(eDirVect, velo); 
            RotateTransform3D pitchTransfrom = new RotateTransform3D(new AxisAngleRotation3D(pitchAxis, AstronomyMath.ToDegrees(pitchAngle))); 
            Vector3D pitchRotDir = pitchTransfrom.Transform(dirVect); 
            return pitchRotDir;
        }

        public static Vector3D applyRollRotation(TrajectoryPoint point, Vector3D dirVect, double rollAngle)
        {
            Vector3D position = point.Position.ToVector();
            Vector3D velo = point.Velocity;      
            //Vector3D pitchAxis =  Vector3D.CrossProduct(dirVect, velo);
            //Vector3D rollAxis = Vector3D.CrossProduct(pitchAxis, dirVect);
            RotateTransform3D rollTransfrom = new RotateTransform3D(new AxisAngleRotation3D(-velo, AstronomyMath.ToDegrees(rollAngle)));
            Vector3D rollRotDir = rollTransfrom.Transform(dirVect);
            return rollRotDir;
        }
        
        public static Vector3D getDirectionVector_TEST(TrajectoryPoint point, double rollAngle, double pitchAngle)
        {
            Vector3D eDirVect = -point.Position.ToVector();
            Vector3D pitchDir = applyPitchlRotation(point, eDirVect, pitchAngle);
            Vector3D resDir = applyRollRotation(point, pitchDir, rollAngle);            
            return resDir;
        }

        public static Vector3D getDirectionVector(TrajectoryPoint point, double rollAngle, double pitchAngle)
        {
            Vector3D eDirVect = -point.Position.ToVector();
            Vector3D rollDir = applyRollRotation(point, eDirVect, rollAngle);
            return applyPitchlRotation(point, rollDir, pitchAngle);

        }

        public static Vector3D getSurfacePoint(TrajectoryPoint point, double rollAngle, double pitchAngle)
        {
            Vector3D dirVector = getDirectionVector(point, rollAngle, pitchAngle);
            return Routines.SphereVectIntersect(dirVector, point.Position, Astronomy.Constants.EarthRadius);
        }

        public LanePos(Vector3D _leftPoint, Vector3D _middlePoint, Vector3D _rightPoint, DateTime _time)
        {
            knowLeftConrol = false;
            knowRightConrol = false;

            leftCartPoint = _leftPoint;
            rightCartPoint = _rightPoint;
            middleCartPoint = _middlePoint;
            middleGeoPoint = GeoPoint.FromCartesian(_middlePoint);
            
            time = _time;
            //TrajPoint = new TrajectoryPoint(,,_time);

            leftGeoPoint = GeoPoint.FromCartesian(leftCartPoint);
            rightGeoPoint = GeoPoint.FromCartesian(rightCartPoint);
            width = GeoPoint.DistanceOverSurface(leftGeoPoint, rightGeoPoint);
        }

        public LanePos(TrajectoryPoint pointKA, double viewAngle, double rollAngle)
        {            
            double minAngle = rollAngle - viewAngle / 2;
            double maxAngle = rollAngle + viewAngle / 2;
            
            Vector3D eDirVect = new Vector3D(-pointKA.Position.X, -pointKA.Position.Y, -pointKA.Position.Z);
            
            RotateTransform3D leftTransform = new RotateTransform3D(new AxisAngleRotation3D(pointKA.Velocity, AstronomyMath.ToDegrees(minAngle)));
            RotateTransform3D rightTransform = new RotateTransform3D(new AxisAngleRotation3D(pointKA.Velocity, AstronomyMath.ToDegrees(maxAngle)));
           
            Vector3D leftVector = leftTransform.Transform(eDirVect);
            Vector3D rightVector = rightTransform.Transform(eDirVect);

            Vector3D leftCrossPoint = Routines.SphereVectIntersect(leftVector, pointKA.Position, Astronomy.Constants.EarthRadius);
            Vector3D rightCrossPoint = Routines.SphereVectIntersect(rightVector, pointKA.Position, Astronomy.Constants.EarthRadius);
            Vector3D middlePoint = pointKA.Position.ToVector();
            
            //var tanHalfVa = Math.Tan(viewAngle / 2);
            //var angle_rad = Math.Atan(tanHalfVa / (Math.Sqrt(tanHalfVa * tanHalfVa + 1)));
            //var angle_degr = AstronomyMath.ToDegrees(angle_rad);

            //Vector3D rotAxis = Vector3D.CrossProduct(rightVector, pointKA.Velocity);

            //RotateTransform3D rightBotTransfrom = new RotateTransform3D(new AxisAngleRotation3D(rotAxis, angle_degr));
            //Vector3D rightBotCrossVector = rightBotTransfrom.Transform(rightVector);
            //Vector3D rightBotCrossPoint = Routines.SphereVectIntersect(rightBotCrossVector, pointKA.Position, Astronomy.Constants.EarthRadius);

            //rotAxis = Vector3D.CrossProduct(leftVector, pointKA.Velocity);

            //RotateTransform3D leftBotTransfrom = new RotateTransform3D(new AxisAngleRotation3D(rotAxis, -angle_degr));
            //Vector3D leftBotCrossVector = leftBotTransfrom.Transform(leftVector);
            //Vector3D leftBotCrossPoint = Routines.SphereVectIntersect(leftBotCrossVector, pointKA.Position, Astronomy.Constants.EarthRadius);

            //double distRight = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(rightBotCrossPoint), GeoPoint.FromCartesian(rightCrossPoint));
            //double distLeft = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(leftBotCrossPoint), GeoPoint.FromCartesian(leftCrossPoint));

            //double dist = Math.Abs(distRight - distLeft);

            // dist идет в lanPos, либо как правая, либо как левая

            leftCrossPoint.Normalize();
            rightCrossPoint.Normalize();
            middlePoint.Normalize();

            //LanePos(leftCrossPoint, rightCrossPoint, pointKA);            

            trajPoint = pointKA;
            leftCartPoint = leftCrossPoint;
            rightCartPoint = rightCrossPoint;
            middleCartPoint = pointKA.Position.ToVector();
            middleCartPoint.Normalize();
            middleGeoPoint = GeoPoint.FromCartesian(middleCartPoint);

            leftGeoPoint = GeoPoint.FromCartesian(leftCartPoint);
            rightGeoPoint = GeoPoint.FromCartesian(rightCartPoint);
            width = GeoPoint.DistanceOverSurface(leftGeoPoint, rightGeoPoint);

            time = pointKA.Time;
            // leftControlPoint = leftCartPoint - middleCartPoint;
            // rightControlPoint = rightCartPoint - middleCartPoint;

            //leftControlPoint = getControlPoint(middleCartPoint, leftCartPoint);
            //rightControlPoint = getControlPoint(middleCartPoint, rightCartPoint);
            knowLeftConrol = false;
            knowRightConrol = false;
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

        public Vector3D LeftControlPoint
        {
            get
            {
                if (knowLeftConrol)
                    return leftControlPoint;
                else
                {                   
                    leftControlPoint = getControlPoint(middleCartPoint, leftCartPoint);
                    knowLeftConrol = true;
                    return leftControlPoint;
                }
            }
        }

        public Vector3D RightControlPoint
        {
            get
            {
                if (knowRightConrol)
                    return rightControlPoint;
                else
                {                    
                    rightControlPoint = getControlPoint(middleCartPoint, rightCartPoint);
                    knowRightConrol = true;
                    return rightControlPoint;
                }
            }
        }

        public TrajectoryPoint TrajPoint{ get{ return trajPoint; } }

        public DateTime Time{ get{ return time; } }

        public Vector3D LeftCartPoint{ get{ return leftCartPoint; } }

        public Vector3D RightCartPoint{ get{ return rightCartPoint; } }

        public Vector3D MiddleCartPoint{ get{ return middleCartPoint; } }

        public GeoPoint LeftGeoPoint{ get{ return leftGeoPoint; } }

        public GeoPoint RightGeoPoint{ get{ return rightGeoPoint; } }

        public GeoPoint MiddleGeoPoint{ get{  return middleGeoPoint; } }

        /// <summary>
        /// Return distance from gpoint to line [leftGeoPoint - rightGeoPoint] in radians
        /// </summary>
        /// <param name="gpoint"></param>
        /// <returns> distance in  in radians</returns>
        public double getDistToPoint(GeoPoint gpoint)
        {
            if (gpoint == leftGeoPoint || gpoint == rightGeoPoint || gpoint == middleGeoPoint)
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
            if (Double.IsNaN(beta)) // too close -> dist == 0
                return 0;
            double h = Math.Asin(Math.Sin(B) * Math.Sin(beta));
            return h;
        }

    }

    public class TargetWorkInterval
    {
        public DateTime fromDt { get; set; }
        public DateTime toDt { get; set; }
        public double minAngle { get; set; }
        public double maxAngle { get; set; }
    }


    public class TrajectoryRoutines
    {
        public static List<PolygonLit> GetOneSatTurnLitParts(List<LanePos> turn)
        {
            List<PolygonLit> res = new List<PolygonLit> { };

            bool onLitStreak = false;
            int streakBegin = -1;

            for (int i = 0; i < turn.Count - 1; ++i)
            {
                Vector3D sun = Astronomy.SunPosition.GetPositionGreenwich(turn[i].Time).ToVector();
                Polygon sector = FormSectorFromLanePoints(turn, i, i + 1);

                var LitAndNot = Polygon.IntersectAndSubtract(sector, Polygon.Hemisphere(sun));
                bool allLit = LitAndNot.Item2.Count == 0;
                bool allUnlit = LitAndNot.Item1.Count == 0;

                if (streakBegin != -1)
                {
                    // On streak -- either continue one or make a master-sector.
                    if ((allLit && onLitStreak) || (allUnlit && !onLitStreak))
                    {
                        continue;
                    }
                    else
                    {
                        Polygon masterSector = FormSectorFromLanePoints(turn, streakBegin, i);
                        res.Add(new PolygonLit { Polygon = masterSector, Lit = onLitStreak });
                        streakBegin = -1;
                    }
                }

                // Not on streak here -- either start one or just add to output lists.
                if (allLit)
                {
                    onLitStreak = true;
                    streakBegin = i;
                }
                else if (allUnlit) // totally unlit
                {
                    onLitStreak = false;
                    streakBegin = i;
                }
                else
                {
                    foreach (Polygon p in LitAndNot.Item1)
                        res.Add(new PolygonLit { Polygon = p, Lit = true });
                    foreach (Polygon p in LitAndNot.Item2)
                        res.Add(new PolygonLit { Polygon = p, Lit = false });
                }                
            }

            return res;
        }

        public static Polygon FormSectorFromLanePoints(List<LanePos> turn, int from, int to, int step = 1)
        {
            List<Vector3D> vertices = new List<Vector3D>();
            List<Vector3D> apexes = new List<Vector3D>();

            for (int i = from; i < to; i += step)
            {
                vertices.Add(turn[i].RightCartPoint);
                apexes.Add(turn[i].RightControlPoint);
            }
            vertices.Add(turn[to].RightCartPoint);
            apexes.Add(new Vector3D(0, 0, 0));
            for (int i = to; i > from; i -= step)
            {
                vertices.Add(turn[i].LeftCartPoint);
                apexes.Add(turn[i].LeftControlPoint);
            }
            vertices.Add(turn[from].LeftCartPoint);
            apexes.Add(new Vector3D(0, 0, 0));

            return new Polygon(vertices, apexes);
        }
    }

    public struct PolygonLit
    {
        public Polygon Polygon { get; set; }
        public bool Lit { get; set; }
    }
}

