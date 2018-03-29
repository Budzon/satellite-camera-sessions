using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;

using SphericalGeom;
using SatelliteRequests;
using SatelliteSessions;
using Common;
using Astronomy;
using OptimalChain;
using Microsoft.Research.Oslo;

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
        private double pitchAngle;

        public SatLane(Astronomy.Trajectory _trajectory, double _rollAngle, double _pitchAngle, double _viewAngle, int polygonStep = OptimalChain.Constants.stripPolygonStep)
        {
            trajectory = _trajectory;
            rollAngle = _rollAngle;
            viewAngle = _viewAngle;
            pitchAngle = _pitchAngle;

            double minAngle = rollAngle - viewAngle / 2;
            double maxAngle = rollAngle + viewAngle / 2;

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

            LanePos firstPos = new LanePos(points[0], viewAngle, rollAngle, 0);

            DateTime sectorFromDT = firstPos.Time;
            DateTime sectorToDT;
            List<LanePos> sectorPoints = new List<LanePos>();

            GeoPoint prevPoint = AstronomyMath.GreenwichToSpherical(firstPos.LeftCartPoint.ToPoint());

            for (int p_ind = 0; p_ind < points_count ; p_ind++)
            {
                LanePos pos = new LanePos(points[p_ind], viewAngle, rollAngle, pitchAngle); 
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
                        if (curTime < request.timeFrom || request.timeTo < curTime)
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
                    
                    if (tmin >= tmax)
                        continue;

                    Order order = new Order();
                    order.captured = int_pol;
                    order.request = request;
                    double subsquare = int_pol.Area;
                    order.intersection_coeff = subsquare / square;
                    var orders = new List<Order>() { order };
                    int type = 0;
                    CaptureConf newcc = new CaptureConf(tmin, tmax, rollAngle, orders, type, null);

                    res.Add(newcc);
                }
            }
            return res;
        }

        public Polygon getSegment(DateTime begTime, DateTime endTime)
        {
            if (Sectors.Count < 1)
                return null;

            if (begTime >= endTime)
                throw new ArgumentException("Incorrect time interval");

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

                int i = 0;
                if (polygonPoints.Count > 0) // уже начался полигон
                    i = 1;

                for (; i < sectorPoints.Count; i++)
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

            return new Polygon(polygonPoints, new Vector3D(0, 0, 0));            
        }

        private LanePos interpolatelanePosByTime(DateTime time)
        { 
            TrajectoryPoint trajPoint = trajectory.GetPoint(time); 
            return new LanePos(trajPoint, viewAngle, rollAngle, 0);
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
            double fullDist = distToFirst + distToSecond; // Math.Abs(sectorPoints[first].getDistToPoint(sectorPoints[second].LeftGeoPoint));   //  Math.Abs(GeoPoint.DistanceOverSurface(sectorPoints[first].KAGeoPoint, sectorPoints[second].KAGeoPoint));
            double fullTime = Math.Abs((sectorPoints[first].Time - sectorPoints[second].Time).TotalMilliseconds);
            double diffMiliSecs = fullTime * distToFirst / fullDist;

            int outside = distToSecond > fullDist ? 1 : -1;
            sign = (first < second) ? -outside : outside;

            return sectorPoints[first].Time.AddMilliseconds(sign * diffMiliSecs);
            /*
            double fullDist = Math.Abs(sectorPoints[first].getDistToPoint(sectorPoints[second].LeftGeoPoint));   //  Math.Abs(GeoPoint.DistanceOverSurface(sectorPoints[first].KAGeoPoint, sectorPoints[second].KAGeoPoint));
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
        //enum RollOrientation
        //{ // в какую сторону от надир осуществлен наклон съемки
        //    left, right
        //};

        private Vector3D leftCartPoint;
        private Vector3D rightCartPoint;
        private Vector3D cartKAPoint;
        private Vector3D leftControlPoint;
        private Vector3D rightControlPoint;
        private bool knowLeftConrol;
        private bool knowRightConrol;
        private bool knowViewPolygon;

        private GeoPoint leftGeoPoint;
        private GeoPoint rightGeoPoint;
        private GeoPoint geoKAPoint;
        private TrajectoryPoint trajPoint;
        private double rollAngle;
        private double pitchAngle;

        private double width;
        private DateTime time;

       //  private RollOrientation rollOrient;

        private Vector3D topLeftViewPoint;
        private Vector3D botLeftViewPoint;
        private Vector3D topRightViewPoint;
        private Vector3D botRightViewPoint;

        public LanePos(Vector3D _leftPoint, Vector3D _kaPoint, Vector3D _rightPoint, DateTime _time)
        {
            knowLeftConrol = false;
            knowRightConrol = false;
            knowViewPolygon = false;  

            leftCartPoint = _leftPoint;
            rightCartPoint = _rightPoint;
            cartKAPoint = _kaPoint;
            geoKAPoint = GeoPoint.FromCartesian(_kaPoint);

            time = _time;
            //TrajPoint = new TrajectoryPoint(,,_time);

            leftGeoPoint = GeoPoint.FromCartesian(leftCartPoint);
            rightGeoPoint = GeoPoint.FromCartesian(rightCartPoint);
            width = GeoPoint.DistanceOverSurface(leftGeoPoint, rightGeoPoint);
        }

        public LanePos(TrajectoryPoint pointKA, double viewAngle, double _rollAngle, double _pitchAngle)
        {
            rollAngle = _rollAngle;
            pitchAngle = _pitchAngle;

            double minAngle = rollAngle - viewAngle / 2;
            double maxAngle = rollAngle + viewAngle / 2;

            Vector3D eDirVect = new Vector3D(-pointKA.Position.X, -pointKA.Position.Y, -pointKA.Position.Z);

            RotateTransform3D leftTransform = new RotateTransform3D(new AxisAngleRotation3D(pointKA.Velocity, AstronomyMath.ToDegrees(minAngle)));
            RotateTransform3D rightTransform = new RotateTransform3D(new AxisAngleRotation3D(pointKA.Velocity, AstronomyMath.ToDegrees(maxAngle)));
             
            Vector3D pitchAxis = Vector3D.CrossProduct(pointKA.Velocity, eDirVect);
            RotateTransform3D pitchTransform = new RotateTransform3D(new AxisAngleRotation3D(pitchAxis, AstronomyMath.ToDegrees(pitchAngle)));

            Vector3D leftVector = pitchTransform.Transform(leftTransform.Transform(eDirVect));
            Vector3D rightVector = pitchTransform.Transform(rightTransform.Transform(eDirVect));

           // Vector3D leftVector = leftTransform.Transform(eDirVect);
           // Vector3D rightVector = rightTransform.Transform(eDirVect);

            Vector3D leftCrossPoint = Routines.SphereVectIntersect(leftVector, pointKA.Position, Astronomy.Constants.EarthRadius);
            Vector3D rightCrossPoint = Routines.SphereVectIntersect(rightVector, pointKA.Position, Astronomy.Constants.EarthRadius);
            Vector3D KAPoint = pointKA.Position.ToVector();

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
            KAPoint.Normalize();

            //LanePos(leftCrossPoint, rightCrossPoint, pointKA);            

            trajPoint = pointKA;
            leftCartPoint = leftCrossPoint;
            rightCartPoint = rightCrossPoint;
            cartKAPoint = pointKA.Position.ToVector();
            CartKAPoint.Normalize();
            geoKAPoint = GeoPoint.FromCartesian(CartKAPoint);

            leftGeoPoint = GeoPoint.FromCartesian(leftCartPoint);
            rightGeoPoint = GeoPoint.FromCartesian(rightCartPoint);
            width = GeoPoint.DistanceOverSurface(leftGeoPoint, rightGeoPoint);

            time = pointKA.Time;
            // leftControlPoint = leftCartPoint - KACartPoint;
            // rightControlPoint = rightCartPoint - KACartPoint;

            //leftControlPoint = getControlPoint(KACartPoint, leftCartPoint);
            //rightControlPoint = getControlPoint(KACartPoint, rightCartPoint);
            knowLeftConrol = false; 
            knowRightConrol = false; 
            knowViewPolygon = false;  
        }
         
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
      
        private Vector3D getControlPoint(Vector3D KAPoint, Vector3D sidePoint)
        {
            Vector3D rotAx = Vector3D.CrossProduct(KAPoint, sidePoint);
            RotateTransform3D leftTransform = new RotateTransform3D(new AxisAngleRotation3D(rotAx, 90));
            Vector3D controlPoint = leftTransform.Transform(KAPoint);
            double lenth = Vector3D.DotProduct(sidePoint, controlPoint) / controlPoint.Length;
            controlPoint.Normalize();
            controlPoint = controlPoint * lenth;
            return controlPoint;
        }

        public Vector3D LeftControlPoint
        {
            get
            {
                if (!knowLeftConrol) 
                {                   
                    leftControlPoint = getControlPoint(CartKAPoint, leftCartPoint);
                    knowLeftConrol = true;
                }
                return leftControlPoint;
            }
        }

        public Vector3D RightControlPoint
        {
            get
            {
                if (!knowRightConrol) 
                {                    
                    rightControlPoint = getControlPoint(CartKAPoint, rightCartPoint);
                    knowRightConrol = true;                    
                }
                return rightControlPoint;
            }
        }

        /// <summary>
        /// получить переднюю (по направлению скорости) левую точку полигона видимости
        /// </summary>
        public Vector3D TopLeftViewPoint
        {
            get
            { 
                if (!knowViewPolygon)
                {
                    calculateViewPoints();                    
                    knowViewPolygon = true;
                }
                return topLeftViewPoint;
            }
        }

        private void calculateViewPoints()
        {
            Vector3D dirVector = LanePos.getDirectionVector(trajPoint, rollAngle, pitchAngle);
            Polygon viewPol = Routines.getViewPolygon(trajPoint, dirVector, OptimalChain.Constants.camera_angle, pointsNum: 1);

            topLeftViewPoint = viewPol.Vertices[0];
            topRightViewPoint = viewPol.Vertices[1];
            botRightViewPoint = viewPol.Vertices[2];
            botLeftViewPoint = viewPol.Vertices[3];
        }

        public TrajectoryPoint TrajPoint{ get{ return trajPoint; } }

        public DateTime Time{ get{ return time; } }

        public Vector3D LeftCartPoint{ get{ return leftCartPoint; } }

        public Vector3D RightCartPoint{ get{ return rightCartPoint; } }

        public Vector3D CartKAPoint{ get{ return cartKAPoint; } }

        public GeoPoint LeftGeoPoint{ get{ return leftGeoPoint; } }

        public GeoPoint RightGeoPoint{ get{ return rightGeoPoint; } }

        public GeoPoint GeoKAPoint{ get{  return geoKAPoint; } }

        /// <summary>
        /// Return distance from gpoint to line [leftGeoPoint - rightGeoPoint] in radians
        /// </summary>
        /// <param name="gpoint"></param>
        /// <returns> distance in  in radians</returns>
        public double getDistToPoint(GeoPoint gpoint)
        {
            if (gpoint == leftGeoPoint || gpoint == rightGeoPoint || gpoint == GeoKAPoint)
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

        /// <summary>
        /// Подсчёт коэффициентов полиномов для коридоров.
        /// </summary>
        /// <param name="fetcher"></param>
        /// <param name="start"></param>
        /// <param name="az">Азимут [рад]</param>
        /// <param name="dist">Длина [мъ</param>
        /// <param name="roll">Крен [рад]</param>
        /// <param name="pitch">Тангаж [рад]</param>
        /// <param name="B1"></param>
        /// <param name="B2"></param>
        /// <param name="L1"></param>
        /// <param name="L2"></param>
        /// <param name="S1"></param>
        /// <param name="S2"></param>
        /// <param name="S3"></param>
        /// <param name="duration">Длительность [с]</param>
        public static void GetCoridorParams(DBTables.DataFetcher fetcher, DateTime start, double az, double dist, double roll, double pitch, out double B1, out double B2, out double L1, out double L2, out double S1, out double S2, out double S3, out double duration)
        {
            dist = Math.Min(dist, 97e3);
            Trajectory traj = fetcher.GetTrajectorySat(start, start.AddMinutes(1)); // должно хватить на коридор
            GeoPoint startPoint = Routines.IntersectOpticalAxisAndEarth(traj[0], roll, pitch);

            getGeodesicLine(startPoint, az, dist, out B1, out B2, out L1, out L2);
            getDistanceCoef(traj, dist, roll, pitch, B1, B2, L1, L2, out S1, out S2, out S3, out duration);
        }

        private static void getDistanceCoef(Trajectory traj, double dist, double roll, double pitch, double b1, double b2, double l1, double l2, out double s1, out double s2, out double s3, out double duration)
        {
            double WD = OptimalChain.RouteParams.WD(roll, pitch);
            List<double> dists = new List<double> { 0 };
            double dt = 1e-1;

            TrajectoryPoint curTP = traj[0];
            GeoPoint curP = Routines.IntersectOpticalAxisAndEarth(curTP, roll, pitch);
            double startLat = AstronomyMath.ToRad(curP.Latitude);
            double startLon = AstronomyMath.ToRad(curP.Longitude);

            while (dists[dists.Count - 1] < dist)
            {
                double s = dists[dists.Count - 1];
                double D = 1e3*(curTP.Position - GeoPoint.ToCartesian(curP, Astronomy.Constants.EarthRadius)).ToVector().Length;
                double W = WD * D;
                s += W * dt;
                dists.Add(s);

                double B = startLat + b1 * s + b2 * s * s;
                double L = startLon + l1 * s + l2 * s * s;
                curP = new GeoPoint(AstronomyMath.ToDegrees(B), AstronomyMath.ToDegrees(L));
                curTP = traj.GetPoint(curTP.Time.AddSeconds(dt));
            }

            int i1 = 1, i2 = dists.Count / 2, i3 = dists.Count - 1;
            double t1 = dt * i1;
            double t2 = dt * i2;
            double t3 = dt * i3;

            Matrix A = new Matrix(new double[][]
                {
                    new double[] {t1, t1*t1, t1*t1*t1},
                    new double[] {t2, t2*t2, t2*t2*t2},
                    new double[] {t3, t3*t3, t3*t3*t3},
                });
            Vector rhs = new Vector(dists[i1], dists[i2], dists[i3]);
            Vector S = Gauss.Solve(A, rhs);
            s1 = S[0];
            s2 = S[1];
            s3 = S[2];
            duration = t3;
        }

        /// <summary>
        /// Возвращает параметры, определяющие геодезическую, по её концам
        /// </summary>
        /// <param name="p">Начальная точка</param>
        /// <param name="pEnd">Конечная точка</param>
        /// <param name="B1"></param>
        /// <param name="B2"></param>
        /// <param name="L1"></param>
        /// <param name="L2"></param>
        public static void getGeodesicLineEndPoints(GeoPoint p, GeoPoint pEnd, 
                                                    out double B1, out double B2, out double L1, out double L2)
        {
            //Vector3D v0 = GeoPoint.ToCartesian(p, 1);
            //Vector3D v00 = GeoPoint.ToCartesian(pEnd, 1);
            //Arc geodesic = new Arc(v0, v00);
            //Arc meridian = new Arc(v0, new Vector3D(0, 0, 1));
            //double az = Math.Asin(Vector3D.DotProduct(v0, Vector3D.CrossProduct(meridian.TangentA, geodesic.TangentA)));
            //double dist = geodesic.CentralAngle * Constants.EarthRadius * 1e3;

            double dB = AstronomyMath.ToRad(pEnd.Latitude - p.Latitude);
            double dL = AstronomyMath.ToRad(pEnd.Longitude - p.Longitude);
            double midB = AstronomyMath.ToRad(pEnd.Latitude + p.Latitude) / 2;
            double mSin = Math.Sin(midB), mCos = Math.Cos(midB);

            double P = dB * (1 - dL * dL / 12 - (dL * mSin) * (dL * mSin) / 24);
            double Q = dL * mCos * (1 + dB * dB / 12 - (dL * mSin) * (dL * mSin) / 24);
            double dist = Math.Sqrt(P * P + Q * Q) * Astronomy.Constants.EarthRadius * 1e3;
            double mA = Math.Atan2(P, Q);
            double dA = dL * mSin * (1 + (dB * dB + dL * dL) / 12 - (dL * mSin) * (dL * mSin) / 24);
            double az = mA - dA / 2;

            getGeodesicLine(p, az, dist, out B1, out B2, out L1, out L2);
        }

        /// <summary>
        /// Возвращает параметры, определяющие геодезическую, по начальной точке и азимуту
        /// </summary>
        /// <param name="p">Начальная точка геодезической</param>
        /// <param name="az">Начальный азимут [рад]</param>
        /// <param name="dist">Длина вдоль геодезической [м]</param>
        /// <param name="B1"></param>
        /// <param name="B2"></param>
        /// <param name="L1"></param>
        /// <param name="L2"></param>
        public static void getGeodesicLine(GeoPoint p, double az, double dist,
                                           out double B1, out double B2, out double L1, out double L2
            /*bool sphericalEarth = true*/)
        {
            Vector Bm, Lm;

            //if (sphericalEarth)
            //{
            //    Vector3D v = GeoPoint.ToCartesian(p, 1);
            //    Arc meridian = new Arc(v, new Vector3D(0, 0, 1));

            //    RotateTransform3D azimuthTransform = new RotateTransform3D(new AxisAngleRotation3D(v, AstronomyMath.ToDegrees(-az)));
            //    Vector3D tangent = azimuthTransform.Transform(meridian.TangentA);

            //    Vector3D geodesicAxis = Vector3D.CrossProduct(v, tangent);
            //    double geodesicAngle = dist / (Constants.EarthRadius * 1e3);

            //    RotateTransform3D geodesicTransform = new RotateTransform3D(new AxisAngleRotation3D(geodesicAxis, AstronomyMath.ToDegrees(geodesicAngle)));
            //    Vector3D vEnd = geodesicTransform.Transform(v);
            //    GeoPoint pEnd = GeoPoint.FromCartesian(vEnd);

            //    Arc geodesic = new Arc(v, vEnd);
            //    GeoPoint pMid = geodesic.Middle();

            //    Bm = new Vector(AstronomyMath.ToRad(pMid.Latitude - p.Latitude), AstronomyMath.ToRad(pEnd.Latitude - p.Latitude));
            //    Lm = new Vector(AstronomyMath.ToRad(pMid.Longitude - p.Longitude), AstronomyMath.ToRad(pEnd.Longitude - p.Longitude));
            //}
            //else
            //{
            var arr = Ode.RK547M(0,
             new Vector(AstronomyMath.ToRad(p.Latitude), AstronomyMath.ToRad(p.Longitude), az),
             (t, x) => new Vector(
                 Math.Cos(x[2]) / Astronomy.Constants.EarthRadius / 1e3,
                 Math.Sin(x[2]) / Math.Cos(x[0]) / Astronomy.Constants.EarthRadius / 1e3,
                 Math.Sin(x[2]) / Astronomy.Constants.EarthRadius / 1e3 * Math.Tan(x[0])),
             new Options { RelativeTolerance = 1e-3 }).SolveFromToStep(0.0, dist, 0.5 * dist).ToArray();

            var p0 = arr[0].X;
            var s0 = arr[0].T;

            var p1 = arr[1].X;
            var s1 = arr[1].T;

            var p2 = arr[2].X;
            var s2 = arr[2].T;

            Bm = new Vector(p1[0] - p0[0], p2[0] - p0[0]);
            Lm = new Vector(p1[1] - p0[1], p2[1] - p0[1]);

            Matrix A = new Matrix(new double[][] { new double[] { 0.5 * dist, 0.25 * dist * dist }, new double[] { dist, dist * dist } });
            Vector B = Gauss.Solve(A, Bm);
            B1 = B[0];
            B2 = B[1];
            Vector L = Gauss.Solve(A, Lm);
            L1 = L[0];
            L2 = L[1];
        }


        public static double getSunHeight(DBTables.DataFetcher fetcher, GeoPoint p, DateTime time)
        {
            var sundata = fetcher.GetDataBeforeEqualDate(DBTables.SunTable.Name, DBTables.SunTable.Time, time, 1);
            if (sundata.Length == 0)
                throw new ArgumentException(String.Format("No sun data for time {0}", time));
            Vector3D sun = DBTables.SunTable.GetPositionUnitEarth(sundata[0]);
            Vector3D v = GeoPoint.ToCartesian(p, 1);
            Vector3D pToSun = sun - v;
            pToSun.Normalize();

            return AstronomyMath.ToRad(Vector3D.AngleBetween(pToSun, -v) - 90);
        }
    }

    public struct PolygonLit
    {
        public Polygon Polygon { get; set; }
        public bool Lit { get; set; }
    }
}

