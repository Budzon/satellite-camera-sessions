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
    [Obsolete("удалить")]
    public class SatTrajectory  /// удалить
    {
        public static Vector3D getInterpolPoint(Vector3D firstPoint, Vector3D secondPoint, DateTime dt1, DateTime dt2, DateTime targetDt)
        {
            double fullDist = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(firstPoint), GeoPoint.FromCartesian(secondPoint));
            double fullTimeDiff = (dt2 - dt1).TotalMilliseconds;
            double timePart = (targetDt - dt1).TotalMilliseconds;
            double patrDist = fullDist * timePart / fullTimeDiff;

            Vector3D rotAxis = Vector3D.CrossProduct(firstPoint, secondPoint);
            RotateTransform3D rotTransform = new RotateTransform3D(new AxisAngleRotation3D(rotAxis, AstronomyMath.ToDegrees(patrDist)));
            Vector3D newMiddlePoint = rotTransform.Transform(firstPoint);

            return newMiddlePoint;
        }
    }
    /*
    public class SatTrajectory : Astronomy.Trajectory
    {
        //public List<TrajectoryPoint> points { get; set; }       

        /// <summary>
        /// Get shooting lane by trajectory and max roll angle  
        /// </summary>
        /// <param name="minAngle">min roll angle in radians.</param>
        /// <param name="readStep">step in points array</param>
        /// /// <param name="maxAngle">max roll angle in radians.</param>          
        public SatLane getCaptureLane(double rollAngle, double viewAngle, int readStep = 1, int polygonStep = 15)
        {
            double minAngle = rollAngle - viewAngle / 2;
            double maxAngle = rollAngle + viewAngle / 2;
             
            List<LanePos> lanePoints = new List<LanePos>();

            for (int p_ind = 0; p_ind < points.Count; p_ind += readStep)
            {

                ////
               //  Vector3D nadirPoint = SatTrajectory.SphereVectIntersect(-points[p_ind].Position.ToVector(), points[p_ind].Position, Astronomy.Constants.EarthRadius);
               //  Vector3D rollPithPoint = SatTrajectory.SphereVectIntersect(-points[p_ind + 1].Position.ToVector(), points[p_ind+1].Position, Astronomy.Constants.EarthRadius);
               //  double distt = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(nadirPoint), GeoPoint.FromCartesian(rollPithPoint));
               //  Console.WriteLine("speed = {0}", distt / 10);
                /////


                Vector3D eDirVect = new Vector3D(-points[p_ind].Position.X, -points[p_ind].Position.Y, -points[p_ind].Position.Z);

                RotateTransform3D leftTransform = new RotateTransform3D(new AxisAngleRotation3D(points[p_ind].Velocity, AstronomyMath.ToDegrees(minAngle)));
                RotateTransform3D rightTransform = new RotateTransform3D(new AxisAngleRotation3D(points[p_ind].Velocity, AstronomyMath.ToDegrees(maxAngle)));
 
                Vector3D leftVector = leftTransform.Transform(eDirVect);
                Vector3D rightVector = rightTransform.Transform(eDirVect);
                                                 
                Vector3D leftCrossPoint = SphereVectIntersect(leftVector, points[p_ind].Position, Astronomy.Constants.EarthRadius);
                Vector3D rightCrossPoint = SphereVectIntersect(rightVector, points[p_ind].Position, Astronomy.Constants.EarthRadius);
                Vector3D middlePoint = points[p_ind].Position.ToVector();

                               
                var tanHalfVa = Math.Tan(viewAngle / 2);
                var angle_rad = Math.Atan(tanHalfVa / (Math.Sqrt(tanHalfVa * tanHalfVa + 1)));
                var angle_degr = AstronomyMath.ToDegrees(angle_rad);


                Vector3D rotAxis = Vector3D.CrossProduct(rightVector, points[p_ind].Velocity);

                RotateTransform3D rightBotTransfrom = new RotateTransform3D(new AxisAngleRotation3D(rotAxis, angle_degr));
                Vector3D rightBotCrossVector = rightBotTransfrom.Transform(rightVector);
                Vector3D rightBotCrossPoint = SatTrajectory.SphereVectIntersect(rightBotCrossVector, points[p_ind].Position, Astronomy.Constants.EarthRadius);

                //RotateTransform3D rightTopTransfrom = new RotateTransform3D(new AxisAngleRotation3D(rotAxis, -angle_degr));
                //Vector3D rightTopCrossVector = rightTopTransfrom.Transform(rightVector);
                // Vector3D rightTopCrossPoint = SatTrajectory.SphereVectIntersect(rightTopCrossVector, points[p_ind].Position, Astronomy.Constants.EarthRadius);
                               
                rotAxis = Vector3D.CrossProduct(leftVector, points[p_ind].Velocity);

                RotateTransform3D leftBotTransfrom = new RotateTransform3D(new AxisAngleRotation3D(rotAxis, -angle_degr));
                Vector3D leftBotCrossVector = leftBotTransfrom.Transform(leftVector);
                Vector3D leftBotCrossPoint = SatTrajectory.SphereVectIntersect(leftBotCrossVector, points[p_ind].Position, Astronomy.Constants.EarthRadius);

                // RotateTransform3D leftTopTransfrom = new RotateTransform3D(new AxisAngleRotation3D(rotAxis, angle_degr));
                // Vector3D leftTopCrossVector = leftTopTransfrom.Transform(leftVector);
                // Vector3D leftTopCrossPoint = SatTrajectory.SphereVectIntersect(leftTopCrossVector, points[p_ind].Position, Astronomy.Constants.EarthRadius);

                double distRight = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(rightBotCrossPoint), GeoPoint.FromCartesian(rightCrossPoint));
                double distLeft = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(leftBotCrossPoint), GeoPoint.FromCartesian(leftCrossPoint));
                //Console.WriteLine("a/2 = {0}, b/2 = {1},     |b-a| / 2 = {2} ",distRight* Astronomy.Constants.EarthRadius, distLeft* Astronomy.Constants.EarthRadius, Math.Abs(distRight - distLeft) * Astronomy.Constants.EarthRadius);

                double dist = Math.Abs(distRight - distLeft);
                // dist идет в lanPos, либо как правая, либо как левая

                leftCrossPoint.Normalize();
                rightCrossPoint.Normalize();
                middlePoint.Normalize();

                LanePos newLanePos = new LanePos(leftCrossPoint, rightCrossPoint, points[p_ind]);
                lanePoints.Add(newLanePos);
            }

            return new SatLane(minAngle, maxAngle, lanePoints, polygonStep);
        }
         

        /// <summary>
        /// just get crossings point of line (by point and vector) and sphere (radius R with the center (0,0,0) )
        /// </summary>
        /// <param name="vect"> directing vector</param>
        /// <param name="point"> initial point </param>
        /// <param name="R"> sphere radius</param>
        /// <returns></returns>
        public static Vector3D SphereVectIntersect(Vector3D vect, Point3D point, double R)
        {
            Vector3D dilatedPoint = new Vector3D(point.X/R, point.Y/R, point.Z/R);
            List<Vector3D> intersection = Routines.IntersectLineUnitSphere(dilatedPoint, vect);

            /// Possible optimization:
            /// If point is outside of the sphere and vect is directed towards it, then the answer is always intersection[0].
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
            double fullTimeDiff = (dt2 - dt1).TotalMilliseconds;
            double timePart = (targetDt - dt1).TotalMilliseconds;
            double patrDist = fullDist * timePart / fullTimeDiff;

            Vector3D rotAxis = Vector3D.CrossProduct(firstPoint, secondPoint);
            RotateTransform3D rotTransform = new RotateTransform3D(new AxisAngleRotation3D(rotAxis, AstronomyMath.ToDegrees(patrDist)));
            Vector3D newMiddlePoint = rotTransform.Transform(firstPoint);

            return newMiddlePoint;
        }
    }
    
     */
   
    
    
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

        public SatLane(Astronomy.Trajectory _trajectory, double _rollAngle, double _viewAngle, int readStep = 1, int polygonStep = 15)
        {
            trajectory = _trajectory;
            rollAngle = _rollAngle;
            viewAngle = _viewAngle;

            double minAngle = rollAngle - viewAngle / 2;
            double maxAngle = rollAngle + viewAngle / 2;

            List<LanePos> lanePoints = new List<LanePos>();
            var points = trajectory.Points;
            var count = trajectory.Count;
            for (int p_ind = 0; p_ind < count; p_ind += readStep)
            {
                //LanePos(TrajectoryPoint pointKA, double viewAngle, double rollAngle)
                LanePos newLanePos = new LanePos(points[p_ind], viewAngle, rollAngle);
                lanePoints.Add(newLanePos);
            }

            createPolygons(minAngle, maxAngle, lanePoints, polygonStep);
        }



        private void createPolygons(double minAngle, double maxAngle, List<LanePos> points, int polygonStep)
        {
            Sectors = new List<LaneSector>();
            //lanePoints = points;

            List<Vector3D> leftLanePoints = new List<Vector3D>();
            List<Vector3D> rightLanePoints = new List<Vector3D>();

            List<Vector3D> leftControlPoints = new List<Vector3D>();
            List<Vector3D> rightControlPoints = new List<Vector3D>();

            if (points.Count == 0)
                return;

            double width = (points[0].LeftCartPoint - points[0].RightCartPoint).Length / 2;

            DateTime sectorFromDT = points[0].Time;
            DateTime sectorToDT;
            List<LanePos> sectorPoints = new List<LanePos>();

            GeoPoint prevPoint = AstronomyMath.GreenwichToSpherical(points[0].LeftCartPoint.ToPoint());

            for (int p_ind = 0; p_ind < points.Count; p_ind++)
            {
                sectorPoints.Add(points[p_ind]);

                if (p_ind == 0)
                {
                    leftLanePoints.Add(points[p_ind].LeftCartPoint);
                    rightLanePoints.Add(points[p_ind].RightCartPoint);

                    leftControlPoints.Add(points[p_ind].leftControlPoint);
                    rightControlPoints.Add(points[p_ind].rightControlPoint);
                    continue;
                }

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

                if (p_ind % polygonStep == 0 || needNewSector)
                {
                    leftLanePoints.Add(points[p_ind].LeftCartPoint);
                    rightLanePoints.Add(points[p_ind].RightCartPoint);

                    leftControlPoints.Add(points[p_ind].leftControlPoint);
                    rightControlPoints.Add(points[p_ind].rightControlPoint);
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
                    newSector.sectorPoints = new List<LanePos>(sectorPoints);
                    Sectors.Add(newSector);

                    sectorPoints.Clear();

                    rightLanePoints.Clear();
                    leftLanePoints.Clear();

                    leftControlPoints.Clear();
                    rightControlPoints.Clear();

                    sectorPoints.Add(points[p_ind]);

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

            bool needStop = false;
            int i = 0;
            foreach (var sector in Sectors)
            {
                var sectorPoints = sector.sectorPoints;
                for (; i < sectorPoints.Count; i++)
                {
                    if (begTime <= sectorPoints[i].Time && sectorPoints[i].Time <= endTime)
                    {
                        polygonPoints.Add(sectorPoints[i].LeftCartPoint);
                        rightPolygonPoints.Add(sectorPoints[i].RightCartPoint);
                    }

                    if (sectorPoints.Count - 1 == i)
                        break;

                    if (sectorPoints[i + 1].Time < begTime)
                        continue;

                    if (sectorPoints[i].Time < begTime && begTime < sectorPoints[i + 1].Time)
                    {
                        var pos = getlanePosByTime(sectorPoints[i], sectorPoints[i + 1], begTime);
                        polygonPoints.Add(pos.Item1);
                        rightPolygonPoints.Add(pos.Item2);
                    }

                    if (sectorPoints[i].Time < endTime && endTime < sectorPoints[i + 1].Time)
                    {
                        var pos = getlanePosByTime(sectorPoints[i], sectorPoints[i + 1], endTime);
                        polygonPoints.Add(pos.Item1);
                        rightPolygonPoints.Add(pos.Item2);
                    }

                    if (sectorPoints[i + 1].Time > endTime)
                    {
                        needStop = true;
                        break;
                    }
                }
                i = 1; // для всех секторов кроме первого начинаем со второго элемента
                if (needStop)
                    break;
                // @todo полигон может быть длиннее 180 (upd - по исходным данным вроде не должен), что вызовет ошибки. Надо разрезать на полигоны или вставить проверку. 
            }

            for (int ind = rightPolygonPoints.Count - 1; ind >= 0; ind--)
                polygonPoints.Add(rightPolygonPoints[ind]);

            TrajectoryPoint pointFrom = new TrajectoryPoint(begTime,  trajectory.GetPosition(begTime),  trajectory.GetVelocity(begTime));
            TrajectoryPoint pointTo = new TrajectoryPoint(endTime,  trajectory.GetPosition(endTime),  trajectory.GetVelocity(endTime));
            Polygon segmenPol = new Polygon(polygonPoints, new Vector3D(0, 0, 0));
            return  Tuple.Create(segmenPol, pointFrom, pointTo);
        }



        private Tuple<Vector3D, Vector3D> getlanePosByTime(LanePos first, LanePos second, DateTime time)
        {
            Vector3D newMiddlePoint = SatTrajectory.getInterpolPoint(first.MiddleCartPoint, second.MiddleCartPoint, first.Time, second.Time, time);

            Vector3D newLeft = newMiddlePoint + (first.LeftCartPoint - first.MiddleCartPoint);
            Vector3D newRight = newMiddlePoint + (first.RightCartPoint - first.MiddleCartPoint);

            return new Tuple<Vector3D, Vector3D>(newLeft, newRight);
        }

        private LanePos interpolatelanePosByTime(DateTime time)
        {
            Point3D pos = trajectory.GetPosition(time);
            Vector3D velo = trajectory.GetVelocity(time);
            TrajectoryPoint trajPoint = new TrajectoryPoint(time, pos, velo);
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
        public Vector3D leftControlPoint;
        public Vector3D rightControlPoint;

        private GeoPoint leftGeoPoint;
        private GeoPoint rightGeoPoint;
        private GeoPoint middleGeoPoint;
        private TrajectoryPoint trajPoint;

        private double width;

        private RollOrientation roll;

        public static Vector3D getDirectionVector(TrajectoryPoint point, double rollAngle, double pitchAngle)
        {
            Vector3D position = point.Position.ToVector();
            Vector3D velo = point.Velocity;
            Vector3D eDirVect = new Vector3D(-position.X, -position.Y, -position.Z);  

            Vector3D rollAxis = Vector3D.CrossProduct(velo, eDirVect);
            RotateTransform3D rollTransfrom = new RotateTransform3D(new AxisAngleRotation3D(velo, AstronomyMath.ToDegrees(rollAngle)));

            Vector3D pitchAxis = Vector3D.CrossProduct(velo, eDirVect);
            RotateTransform3D pitchTransfrom = new RotateTransform3D(new AxisAngleRotation3D(pitchAxis, AstronomyMath.ToDegrees(pitchAngle)));

            Vector3D dirVector = rollTransfrom.Transform(pitchTransfrom.Transform(eDirVect));
            return dirVector;
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

            var tanHalfVa = Math.Tan(viewAngle / 2);
            var angle_rad = Math.Atan(tanHalfVa / (Math.Sqrt(tanHalfVa * tanHalfVa + 1)));
            var angle_degr = AstronomyMath.ToDegrees(angle_rad);

            Vector3D rotAxis = Vector3D.CrossProduct(rightVector, pointKA.Velocity);

            RotateTransform3D rightBotTransfrom = new RotateTransform3D(new AxisAngleRotation3D(rotAxis, angle_degr));
            Vector3D rightBotCrossVector = rightBotTransfrom.Transform(rightVector);
            Vector3D rightBotCrossPoint = Routines.SphereVectIntersect(rightBotCrossVector, pointKA.Position, Astronomy.Constants.EarthRadius);

            rotAxis = Vector3D.CrossProduct(leftVector, pointKA.Velocity);

            RotateTransform3D leftBotTransfrom = new RotateTransform3D(new AxisAngleRotation3D(rotAxis, -angle_degr));
            Vector3D leftBotCrossVector = leftBotTransfrom.Transform(leftVector);
            Vector3D leftBotCrossPoint = Routines.SphereVectIntersect(leftBotCrossVector, pointKA.Position, Astronomy.Constants.EarthRadius);

            double distRight = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(rightBotCrossPoint), GeoPoint.FromCartesian(rightCrossPoint));
            double distLeft = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(leftBotCrossPoint), GeoPoint.FromCartesian(leftCrossPoint));

            double dist = Math.Abs(distRight - distLeft);
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

            // leftControlPoint = leftCartPoint - middleCartPoint;
            // rightControlPoint = rightCartPoint - middleCartPoint;

            leftControlPoint = getControlPoint(middleCartPoint, leftCartPoint);
            rightControlPoint = getControlPoint(middleCartPoint, rightCartPoint);
        }

        //public LanePos(Vector3D _leftPoint, Vector3D _rightPoint, TrajectoryPoint _trajPoint)
        //{
        //    trajPoint = _trajPoint;
        //    leftCartPoint = _leftPoint;
        //    rightCartPoint = _rightPoint;
        //    middleCartPoint = _trajPoint.Position.ToVector();
        //    middleCartPoint.Normalize();
        //    middleGeoPoint = GeoPoint.FromCartesian(middleCartPoint); 

        //    leftGeoPoint = GeoPoint.FromCartesian(leftCartPoint);
        //    rightGeoPoint = GeoPoint.FromCartesian(rightCartPoint);
        //    width = GeoPoint.DistanceOverSurface(leftGeoPoint, rightGeoPoint);

        //    // leftControlPoint = leftCartPoint - middleCartPoint;
        //    // rightControlPoint = rightCartPoint - middleCartPoint;

        //    leftControlPoint = getControlPoint(middleCartPoint, leftCartPoint);
        //    rightControlPoint = getControlPoint(middleCartPoint, rightCartPoint);

        //    //// неверно для полос захвата
        //    /*
        //    leftControlPoint = leftCartPoint - rightCartPoint;
        //    leftControlPoint.Normalize();
        //    leftControlPoint = leftControlPoint * (leftCartPoint - rightCartPoint).Length / 2;

        //    rightControlPoint = rightCartPoint - leftCartPoint; 
        //    rightControlPoint.Normalize();
        //    rightControlPoint = rightControlPoint * (rightCartPoint - leftCartPoint).Length / 2;
        //     */
        //}

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

        public TrajectoryPoint TrajPoint{ get{ return trajPoint; } }

        public DateTime Time{ get{ return TrajPoint.Time; } }

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



}

