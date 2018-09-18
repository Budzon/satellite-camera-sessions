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
using SessionsPlanning;
//using System.Data.SqlTypes;
using Microsoft.SqlServer.Types;


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

        public DateTime FromDt
        {
            get
            {
                return trajectory[0].Time;
            }
        }
        public DateTime ToDt
        {
            get
            { 
                return trajectory.Last().Time;
            }
        }

        /// <summary>
        ///  Функция рассчитывает полигон видимости для полосы с ненулевым тангажом и креном
        /// </summary>
        /// <param name="trajectory"></param>
        /// <param name="rollAngle"></param>
        /// <param name="pitchAngle"></param>
        /// <param name="viewAngle"></param>
        /// <returns> полигон видимости </returns>
        public static Polygon getRollPitchLanePolygon(Astronomy.Trajectory trajectory, double rollAngle, double pitchAngle)
        {
            int count = trajectory.Count;

            if (count <= 1)
            {
                throw new ArgumentException("Not enough points to create an overview polygon");
            }
            // количество точек будущего полигона - две точки с каждого полигона видимости
            int vertNum = count * 2 + 2;
            Vector3D[] points = new Vector3D[vertNum];

            // первый кадр
            SatelliteCoordinates firstKaPos = new SatelliteCoordinates(trajectory.Points[0]);
            firstKaPos.addRollPitchRot(rollAngle, pitchAngle);
            if (Math.Sign(rollAngle) == Math.Sign(pitchAngle))
            {
                points[0] = firstKaPos.BotRightViewPoint;
                points[vertNum - 1] = firstKaPos.BotLeftViewPoint;
                points[1] = firstKaPos.TopRightViewPoint;
            }
            else
            {
                points[0] = firstKaPos.BotRightViewPoint;
                points[vertNum - 1] = firstKaPos.BotLeftViewPoint;
                points[vertNum - 2] = firstKaPos.TopLeftViewPoint;
            }

            for (int i = 1; i < count - 1; i++)
            {
                SatelliteCoordinates curKaPos = new SatelliteCoordinates(trajectory.Points[i]);
                curKaPos.addRollPitchRot(rollAngle, pitchAngle);

                if (Math.Sign(rollAngle) == Math.Sign(pitchAngle))
                {
                    points[i + 1] = curKaPos.TopRightViewPoint;
                    points[vertNum - 1 - i] = curKaPos.BotLeftViewPoint;
                }
                else
                {
                    points[i] = curKaPos.BotRightViewPoint;
                    points[vertNum - 2 - i] = curKaPos.TopLeftViewPoint;
                }
            }
            SatelliteCoordinates lastKaPos = new SatelliteCoordinates(trajectory.Points.Last());
            lastKaPos.addRollPitchRot(rollAngle, pitchAngle);
            if (Math.Sign(rollAngle) == Math.Sign(pitchAngle))
            {
                points[count] = lastKaPos.TopRightViewPoint;
                points[count + 1] = lastKaPos.TopLeftViewPoint;
                points[count + 2] = lastKaPos.BotLeftViewPoint;
            }
            else
            {
                points[count - 1] = lastKaPos.BotRightViewPoint;
                points[count] = lastKaPos.TopRightViewPoint;
                points[count + 1] = lastKaPos.TopLeftViewPoint;
            }

            return new Polygon(points.ToList());
        }

       
        public static List<LanePos> GetViewLane(Trajectory trajectory, TimePeriod period)
        {
            Trajectory traj = trajectory.getSubTrajectory(period.dateFrom, period.dateTo);
            double angle = 2 * OptimalChain.Constants.max_roll_angle + OptimalChain.Constants.camera_angle;
            List<LanePos> res = traj.Points.Select(p => new LanePos(p, angle, 0)).ToList();
            return res;
        }

        public SatLane(Astronomy.Trajectory _trajectory, double _rollAngle, double _viewAngle)
        {
            if (_trajectory.Count <= 1)
            {
                throw new ArgumentException("Not enough points to create an overview lane");
            }

            trajectory = _trajectory;
            rollAngle = _rollAngle;
            viewAngle = _viewAngle;

            double minAngle = rollAngle - viewAngle / 2;
            double maxAngle = rollAngle + viewAngle / 2;

            createPolygons(minAngle, maxAngle, OptimalChain.Constants.stripStepPassing);
        }


        private void createPolygons(double minAngle, double maxAngle, int polygonStep)
        {
            Sectors = new List<LaneSector>();

            List<Vector3D> leftLanePoints = new List<Vector3D>();
            List<Vector3D> rightLanePoints = new List<Vector3D>();

            if (trajectory.Count == 0)
                return;

            TrajectoryPoint[] points = trajectory.Points;
            int points_count = trajectory.Count;

            LanePos firstPos = new LanePos(points[0], viewAngle, rollAngle);

            DateTime sectorFromDT = firstPos.Time;
            DateTime sectorToDT;
            List<LanePos> sectorPoints = new List<LanePos>();

            GeoPoint prevPoint = AstronomyMath.GreenwichToSpherical(firstPos.LeftCartPoint.ToPoint());

            for (int p_ind = 0; p_ind < points_count; p_ind++)
            {
                LanePos pos = new LanePos(points[p_ind], viewAngle, rollAngle);
                sectorPoints.Add(pos);

                if (p_ind == 0) // первая точка полосы
                {
                    SatelliteCoordinates firstKaPos = new SatelliteCoordinates(trajectory.Points[p_ind]);
                    firstKaPos.addRollRot(rollAngle);
                    leftLanePoints.Add(firstKaPos.BotLeftViewPoint);
                    rightLanePoints.Add(firstKaPos.BotRightViewPoint);

                    leftLanePoints.Add(pos.LeftCartPoint);
                    rightLanePoints.Add(pos.RightCartPoint);
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
                    double curDist = GeoPoint.DistanceOverSurface(prevPoint, point);
                    needNewSector = curDist > AstronomyMath.ToRad(90); // @todo плохой способ. Правильнее будет, если сделать 180, но так не работает, разобраться почему?
                }

                if (p_ind % polygonStep == 0 || needNewSector)
                {
                    leftLanePoints.Add(pos.LeftCartPoint);
                    rightLanePoints.Add(pos.RightCartPoint);

                    if (p_ind == points_count - 1) // если точка последняя
                    {
                        SatelliteCoordinates firstKaPos = new SatelliteCoordinates(trajectory.Points[p_ind]);
                        firstKaPos.addRollRot(rollAngle);
                        leftLanePoints.Add(firstKaPos.TopLeftViewPoint);
                        rightLanePoints.Add(firstKaPos.TopRightViewPoint);
                    }
                }

                if (needNewSector)
                {                  
                    sectorToDT = points[p_ind].Time;                     
                    //leftLanePoints.Reverse();
                    for (int i = leftLanePoints.Count - 1; i >= 0;i--)
                    {
                        rightLanePoints.Add(leftLanePoints[i]);
                    }
                                            
                    Polygon pol = new Polygon(rightLanePoints);
                    
                    LaneSector newSector = new LaneSector(pol, sectorPoints, sectorFromDT, sectorToDT);
                    Sectors.Add(newSector);

                    sectorPoints = new List<LanePos>();
                    rightLanePoints.Clear();
                    leftLanePoints.Clear(); 
                    sectorPoints.Add(pos);

                    leftLanePoints.Add(pos.LeftCartPoint);
                    rightLanePoints.Add(pos.RightCartPoint);      
               
                    sectorFromDT = sectorToDT;
                    prevPoint = point;                    
                }
            }
        }

        public List<CaptureConf> getCaptureConfs(RequestParams request)
        {
            List<CaptureConf> res = new List<CaptureConf>();

            foreach (var reqPol in request.polygons)
            {
                foreach (LaneSector sector in Sectors)
                {
                    List<Polygon> intersections = Polygon.Intersect(sector.polygon, reqPol);
                    foreach (var int_pol in intersections)
                    {
                        var verts = int_pol.Vertices;
                        DateTime tFrom = sector.getPointTime(verts[0]);
                        DateTime tTo = tFrom;
                        Vector3D pointFrom = verts[0];
                        Vector3D pointTo = verts[0];
                        bool outOfRange = false;
                        foreach (var point in verts)
                        {
                            DateTime curTime = sector.getPointTime(point);
                            if (curTime < request.timeFrom || request.timeTo < curTime)
                            {
                                outOfRange = true;
                                break;
                            }
                            else if (curTime < tFrom)
                            {
                                tFrom = curTime;
                                pointFrom = point;
                            }
                            else if (curTime > tTo)
                            {
                                tTo = curTime;
                                pointTo = point;
                            }
                        }

                        if (outOfRange)
                            break;

                        var vdefl = getViewDeflect(tFrom, pointFrom);
                        DateTime shootingFrom = tFrom.AddSeconds(vdefl);
                        DateTime shootingTo = tTo.AddSeconds(-getViewDeflect(tTo, pointTo));

                        if (shootingTo <= shootingFrom) // полоса конфигурации съемки короче кадра, делаем снимок по центру                        
                             shootingTo = shootingFrom = tFrom.AddSeconds((tTo - tFrom).TotalSeconds / 2);

                        double subsquare = int_pol.Area;
                        Order order = new Order(request, int_pol, subsquare / request.Square); 
                        
                        var orders = new List<Order>() { order };                        
                        CaptureConf newcc = new CaptureConf(shootingFrom, shootingTo, rollAngle, orders, WorkingType.Shooting, null);

                        res.Add(newcc);
                    }
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

            var lastSector = Sectors.Last();
            var lastPoint = lastSector.sectorPoints.Last();

            if (Sectors[0].sectorPoints[0].Time > begTime || lastPoint.Time < endTime)
                throw new System.ArgumentException("Incorrect time interval.");

            List<Vector3D> leftPolygonPoints = new List<Vector3D>();
            List<Vector3D> rightPolygonPoints = new List<Vector3D>();

            bool found_beg = false, found_end = false;
            foreach (var sector in Sectors)
            {
                var sectorPoints = sector.sectorPoints;

                int i = 0;
                if (leftPolygonPoints.Count > 0) // уже начался полигон
                    i = 1;

                for (; i < sectorPoints.Count; i++)
                {
                    if (sectorPoints[i].Time < begTime)
                        continue;

                    if (sectorPoints[i].Time > endTime)
                        break;

                    if (begTime < sectorPoints[i].Time && sectorPoints[i].Time < endTime)
                    {
                        leftPolygonPoints.Add(sectorPoints[i].LeftCartPoint);
                        rightPolygonPoints.Add(sectorPoints[i].RightCartPoint);
                        continue;
                    }

                    if (begTime == sectorPoints[i].Time && sectorPoints[i].Time == endTime)
                    {
                        leftPolygonPoints.Add(sectorPoints[i].LeftCartPoint);
                        rightPolygonPoints.Add(sectorPoints[i].RightCartPoint);

                        if (begTime == sectorPoints[i].Time)
                            found_beg = true;
                        if (endTime == sectorPoints[i].Time)
                            found_end = true;
                    }
                }
            }

            LanePos fisrt = Sectors[0].sectorPoints[0];
            LanePos last = Sectors.Last().sectorPoints.Last();

            if (!found_beg)
            {
                LanePos posFrom = interpolatelanePosByTime(begTime);
                leftPolygonPoints.Insert(0, posFrom.LeftCartPoint);
                rightPolygonPoints.Insert(0, posFrom.RightCartPoint);
                fisrt = posFrom;
            }

            if (!found_end)
            {
                LanePos posTo = interpolatelanePosByTime(endTime);
                leftPolygonPoints.Add(posTo.LeftCartPoint);
                rightPolygonPoints.Add(posTo.RightCartPoint);
                last = posTo;
            }

            // учёт трапецевидности

            leftPolygonPoints.Insert(0, fisrt.KaCoords.BotLeftViewPoint);
            rightPolygonPoints.Insert(0, fisrt.KaCoords.BotRightViewPoint);

            leftPolygonPoints.Add(last.KaCoords.TopLeftViewPoint);
            rightPolygonPoints.Add(last.KaCoords.TopRightViewPoint);
    

            for (int i = leftPolygonPoints.Count - 1; i >= 0; i--)            
                rightPolygonPoints.Add(leftPolygonPoints[i]);
            
            return new Polygon(rightPolygonPoints);
        }


        /// <summary>
        /// возвращает время в секундах, характеризующее расстояние между серединой кадра и его границей по линии точки target
        /// 
        ///         ________R________
        ///        /        |b       \
        ///       /_________|         \
        ///      / tDefl    |          \
        ///     /           |a          \
        ///    /____________|____________\ 
        ///                 L
        /// </summary>
        /// <param name="dtime">время кадра</param>
        /// <param name="target">снимаемая точка</param>
        /// <returns>упреждение в секунах</returns>
        private double getViewDeflect(DateTime dtime, Vector3D target)
        {
            TrajectoryPoint trajPoint = trajectory.GetPoint(dtime);
            LanePos curPos = new LanePos(trajPoint, viewAngle, rollAngle);
            double a = GeoPoint.DistanceOverSurface(curPos.LeftCartPoint, target);            
            double ab = GeoPoint.DistanceOverSurface(curPos.LeftGeoPoint, curPos.RightGeoPoint);
            double botTrapezBase = GeoPoint.DistanceOverSurface(curPos.KaCoords.BotLeftViewPoint, curPos.KaCoords.TopLeftViewPoint);
            double topTrapezBase = GeoPoint.DistanceOverSurface(curPos.KaCoords.BotRightViewPoint, curPos.KaCoords.TopRightViewPoint);
            double triangleBase = Math.Abs(topTrapezBase - botTrapezBase) / 2;
            double deflect = triangleBase * a / ab + Math.Min(botTrapezBase, topTrapezBase) / 2; // разница между серединой кадра и его границей (по линии снимаемой точки target) в радианах
            double velo = trajPoint.Velocity.Length / trajPoint.Position.ToVector().Length;
            double timeDeflect = deflect / velo;  // разница между серединой кадра и его границей (по линии снимаемой точки target) в секундах
            return timeDeflect;
        }


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
        public DateTime fromDT { get; private set; }
        public DateTime toDT { get; private set; }
        public Polygon polygon { get; private set; }
        public List<LanePos> sectorPoints { get; private set; }

        public LaneSector(Polygon _polygon, List<LanePos> _points, DateTime _fromDt, DateTime _toDt)
        {
            polygon = _polygon;
            sectorPoints = _points;
            fromDT = _fromDt;
            toDT = _toDt;
        }
         

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
            double fullDist = distToFirst + distToSecond;
            double fullTime = Math.Abs((sectorPoints[first].Time - sectorPoints[second].Time).TotalMilliseconds);
            double diffMiliSecs = fullTime * distToFirst / fullDist;

            int outside = distToSecond > fullDist ? 1 : -1;
            sign = (first < second) ? -outside : outside;

            return sectorPoints[first].Time.AddMilliseconds(sign * diffMiliSecs);
        }
    }

    /// <summary>
    /// класс, содержащий в себе положение в полосе в момент времени, то есть "правую" и "левую" точку и время этого положения
    /// </summary>
    public class LanePos
    {       
        public LanePos(Vector3D _leftPoint, Vector3D _kaPoint, Vector3D _rightPoint, DateTime _time)
        { 
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


        public LanePos(TrajectoryPoint pointKA, double viewAngle, double _rollAngle)
        {
            rollAngle = _rollAngle;

            double minAngle = rollAngle - viewAngle / 2;
            double maxAngle = rollAngle + viewAngle / 2;

            Vector3D leftVector = getDirectionVector(pointKA, minAngle, 0);
            Vector3D rightVector = getDirectionVector(pointKA, maxAngle, 0);

            Vector3D leftCrossPoint = Routines.SphereVectIntersect(leftVector, pointKA.Position, Astronomy.Constants.EarthRadius);
            Vector3D rightCrossPoint = Routines.SphereVectIntersect(rightVector, pointKA.Position, Astronomy.Constants.EarthRadius);
            cartKAPoint = pointKA.Position.ToVector();

            leftCrossPoint.Normalize();
            rightCrossPoint.Normalize();
            cartKAPoint.Normalize();

            KaCoords = new SatelliteCoordinates(pointKA);
            KaCoords.addRollRot(rollAngle);

            leftCartPoint = leftCrossPoint;
            rightCartPoint = rightCrossPoint; 
            CartKAPoint.Normalize();
            geoKAPoint = GeoPoint.FromCartesian(CartKAPoint);

            leftGeoPoint = GeoPoint.FromCartesian(leftCartPoint);
            rightGeoPoint = GeoPoint.FromCartesian(rightCartPoint);
            width = GeoPoint.DistanceOverSurface(leftGeoPoint, rightGeoPoint);

            time = pointKA.Time;
        }

        public DateTime Time { get { return time; } }

        public Vector3D LeftCartPoint { get { return leftCartPoint; } }

        public Vector3D RightCartPoint { get { return rightCartPoint; } }

        public Vector3D CartKAPoint { get { return cartKAPoint; } }

        public GeoPoint LeftGeoPoint { get { return leftGeoPoint; } }

        public GeoPoint RightGeoPoint { get { return rightGeoPoint; } }

        public GeoPoint GeoKAPoint { get { return geoKAPoint; } }

        public SatelliteCoordinates KaCoords { get; private set; }
 
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


        public static Vector3D getDirectionVector(TrajectoryPoint point, double rollAngle, double pitchAngle)
        {
            SatelliteCoordinates ka = new SatelliteCoordinates(point);
            ka.addRollRot(rollAngle);
            ka.addPitchRot(pitchAngle);
            return ka.ViewDir;
        }
        
        public static Vector3D getSurfacePoint(TrajectoryPoint point, double rollAngle, double pitchAngle)
        {
            Vector3D dirVector = getDirectionVector(point, rollAngle, pitchAngle);
            return Routines.SphereVectIntersect(dirVector, point.Position, Astronomy.Constants.EarthRadius);
        }

        #region Private part
        private Vector3D leftCartPoint;
        private Vector3D rightCartPoint;
        private Vector3D cartKAPoint;

        private GeoPoint leftGeoPoint;
        private GeoPoint rightGeoPoint;
        private GeoPoint geoKAPoint;
        private double rollAngle;

        private double width;
        private DateTime time;
        #endregion
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
        /*
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
        */

        public static Polygon FormSectorFromLanePoints(List<LanePos> turn, int from, int to)
        {
            List<GeoPoint> vertices = new List<GeoPoint>();
            for (int i = from; i <= to; i++)
                vertices.Add(GeoPoint.FromCartesian(turn[i].RightCartPoint));
            for (int i = to; i >= from; --i)
                vertices.Add(GeoPoint.FromCartesian(turn[i].LeftCartPoint));
            return new Polygon(vertices);
        }

        /// <summary>
        /// Подсчёт коэффициентов полиномов для коридоров.
        /// </summary>
        /// <param name="traj">Часть траектории, заведомо покрывающая время съемки</param>
        /// <param name="startTime">Момент начала съемкиН</param>
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
        public static void GetCoridorParams(Trajectory traj, DateTime startTime, double az, double dist, double roll, double pitch, out double B1, out double B2, out double L1, out double L2, out double S1, out double S2, out double S3, out double duration)
        {
            dist = Math.Min(dist, 97e3);
            //Trajectory traj = fetcher.GetTrajectorySat(start, start.AddMinutes(1)); // должно хватить на коридор
            SatelliteCoordinates satCoord = new SatelliteCoordinates(traj.GetPoint(startTime), roll, pitch);
            GeoPoint startPoint = GeoPoint.FromCartesian(satCoord.MidViewPoint);

            getGeodesicLine(startPoint, az, dist, out B1, out B2, out L1, out L2);
            getDistanceCoef(traj, startTime, dist, roll, pitch, B1, B2, L1, L2, out S1, out S2, out S3, out duration);
        }

        public static void GetCoridorParams(Trajectory traj, DateTime startTime, double start_roll, double start_pitch,
            GeoPoint end,
            out double B1, out double B2, out double L1, out double L2, out double S1, out double S2, out double S3, out double duration, out double dist)
        {
            // Trajectory traj = fetcher.GetTrajectorySat(start, start.AddMinutes(1)); // должно хватить на коридор
            SatelliteCoordinates satCoord = new SatelliteCoordinates(traj.GetPoint(startTime), start_roll, start_pitch);
            GeoPoint startPoint = GeoPoint.FromCartesian(satCoord.MidViewPoint);
            dist = GeoPoint.DistanceOverSurface(startPoint, end) * Astronomy.Constants.EarthRadius * 1e3;

            getGeodesicLineEndPoints(startPoint, end, out B1, out B2, out L1, out L2);
            getDistanceCoef(traj, startTime, dist, start_roll, start_pitch, B1, B2, L1, L2, out S1, out S2, out S3, out duration);
        }

        public static void GetCoridorParams(Trajectory traj, DateTime startTime, GeoPoint startPoint,
            GeoPoint end,
            out double B1, out double B2, out double L1, out double L2, out double S1, out double S2, out double S3, out double duration, out double dist, out double roll, out double pitch)
        {
            //Trajectory traj = fetcher.GetTrajectorySat(startTime, startTime.AddMinutes(1)); // должно хватить на коридор
            dist = GeoPoint.DistanceOverSurface(startPoint, end) * Astronomy.Constants.EarthRadius * 1e3;

            Routines.GetRollPitch(traj.GetPoint(startTime), startPoint, out roll, out pitch);

            getGeodesicLineEndPoints(startPoint, end, out B1, out B2, out L1, out L2);
            getDistanceCoef(traj, startTime, dist, roll, pitch, B1, B2, L1, L2, out S1, out S2, out S3, out duration);
        }

        public static void GetCustomCoridorParams(Trajectory traj, DateTime startTime, Curve curve, out CoridorParams coridorParams)
        {
            //Trajectory traj = fetcher.GetTrajectorySat(startTime, startTime.AddMinutes(1));
            double roll, pitch;
            Routines.GetRollPitch(traj.GetPoint(startTime), curve[0], out roll, out pitch);

            // Find maximum curvature
            int ind_curv = 0;
            for (int i = 1; i < curve.Count - 1; ++i)
            {
                if (Math.Abs(curve.Curvatures[i]) > Math.Abs(curve.Curvatures[ind_curv]))
                {
                    ind_curv = i;
                }
            }
            if (ind_curv == 0 || ind_curv == curve.Count - 1)
                ind_curv = curve.Count / 2;

            ///// Solve lat(s) = A + Bs + Css at s=0 and s=dist.
            ///// Get a solution ABC0 and a kernel V, so that ABC = ABC0 + t * V
            //Vector3D zero_dist = new Vector3D(1, 0, 0);
            //Vector3D midl_dist = new Vector3D(1, dist / 2, dist * dist / 4);
            //Vector3D full_dist = new Vector3D(1, dist, dist * dist);
            //Vector3D ABC0 = Routines.SolveSLE2x3(zero_dist, full_dist, curve[0].Latitude, curve[curve.Count - 1].Latitude);
            //Vector3D V = new Vector3D(0, dist, -1);//Vector3D.CrossProduct(zero_dist, full_dist);
            ///// Let X = (1, x, x*x).
            ///// G(x) is the latitude curve, 
            ///// P2(x) = <V,X>/<V,mid_dist>,
            ///// K(x) = G(x) - <ABC0,X> + <ABC0,mid_dist>P2(x)
            ///// g = <K, P2> / <P2, P2> in L2 sense
            ///// t = (g - <ABC0,mid_dist>) / <V, mid_dist>
            //double KP2 = 0, P2P2 = 0;
            //for (int i = 0; i < curve.Count - 1; ++i)
            //{
            //    Vector3D cur_dist = new Vector3D(1, dists[i], dists[i] * dists[i]);
            //    double P2 = Vector3D.DotProduct(V, cur_dist) / Vector3D.DotProduct(V, midl_dist);
            //    double K = curve[i].Latitude - Vector3D.DotProduct(ABC0, cur_dist);
            //    KP2 += K * P2 * (dists[i + 1] - dists[i]);
            //    P2P2 += P2 * P2 * (dists[i + 1] - dists[i]);
            //}
            ////double g = Vector3D.DotProduct(ABC0, midl_dist) + KP2 / P2P2;
            //double t = (KP2 / P2P2) / Vector3D.DotProduct(V, midl_dist);
            //Vector3D ABC = ABC0 + t * V;
            //B1 = ABC.Y;
            //B2 = ABC.Z;

            // Solve for longitude
            //ind_curv = curve.Count / 2;
            GeoPoint[] refs = new GeoPoint[] { curve[0], curve[ind_curv], curve[curve.Count - 1] };
            double[] lats = refs.Select(gp => AstronomyMath.ToRad(gp.Latitude)).ToArray();
            double[] lons = refs.Select(gp => AstronomyMath.ToRad(gp.Longitude)).ToArray();
            Vector Bm = new Vector(lats[1] - lats[0], lats[2] - lats[0]);
            Vector Lm = new Vector(lons[1] - lons[0], lons[2] - lons[0]);

            //// Find lats based on lons
            //Matrix A_for_preB = new Matrix(new double[][]
            //{ 
            //    new double[] { 1, lons[0], lons[0] * lons[0] },
            //    new double[] { 1, lons[1], lons[1] * lons[1] },
            //    new double[] { 1, lons[2], lons[2] * lons[2] }
            //});
            //Vector preB = Gauss.Solve(A_for_preB, Bm);

            // Find lons based on dist
            double d1 = curve.Meters * (double)ind_curv / (curve.Count - 1);
            double d2 = curve.Meters;
            Matrix A = new Matrix(new double[][] { new double[] { d1, d1 * d1 }, new double[] { d2, d2 * d2 } });
            Vector B = Gauss.Solve(A, Bm);
            double B1 = B[0];
            double B2 = B[1];
            Vector L = Gauss.Solve(A, Lm);
            double L1 = L[0];
            double L2 = L[1];

            //// Plug to find B
            //double B1 = preB[1] * L1 + 2 * preB[2] * lons[0] * L1;
            //double B2 = preB[1] * L2 + preB[2] * (L1 * L1 + 2 * lons[0] * L2);

            // Find average curvature
            //double curv = 0;
            //for (int i = 1; i < curve.Count - 1; ++i)
            //{
            //    curv += (curve[i + 1].Latitude - 2 * curve[i].Latitude + curve[i - 1].Latitude)
            //        / Math.Pow((dists[i + 1] - dists[i - 1]) / 2, 2);
            //}
            //curv /= (curve.Count - 2) * 2;

            //B2 = curv / 2;
            //B1 = (curve[curve.Count - 1].Latitude - curve[0].Latitude - B2 * dist * dist) / dist;


            // Find latitude through longitude and curve lat = F(lon)
            //double dlon1 = curve[1].Longitude - curve[0].Longitude;
            //double dlon2 = curve[2].Longitude - curve[1].Longitude;
            //double F_prime = (curve[1].Latitude - curve[0].Latitude) / dlon1;
            //double F_2prime = (curve[2].Latitude - 2 * curve[1].Latitude + curve[0].Latitude) * 2 / (dlon1 + dlon2);
            //B1 = F_prime * L1;
            //B2 = F_prime * L2 + F_2prime * L1 * L1 / 2;
            double S1, S2, S3, duration;
            getDistanceCoef(traj, startTime, curve.Meters, roll, pitch, B1, B2, L1, L2, out S1, out S2, out S3, out duration);

            coridorParams = new CoridorParams(L1, L2, B1, B2, S1, S2, S3, 0, roll, pitch, startTime, startTime.AddSeconds(duration));
            //{
            //    StartTime = startTime,
            //    EndTime = startTime.AddSeconds(duration),
            //    StartRoll = roll,
            //    StartPitch = pitch,
            //    CoridorCoefs = new PolinomCoef { B1 = B1, B2 = B2, L1 = L1, L2 = L2, S1 = S1, S2 = S2, S3 = S3, WD_K = 0 }
            //};
        }

        private static void getDistanceCoef(Trajectory traj, DateTime startTime, double dist, double roll, double pitch, double b1, double b2, double l1, double l2, out double s1, out double s2, out double s3, out double duration)
        {
            double WD = OptimalChain.RouteParams.WD(roll, pitch);
            List<double> dists = new List<double> { 0 };
            double dt = 1e-1;

            TrajectoryPoint curTP = traj.GetPoint(startTime);
            SatelliteCoordinates satCoord = new SatelliteCoordinates(curTP, roll, pitch);
            GeoPoint curP = GeoPoint.FromCartesian(satCoord.MidViewPoint);
            double startLat = AstronomyMath.ToRad(curP.Latitude);
            double startLon = AstronomyMath.ToRad(curP.Longitude);

            while (dists[dists.Count - 1] < dist * 1.1)
            {
                double s = dists[dists.Count - 1];
                double D = 1e3 * (curTP.Position - GeoPoint.ToCartesian(curP, Astronomy.Constants.EarthRadius)).ToVector().Length;
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
            Vector3D v0 = GeoPoint.ToCartesian(p, 1);
            Vector3D v00 = GeoPoint.ToCartesian(pEnd, 1);
            Arc geodesic = new Arc(v0, v00);
            Arc meridian = new Arc(v0, new Vector3D(0, 0, 1));
            double sin = Vector3D.DotProduct(v0, Vector3D.CrossProduct(meridian.TangentA, geodesic.TangentA));
            double cos = Vector3D.DotProduct(meridian.TangentA, geodesic.TangentA);
            double az = Math.Atan2(sin, cos);
            double dist = geodesic.CentralAngle * Astronomy.Constants.EarthRadius * 1e3;

            getGeodesicLine(p, -az, dist, out B1, out B2, out L1, out L2);

            //double dB = AstronomyMath.ToRad(pEnd.Latitude - p.Latitude);
            //double dL = AstronomyMath.ToRad(pEnd.Longitude - p.Longitude);
            //double midB = AstronomyMath.ToRad(pEnd.Latitude + p.Latitude) / 2;
            //double mSin = Math.Sin(midB), mCos = Math.Cos(midB);

            //double P = dB * (1 - dL * dL / 12 - (dL * mSin) * (dL * mSin) / 24);
            //double Q = dL * mCos * (1 + dB * dB / 12 - (dL * mSin) * (dL * mSin) / 24);
            //double dist = Math.Sqrt(P * P + Q * Q) * Astronomy.Constants.EarthRadius * 1e3;
            //double mA = Math.Atan2(P, Q);
            //double dA = dL * mSin * (1 + (dB * dB + dL * dL) / 12 - (dL * mSin) * (dL * mSin) / 24);
            //double az = mA - dA / 2;

            //getGeodesicLine(p, Math.PI / 2 - az, dist, out B1, out B2, out L1, out L2);
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
            int steps = 2;
            double step = dist / steps;
            var arr = Ode.RK547M(0,
             new Vector(AstronomyMath.ToRad(p.Latitude), AstronomyMath.ToRad(p.Longitude), az),
             (t, x) => new Vector(
                 Math.Cos(x[2]) / Astronomy.Constants.EarthRadius / 1e3,
                 Math.Sin(x[2]) / Math.Cos(x[0]) / Astronomy.Constants.EarthRadius / 1e3,
                 Math.Sin(x[2]) / Astronomy.Constants.EarthRadius / 1e3 * Math.Tan(x[0])),
             new Options { RelativeTolerance = 1e-3 }).SolveFromToStep(0.0, dist, step).ToArray();

            var p0 = arr[0].X;
            var s0 = arr[0].T;
            int mid = steps / 2;
            var p1 = arr[mid].X;
            var s1 = arr[mid].T;
            int end = arr.Length - 1;
            var p2 = arr[end].X;
            var s2 = arr[end].T;

            Bm = new Vector(p1[0] - p0[0], p2[0] - p0[0]);
            Lm = new Vector(p1[1] - p0[1], p2[1] - p0[1]);

            double d1 = mid * step;
            double d2 = end * step;

            Matrix A = new Matrix(new double[][] { new double[] { d1, d1 * d1 }, new double[] { d2, d2 * d2 } });
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


    public class SatelliteCoordinates
    {    
        public TrajectoryPoint trajPos { get; private set; } 

        public SatelliteCoordinates(TrajectoryPoint _trajPos)
        {
            trajPos = _trajPos;
            kaY = -trajPos.Position.ToVector();
            kaZ = trajPos.Velocity;
            kaX = Vector3D.CrossProduct(kaY, kaZ);

            kaX.Normalize();
            kaY.Normalize();
            kaZ.Normalize();

            viewParams = new Lazy<LazyViewParams>(() => calculateViewPolygon());
        }

        public SatelliteCoordinates(TrajectoryPoint _trajPos, double rollAngle, double pitchAngle)
            : this(_trajPos)
        {
            addRollPitchRot(rollAngle, pitchAngle);
        }

        public SatelliteCoordinates(SatelliteCoordinates posCopy)
        {
            trajPos = posCopy.trajPos;
            kaX = posCopy.kaX;
            kaY = posCopy.kaY;
            kaZ = posCopy.kaZ;
            viewParams = posCopy.viewParams;
        }

        /// <summary>
        /// направление на объект съемки 
        /// </summary>
        public Vector3D ViewDir { get { return kaY; } }

        public Vector3D RollAxis { get { return kaZ; } }

        public Vector3D PitchAxis { get { return kaX; } }

        public void addRollPitchRot(double rollAngle, double pitchAngle)
        {
            addRollRot(rollAngle);
            addPitchRot(pitchAngle);
        }
        

        public void addPitchRot(double angle)
        {
            if (angle == 0)
                return;
            Matrix rotMatr = Routines.getRotMatr(kaX, -angle);
            kaY = Routines.applyRotMatr(kaY, rotMatr);
            kaZ = Routines.applyRotMatr(kaZ, rotMatr);
            viewParams = new Lazy<LazyViewParams>(() => calculateViewPolygon());
        }
        

        public void addRollRot(double angle)
        {
            if (angle == 0)
                return;
            /// поворачиваем в обратную сторону, так как за положительный крен принят поворот по часовой
            Matrix rotMatr = Routines.getRotMatr(kaZ, -angle);
            kaX = Routines.applyRotMatr(kaX, rotMatr);
            kaY = Routines.applyRotMatr(kaY, rotMatr);
            viewParams = new Lazy<LazyViewParams>(() => calculateViewPolygon());
        }


        /// <summary>
        /// получить переднюю (по направлению скорости) левую точку полигона видимости
        /// </summary>
        public Vector3D TopLeftViewPoint
        {
            get { return viewParams.Value.topLeftViewPoint; }
        }

        /// <summary>
        /// получить заднюю (по направлению скорости) левую точку полигона видимости
        /// </summary>
        public Vector3D BotLeftViewPoint
        {
            get { return viewParams.Value.botLeftViewPoint; }
        }

        /// <summary>
        /// получить переднюю (по направлению скорости) правую точку полигона видимости
        /// </summary>
        public Vector3D TopRightViewPoint
        {
            get { return viewParams.Value.topRightViewPoint; }
        }


        /// <summary>
        /// получить заднюю (по направлению скорости) правую точку полигона видимости
        /// </summary>
        public Vector3D BotRightViewPoint
        {
            get { return viewParams.Value.botRightViewPoint; }
        }

        /// <summary>
        /// получить центр полигона видимости
        /// </summary>
        public Vector3D MidViewPoint
        {
            get { return viewParams.Value.midViewPoint; }
        }

        public Polygon ViewPolygon
        {
            get{ return viewParams.Value.viewPolygon; }
        }

      
        /// <summary>
        /// 
        /// </summary>
        /// <param name="pointTo">точка (например центр другого кадра), относительно которой выбираем точки</param>
        /// <returns> Tuple<правая точка, левая точка></returns>
        public Polygon getOptimalStrip(SatelliteCoordinates satTo)
        {
            SphericalVector dirVect = new SphericalVector(MidViewPoint, satTo.MidViewPoint);

            var fromPoints = ViewPolygon.Vertices.Where(p => !satTo.ViewPolygon.Contains(p));
            var toPoints = satTo.ViewPolygon.Vertices.Where(p => !this.ViewPolygon.Contains(p));

            var fromPointsLeft = fromPoints.Where(p => dirVect.getPointSide(p) == SphericalVector.PointSide.Left);
            var fromPointsRight = fromPoints.Where(p => dirVect.getPointSide(p) == SphericalVector.PointSide.Right);

            var toPointsLeft = toPoints.Where(p => dirVect.getPointSide(p) == SphericalVector.PointSide.Left);
            var toPointsRight = toPoints.Where(p => dirVect.getPointSide(p) == SphericalVector.PointSide.Right);
            
            SphericalVector left = findSideLine(fromPointsLeft, toPointsLeft, SphericalVector.PointSide.Left);
            SphericalVector right = findSideLine(fromPointsRight, toPointsRight, SphericalVector.PointSide.Right);
             
            Polygon stripPol = new Polygon(
                new List<GeoPoint>(){
                    right.From, right.To, left.To, left.From
            });
            
            return stripPol;
        }


        /// <summary>
        /// получить время и крен, при котором можно снять ту же область с другим тангажом (аргумент pitch)
        /// </summary>
        /// <param name="traj">объект траектории</param>
        /// <param name="pitch">требуемый тангаж</param>
        /// <param name="roll">начальный крен</param>
        /// <param name="maxPitch">Максимально допустимый тангаж</param>
        /// <returns> <время, крен> </returns>
        public Tuple<DateTime, double> getPitchAlternative(Trajectory traj, double pitch, double roll)
        {
            MinimizeFunction func = (_1) => { return getPitchDelta(traj, pitch, _1); };
            double eps = OptimalChain.Constants.roll_correction_epsilon;

            double goalTime = goldenSectionSearch(func, -OptimalChain.Constants.maxPitchTimeDelta, OptimalChain.Constants.maxPitchTimeDelta, eps); 
           
            DateTime newDt = this.trajPos.Time.AddSeconds(goalTime);
            TrajectoryPoint newPos = traj.GetPoint(newDt);
            double newRoll, tmppitch;
            Routines.GetRollPitch(newPos, GeoPoint.FromCartesian(MidViewPoint), out newRoll, out tmppitch);

            return Tuple.Create(newDt, newRoll);
        }



        #region Private part

        private Vector3D kaX;
        private Vector3D kaY;
        private Vector3D kaZ;
        private Lazy<LazyViewParams> viewParams;

        /// <summary>
        /// Вычислить угол тангажа съемки при времени t
        /// </summary>        
        /// <param name="goalPitch">целевой угол тангажа</param>
        /// <param name="t">время в секундах относительно this.trajpos.Time</param>
        /// <returns>разница по модулю между целевым тангажом и полученным</returns>
        private double getPitchDelta(Trajectory traj, double goalPitch, double t)
        {
            double pitch = getPitchForTime(traj, t);
            return Math.Abs(goalPitch - pitch);
        }

        private double getPitchForTime(Trajectory traj, double t)
        {
            DateTime newDt = this.trajPos.Time.AddSeconds(t);
            TrajectoryPoint newPos = traj.GetPoint(newDt);

            double roll, pitch;
            Routines.GetRollPitch(newPos, GeoPoint.FromCartesian(MidViewPoint), out roll, out pitch);
            return pitch;
        }

        private static SphericalVector findSideLine(IEnumerable<Vector3D> fromPoints, IEnumerable<Vector3D> toPoints, SphericalVector.PointSide checkSign)
        {
            var allFromPoints = fromPoints.Concat(toPoints);
            var pointPairs = fromPoints.SelectMany(f => toPoints.Select(t => Tuple.Create(f, t)));
            foreach (var pair in pointPairs)
            {
                bool ok = true;
                SphericalVector testLine = new SphericalVector(pair.Item1, pair.Item2);
                foreach (var testP in allFromPoints)
                {
                    if (testLine.getPointSide(testP) == checkSign)
                    {
                        ok = false;
                        break;
                    }
                }
                if (ok)
                {
                    return testLine;
                }
            }
            throw new Exception("Corridor formation error"); // сюда мы дойти не должны при корректном исполнении
        }

        
        private struct LazyViewParams
        {
            public Vector3D topRightViewPoint;
            public Vector3D topLeftViewPoint;
            public Vector3D botRightViewPoint;
            public Vector3D botLeftViewPoint;
            public Vector3D midViewPoint;
            public Polygon viewPolygon;
        }

        private LazyViewParams calculateViewPolygon()
        {            
            double half = OptimalChain.Constants.camera_angle / 2;
            double tgHalf = Math.Tan(half);

            Vector3D topRight = kaY / tgHalf + kaX + kaZ;
            Vector3D topLeft = kaY / tgHalf - kaX + kaZ;
            Vector3D botLeft = kaY / tgHalf - kaX - kaZ;
            Vector3D botRight = kaY / tgHalf + kaX - kaZ;

            LazyViewParams res = new LazyViewParams();
            res.topRightViewPoint = Routines.SphereVectIntersect(topRight, trajPos.Position, Astronomy.Constants.EarthRadius);
            res.topLeftViewPoint = Routines.SphereVectIntersect(topLeft, trajPos.Position, Astronomy.Constants.EarthRadius);
            res.botRightViewPoint = Routines.SphereVectIntersect(botRight, trajPos.Position, Astronomy.Constants.EarthRadius);
            res.botLeftViewPoint = Routines.SphereVectIntersect(botLeft, trajPos.Position, Astronomy.Constants.EarthRadius);
            res.midViewPoint = Routines.SphereVectIntersect(kaY, trajPos.Position, Astronomy.Constants.EarthRadius);
            res.viewPolygon = new Polygon(new List<Vector3D>()
              { res.topRightViewPoint,
                res.topLeftViewPoint,
                res.botLeftViewPoint, 
                res.botRightViewPoint });
            return res;
        } 

        private delegate double MinimizeFunction(double x);
        private double goldenSectionSearch(MinimizeFunction function, double min, double max, double eps)
        {
            double prop = (1 + Math.Sqrt(5)) / 2;
            double delta = (max - min) / prop;
            double x1 = max - delta;
            double x2 = min + delta;
            double y1 = function(x1);
            double y2 = function(x2);
            if (y1 >= y2)
                min = x1;
            else
                max = x2;

            if (Math.Abs(max - min) < eps)
                return (min + max) / 2;
            else
                return goldenSectionSearch(function, min, max, eps);
        }
        #endregion

    }



    public class SphericalVector
    {
        public SphericalVector(Vector3D f, Vector3D t)            
        {
            CartFrom = f;
            CartTo = t;
            lazyParams = new Lazy<LazyParams>(() => calcSpherical());
        }

        public Vector3D CartFrom { get; private set; }
        public Vector3D CartTo { get; private set; }

        public GeoPoint From
        {
            get { return lazyParams.Value.geoFrom;} 
        }

        public GeoPoint To
        {
            get { return lazyParams.Value.geoTo; }
        }
         
        /// <summary>
        /// с какой стороны от вектора лежит прямая
        /// </summary>
        public enum PointSide { Right, Left, Between }

        public PointSide getPointSide(Vector3D tstP)
        {
            if (tstP.Equals(CartFrom) || tstP.Equals(CartTo))
                return PointSide.Between;

            Vector3D p = tstP;
            Vector3D f = CartFrom;
            Vector3D t = CartTo;

            double res = p.X * f.Y * t.Z - p.X * t.Y * f.Z - f.X * p.Y * t.Z
                + f.X * t.Y * p.Z + t.X * p.Y * f.Z - t.X * f.Y * p.Z;

            if (res < -eps)
                return PointSide.Right;
            if (res > eps)
                return PointSide.Left;

            return PointSide.Between;
        }

        public GeoPoint? getIntersectWith(SphericalVector other)
        {
            SqlGeography intersection = Geography.STIntersection(other.Geography);
            if (intersection.STNumGeometries() == 0)
                return null;
            else
                return new GeoPoint((double)intersection.STPointN(1).Lat, (double)intersection.STPointN(1).Long);
        }

        public SphericalVector getReverse()
        {
            return new SphericalVector(CartTo, CartFrom);
        }

        #region Private part

        private const double eps = 0;// 0.00001; // точность определения стороны точки

        private Lazy<LazyParams> lazyParams;

        private struct LazyParams
        {
            public SqlGeography geography;
            public GeoPoint geoFrom;
            public GeoPoint geoTo;
        }
        
        private LazyParams calcSpherical()
        {
            LazyParams res = new LazyParams();
            res.geoFrom = GeoPoint.FromCartesian(CartFrom);
            res.geoTo = GeoPoint.FromCartesian(CartTo);

            SqlGeographyBuilder builder = new SqlGeographyBuilder();
            builder.SetSrid(4326);
            builder.BeginGeography(OpenGisGeographyType.LineString);
            builder.BeginFigure(res.geoFrom.Latitude, res.geoFrom.Longitude);
            builder.AddLine(res.geoTo.Latitude, res.geoTo.Longitude);
            builder.EndFigure();
            builder.EndGeography();
            res.geography = builder.ConstructedGeography;
            return res;
        }

        public SqlGeography Geography
        {
            get { return lazyParams.Value.geography; }
        } 
        #endregion
    }

    public class Curve
    {
        public GeoPoint[] Vertices { get; private set; }
        public double[] Distances { get; private set; }
        public double Meters { get; private set; }
        public int Count { get { return Vertices.Length; } }
        public double[] DerivativesLat { get; private set; }
        public double[] DerivativesLon { get; private set; }
        public double[] SndDerivativesLat { get; private set; }
        public double[] SndDerivativesLon { get; private set; }
        public double[] Curvatures { get; private set; }

        public double[] DerivativesWRTlon { get; private set; }
        public double[] SndDerivativesWRTlon { get; private set; }
        public double[] CurvaturesWRTlon { get; private set; }

        public double[] CurvD { get; private set; }
        public double Turnage { get; private set; }

        public GeoPoint this[int index] { get { return Vertices[index]; } }

        public Curve(IEnumerable<GeoPoint> vertices)
        {
            Vertices = vertices.ToArray();
            Distances = new double[Count];

            Distances[0] = 0;
            for (int i = 0; i < Count - 1; ++i)
                Distances[i + 1] = GeoPoint.DistanceOverSurface(Vertices[i], Vertices[i + 1]) * Astronomy.Constants.EarthRadius * 1e3;

            Meters = Distances.Sum();

            DerivativesWRTlon = new double[Count];
            SndDerivativesWRTlon = new double[Count];
            DerivativesWRTlon[0] = (Vertices[1].Latitude - Vertices[0].Latitude) / (Vertices[1].Longitude - Vertices[0].Longitude);

            DerivativesLat = new double[Count];
            DerivativesLon = new double[Count];
            SndDerivativesLat = new double[Count];
            SndDerivativesLon = new double[Count];
            DerivativesLat[0] = (Vertices[1].Latitude - Vertices[0].Latitude) / Distances[1];
            DerivativesLon[0] = (Vertices[1].Longitude - Vertices[0].Longitude) / Distances[1];
            for (int i = 1; i < Count - 1; ++i)
            {
                double ds = (Distances[i + 1] + Distances[i]) / 2;
                SndDerivativesLat[i] = (Vertices[i - 1].Latitude - 2 * Vertices[i].Latitude + Vertices[i + 1].Latitude) / (ds * ds);
                SndDerivativesLon[i] = (Vertices[i - 1].Longitude - 2 * Vertices[i].Longitude + Vertices[i + 1].Longitude) / (ds * ds);
                DerivativesLat[i] = (Vertices[i + 1].Latitude - Vertices[i - 1].Latitude) / (2 * ds);
                DerivativesLon[i] = (Vertices[i + 1].Longitude - Vertices[i - 1].Longitude) / (2 * ds);

                double dlon = (Vertices[i + 1].Longitude - Vertices[i - 1].Longitude) / 2;
                SndDerivativesWRTlon[i] = (Vertices[i - 1].Latitude - 2 * Vertices[i].Latitude + Vertices[i + 1].Latitude)
                    / (dlon * dlon);
                DerivativesWRTlon[i] = (Vertices[i + 1].Latitude - Vertices[i - 1].Latitude) / (2 * dlon);

            }
            DerivativesLat[Count - 1] = (Vertices[Count - 1].Latitude - Vertices[Count - 2].Latitude) / Distances[Count - 1];
            DerivativesLon[Count - 1] = (Vertices[Count - 1].Longitude - Vertices[Count - 2].Longitude) / Distances[Count - 1];
            SndDerivativesLat[0] = SndDerivativesLat[1];
            SndDerivativesLat[Count - 1] = SndDerivativesLat[Count - 2];
            SndDerivativesLon[0] = SndDerivativesLon[1];
            SndDerivativesLon[Count - 1] = SndDerivativesLon[Count - 2];
            DerivativesWRTlon[Count - 1] = (Vertices[Count - 1].Latitude - Vertices[Count - 2].Latitude) / (Vertices[Count - 1].Longitude - Vertices[Count - 2].Longitude);
            SndDerivativesWRTlon[0] = SndDerivativesWRTlon[1];
            SndDerivativesWRTlon[Count - 1] = SndDerivativesWRTlon[Count - 2];

            Curvatures = new double[Count];
            CurvaturesWRTlon = new double[Count];
            CurvD = new double[Count];
            Turnage = 0;
            for (int i = 0; i < Count; ++i)
            {
                CurvaturesWRTlon[i] = SndDerivativesWRTlon[i] / Math.Pow(1 + DerivativesWRTlon[i] * DerivativesWRTlon[i], 1.5);
                Curvatures[i] = (DerivativesLat[i] * SndDerivativesLon[i] - DerivativesLon[i] * SndDerivativesLat[i]) /
                    Math.Pow(DerivativesLat[i] * DerivativesLat[i] + DerivativesLon[i] * DerivativesLon[i], 1.5);
                if (i == 0 || i == Count - 1)
                    CurvD[i] = 0;
                else
                    CurvD[i] = Curvatures[i] * (Distances[i] + Distances[i + 1]);
                Turnage += Math.Abs(CurvD[i]);
            }
            //Curvatures[0] = Curvatures[1];
            //Curvatures[Count - 1] = Curvatures[Count - 2];
        }

        public List<Curve> BreakIntoShorterParts(double maxCurv)
        {
            List<Curve> parts = new List<Curve>();

            double curCurv = 0; ;
            int begInd = 0, endInd = 0;

            while (endInd < Count - 1)
            {
                curCurv += Math.Abs(CurvD[endInd]);
                if (curCurv < maxCurv)
                {
                    endInd++;
                    continue;
                }
                else
                {
                    //parts.Add(new Curve(SubArray(this.Vertices, begInd, endInd - begInd + 1)));
                    if (Count - endInd < 2)
                        endInd = Count - 1;
                    parts.Add(new Curve(Vertices.Where((gp, ind) => (ind >= begInd) && (ind <= endInd))));
                    begInd = endInd;
                    curCurv = 0;
                }
            }
            if (begInd != endInd)
            {
                parts.Add(new Curve(Vertices.Where((gp, ind) => (ind >= begInd) && (ind <= endInd))));
            }

            if (parts.Any(curve => curve.Count < 3))
                throw new NotEnougPointsException("Add more intermediate points to the curve.");
            return parts;

            //double thresh = 10;
            //List<List<int>> divisions = new List<List<int>>();
            double totalTV = 0;
            for (int i = 1; i < Count - 1; ++i)
            {
                totalTV += Math.Abs(Curvatures[i] - Curvatures[i + 1]);
            }
            double minfunc = 0;
            List<int> division = new List<int>();
            for (int m = 1; 2 * m + 1 <= Count; ++m)
            {
                foreach (var div in Divide(0, Count - 1, m))
                {
                    div.Add(Count - 1);
                    double maxtv = 0;
                    for (int i = 0; i < m; ++i)
                    {
                        double tv = 0, len = Distances[div[i]];
                        for (int j = div[i] + 1; j < div[i + 1] - 1; ++j)
                        {
                            tv += Math.Abs(Curvatures[j] - Curvatures[j + 1]);
                            len += Distances[j];
                        }
                        maxtv = Math.Max(maxtv, tv / Math.Sqrt(len));
                    }
                    double func = maxtv / totalTV * Math.Sqrt(Meters);
                    if ((m == 1) || (func < minfunc))
                    {
                        minfunc = func;
                        division = div;
                    }
                    //if (maxtv < thresh)
                    //    divisions.Add(div);
                }
                //if (divisions.Count > 0)
                //    break;
            }

            //List<int> minLens = new List<int>();
            //foreach (var div in divisions)
            //{
            //    int step = div[1] - div[0];
            //    for (int i = 1; i < div.Count / 2; ++i)
            //        step = Math.Min(step, div[2 * i + 1] - div[2 * i]);
            //    minLens.Add(step);
            //}
            //int maxMinLen = 0, index = -1;
            //for (int i = 0; i < minLens.Count; ++i)
            //{
            //    if (minLens[i] > maxMinLen)
            //    {
            //        maxMinLen = minLens[i];
            //        index = i;
            //    }
            //}
            //List<int> division = index == -1 ? new List<int>{0, Count - 1} : divisions[index];
            for (int i = 0; i < division.Count - 1; ++i)
            {
                parts.Add(new Curve(Vertices.Where((gp, ind) => (ind >= division[i]) && (ind <= division[i + 1]))));
            }

            return parts;
        }

        public List<Curve> BreakByCurvature()
        {
            List<Curve> curves = new List<Curve>();
            List<GeoPoint> curCurve = new List<GeoPoint>();
            //List<GeoPoint> straight = new List<GeoPoint>() { Vertices[0] };

            //int lastSign = 1, curSign = 1;
            //for (int i = 1; i < Count - 1; ++i)
            //{
            //    if (i == 1)
            //    {
            //        lastSign = Math.Sign(Curvatures[1]);
            //        curCurve.Add(Vertices[0]);
            //        curCurve.Add(Vertices[1]);
            //        continue;
            //    }
            //    curSign = Math.Sign(Curvatures[i]);
            //    if (curSign * lastSign == 1)
            //    {
            //        curCurve.Add(Vertices[i]);
            //    }
            //    else
            //    {
            //        lastSign *= -1;
            //        curves.Add(new Curve(curCurve));
            //        curCurve = new List<GeoPoint>() { Vertices[i - 1], Vertices[i] };
            //        //curCurve = new List<GeoPoint>() { Vertices[i] };
            //    }
            //}
            //curCurve.Add(Vertices[Count - 1]);
            //curves.Add(new Curve(curCurve));

            //double threshold = (0.75 * Curvatures.Average() + 0.25 * Curvatures.Min());

            //for (int i = 1; i < Vertices.Length - 1; ++i)
            //{
            //    if (Math.Abs(Curvatures[i - 1]) < threshold)
            //    {
            //        if (curCurve.Count > 0)
            //        {
            //            if (curCurve.Count < 3)
            //            {
            //                straight.AddRange(curCurve);
            //            }
            //            else
            //            {
            //                curves.Add(new Curve(curCurve));
            //            }
            //            curCurve = new List<GeoPoint>();
            //            straight.Add(Vertices[i - 1]);
            //        }
            //        straight.Add(Vertices[i]);
            //    }
            //    else
            //    {
            //        if (straight.Count > 0)
            //        {
            //            if (straight.Count < 3)
            //            {
            //                curCurve.AddRange(straight);
            //            }
            //            else
            //            {
            //                curves.Add(new Curve(straight));
            //            }
            //            straight = new List<GeoPoint>();
            //            curCurve.Add(Vertices[i - 1]);
            //        }

            //        curCurve.Add(Vertices[i]);
            //    }
            //}
            //curCurve.AddRange(straight);
            //curCurve.Add(Vertices[Vertices.Length - 1]);
            //curves.Add(new Curve(curCurve));

            //double threshold = 1e-9;
            int sign = 0;
            List<int> signChange = new List<int> { 0 };
            int lastIndex = 0;

            for (int i = 1; i < Count - 1; ++i)
            {
                if (Comparison.IsZero(Curvatures[i]))
                    continue;
                else
                {
                    int curSign = Math.Sign(Curvatures[i]);

                    if (sign * curSign == -1)
                    {
                        signChange.Add(lastIndex);
                        signChange.Add(i);
                    }

                    sign = curSign;
                    lastIndex = i;
                }
            }
            signChange.Add(Count - 1);

            int from = 0;
            for (int i = 0; i < signChange.Count / 2 - 1; ++i)
            {
                int to = (signChange[2 * i + 1] + signChange[2 * i + 2]) / 2 + 1;
                if (to - from < 3)
                    continue;
                for (int j = from; j <= to; ++j)
                    curCurve.Add(Vertices[j]);
                from = to - 1;
                curves.Add(new Curve(curCurve));
                curCurve = new List<GeoPoint>();
            }
            for (int j = from; j < Count; ++j)
                curCurve.Add(Vertices[j]);
            curves.Add(new Curve(curCurve));

            return curves;
        }

        public List<Curve> BreakByCurvatureAndDistance(double maxDist)
        {
            var parts = BreakByCurvature();
            List<Curve> res = new List<Curve>();
            foreach (var part in parts)
                res.AddRange(part.BreakIntoShorterParts(maxDist));

            return res;
        }

        public static IEnumerable<List<int>> Divide(int beg, int end, int numOfIntervals)
        {
            if (end - beg >= 3)
            {
                List<int> next = new List<int>();
                if (numOfIntervals == 1)
                {
                    next.Add(beg);
                    //next.Add(end);
                    yield return next;
                }
                else //((numOfIntervals >= 1) && (2 * numOfIntervals <= end - beg))
                {
                    int subPts = 3 * (numOfIntervals - 1);
                    for (int i = beg + 3; i <= end - 3; ++i)
                    {
                        if (subPts <= i - beg && subPts <= end - i)
                        {
                            for (int k = 1; k < numOfIntervals; ++k)
                                foreach (var left in Divide(beg, i, k))
                                    foreach (var right in Divide(i, end, numOfIntervals - k))
                                    {
                                        next.AddRange(left);
                                        next.AddRange(right);
                                        yield return next;
                                        next = new List<int>();
                                    }
                        }
                    }
                }
            }
        }

        public string ToWkt()
        {
            string s = "LINESTRING(";
            for (int i = 0; i < Count; ++i)
            {
                s += Vertices[i].Longitude.ToString().Replace(",", ".") + " " + Vertices[i].Latitude.ToString().Replace(",", ".");
                if (i < Count - 1)
                    s += ",";
            }
            return s + ")";
        }
        
        public class NotEnougPointsException : ArgumentException
        {
            public NotEnougPointsException(string message)
                : base(message)
            { }
        }
    }

}
