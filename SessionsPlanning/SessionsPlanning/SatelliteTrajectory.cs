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
                throw new System.ArgumentException("Wrong arguments. minAngle >= maxAngle", "original");

            List<LanePos> lanePoints = new List<LanePos>();
            
            for (int p_ind = 0; p_ind < points.Count - 1; p_ind += 10)
            {
                Vector3D eDirVect = new Vector3D(-points[p_ind].Position.X, -points[p_ind].Position.Y, -points[p_ind].Position.Z);

                RotateTransform3D leftTransform = new RotateTransform3D(new AxisAngleRotation3D(points[p_ind].Velocity, AstronomyMath.ToDegrees(minAngle)));
                RotateTransform3D rightTransform = new RotateTransform3D(new AxisAngleRotation3D(points[p_ind].Velocity, AstronomyMath.ToDegrees(maxAngle)));

                Vector3D leftVector = leftTransform.Transform(eDirVect);
                Vector3D rightVector = rightTransform.Transform(eDirVect);

                Vector3D leftCrossPoint = SphereVectIntersect(leftVector, points[p_ind].Position, Astronomy.Constants.EarthRadius).ToVector();
                Vector3D rightCrossPoint = SphereVectIntersect(rightVector, points[p_ind].Position, Astronomy.Constants.EarthRadius).ToVector();
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
        public List<LanePos> lanePoints;

        public List<LaneSector> Sectors;

        public SatLane(double minAngle, double maxAngle, List<LanePos> points)
        {
            Sectors = new List<LaneSector>();
            lanePoints = points;


            double width = (points[0].LeftPoint - points[0].RightPoint).Length / 2;

            //GeoPoint lgp1 = new GeoPoint(10, 0);
            //GeoPoint rgp1 = new GeoPoint(-10, 0);

            //GeoPoint lgp2 = new GeoPoint(10, 150);
            //GeoPoint rgp2 = new GeoPoint(-10, 150);

            //GeoPoint lgp1 = new GeoPoint(-21.8312349379119, -174.294215474924);
            //GeoPoint rgp1 = new GeoPoint(-13.3636199339284, 174.148629271623);

            //GeoPoint lgp2 = new GeoPoint(44.643794933894, -88.2042299006211);
            //GeoPoint rgp2 = new GeoPoint(58.2581814645211, -90.0751693889995);

            //Vector3D lp1 = GeoPoint.ToCartesian(lgp1, 1);
            //Vector3D rp1 = GeoPoint.ToCartesian(rgp1, 1);
            //Vector3D lp2 = GeoPoint.ToCartesian(lgp2, 1);
            //Vector3D rp2 = GeoPoint.ToCartesian(rgp2, 1);

            //Vector3D cpl = lp1 - rp1;
            //cpl.Normalize();
            //cpl = cpl * (lp1 - rp1).Length / 2;

            //Vector3D cpr = rp1 - lp1;
            //cpr.Normalize();
            //cpr = cpr * (rp1 - lp1).Length / 2;

            //LaneSector sector = new LaneSector();
            //List<Vector3D> pints = new List<Vector3D>();
            //pints.Add(lp1);
            //pints.Add(rp1);
            //pints.Add(rp2);
            //pints.Add(lp2);

            //List<Vector3D> cpints = new List<Vector3D>();
            //cpints.Add(new Vector3D(0, 0, 0));
            //cpints.Add(cpr);
            //cpints.Add(new Vector3D(0, 0, 0));
            //cpints.Add(cpl);

            //sector.polygon = new Polygon(pints, cpints);
            //Sectors.Add(sector);
                        
            //return;
            

            List<Vector3D> leftLanePoints = new List<Vector3D>();
            List<Vector3D> rightLanePoints = new List<Vector3D>();
            
            List<Vector3D> leftControlPoints = new List<Vector3D>();
            List<Vector3D> rightControlPoints = new List<Vector3D>();

            DateTime sectorFromDT = points[0].time;
            DateTime sectorToDT;

            GeoPoint prevPoint = AstronomyMath.GreenwichToSpherical(points[0].LeftPoint.ToPoint());
            for (int p_ind = 0; p_ind < points.Count; p_ind++)
            {
                leftLanePoints.Add(points[p_ind].LeftPoint);
                rightLanePoints.Add(points[p_ind].RightPoint);
                
                Vector3D leftControlPoint = points[p_ind].LeftPoint - points[p_ind].RightPoint;
                leftControlPoint.Normalize();
                leftControlPoint = leftControlPoint * (points[p_ind].LeftPoint - points[p_ind].RightPoint).Length / 2;

                Vector3D rightControlPoint = points[p_ind].RightPoint - points[p_ind].LeftPoint;
                rightControlPoint.Normalize();
                rightControlPoint = rightControlPoint * (points[p_ind].RightPoint - points[p_ind].LeftPoint).Length / 2;
                
                leftControlPoints.Add(leftControlPoint);
                rightControlPoints.Add(rightControlPoint);

                if (p_ind == 0)
                    continue;

                GeoPoint point = GeoPoint.FromCartesian(points[p_ind].LeftPoint);

                bool needNewSector = false;

                if (p_ind == points.Count - 1)
                {
                    needNewSector = true;
                }                    
                else
                {
                    GeoPoint nextPoint = GeoPoint.FromCartesian(points[p_ind + 1].LeftPoint);
                    double curDist = GeoPoint.DistanceOverSurface(prevPoint, point);
                    double distNext = GeoPoint.DistanceOverSurface(prevPoint, nextPoint);
                    needNewSector = distNext < curDist;
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

                    leftLanePoints.Add(points[p_ind].LeftPoint);
                    rightLanePoints.Add(points[p_ind].RightPoint);

                    leftControlPoints.Add(leftControlPoint);
                    rightControlPoints.Add(rightControlPoint);

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
            double square = region.Square();
            foreach(LaneSector sector in Sectors)
            {
                Tuple<IList<Polygon>, IList<Polygon>> reTuple = Polygon.IntersectAndSubtract(sector.polygon, region);
                foreach (var int_pol in reTuple.Item1)
                {
                    var verts = int_pol.Vertices;
                    var en = verts.GetEnumerator();
                    en.MoveNext();
                    DateTime tmin = getPointTime(en.Current, sector.fromDT, sector.toDT);
                    DateTime tmax = tmin;
                    Vector3D minPoint, maxPoint;
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
                            minPoint = point;
                            tmin = curTime;
                        }
                        else if (curTime > tmax)
                        {
                            maxPoint = point;
                            tmax = curTime;
                        }
                    }
                    if (outOfRange)
                        break;
                    CaptureConf newcc = new CaptureConf();
                    Order order = new Order();
                    order.request = request;
                    double subsquare = int_pol.Square();
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
            double minDist = lanePoints[0].getDistToPoint(geoPoint);
            int minInd = 0;

            for (int i = 1; i < lanePoints.Count - 1; i++)
            {
                if (lanePoints[i].time < fromDt)
                    continue;
                else if (lanePoints[i].time > toDt)
                    break;

                double dist = lanePoints[i].getDistToPoint(geoPoint);
                if (dist < minDist)
                {
                    minDist = dist;
                    minInd = i;
                }
            }

            LanePos curPos = lanePoints[minInd];
            LanePos nextPos = (minInd == lanePoints.Count - 1 ) ? lanePoints[minInd - 1] : lanePoints[minInd + 1];
 
            double distCur = curPos.getDistToPoint(geoPoint);
            double distNext = nextPos.getDistToPoint(geoPoint);

            double diff = Math.Abs((curPos.time - nextPos.time).TotalSeconds) * distCur / (distCur + distNext);
            DateTime res = (minInd == lanePoints.Count - 1) ? nextPos.time.AddSeconds(diff) : curPos.time.AddSeconds(diff);             
            return res;
        }
        

        public Polygon getSegment(DateTime begTime, DateTime endTime)
        {
            if (lanePoints[0].time > begTime || lanePoints[lanePoints.Count - 1].time < endTime)
                throw new System.ArgumentException("Incorrect time interval.");

            List<Vector3D> polygonPoints = new List<Vector3D>();
            List<Vector3D> rightPolygonPoints = new List<Vector3D>();

            for (int i = 0; i < lanePoints.Count - 1; i++)
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
                    polygonPoints.Add(lanePoints[i].LeftPoint);
                    rightPolygonPoints.Add(lanePoints[i].RightPoint);
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
        private Vector3D leftCartPoint;
        private Vector3D rightCartPoint;
        public Vector3D middleCartPoint {get; set;}
        // точка, поворот вокруг которой на нужный угол даст нужный малый круг.
        public Vector3D pointToPlane { get; set; }

        private GeoPoint leftGeoPoint;
        private GeoPoint rightGeoPoint;

        private double width; 

        public DateTime time;

        public LanePos(Vector3D _leftPoint, Vector3D _middlePoint, Vector3D _rightPoint, DateTime _time)
        {
            LeftPoint = _leftPoint;
            RightPoint = _rightPoint;
            middleCartPoint = _middlePoint;
            time = _time;
        }

        public Vector3D LeftPoint
        {
            get
            {
                return leftCartPoint;
            }
            set
            {
                leftCartPoint = value;
                leftGeoPoint = GeoPoint.FromCartesian(value);
                width = GeoPoint.DistanceOverSurface(leftGeoPoint, rightGeoPoint);
            }
        }

        public Vector3D RightPoint
        {
            get
            {
                return rightCartPoint;
            }
            set
            {
                rightCartPoint = value;
                rightGeoPoint = GeoPoint.FromCartesian(value);
                width = GeoPoint.DistanceOverSurface(leftGeoPoint, rightGeoPoint);
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

