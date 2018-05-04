using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Common;
using Astronomy;
using System.Windows.Media.Media3D;
using SphericalGeom;
using SatelliteTrajectory;

namespace SatelliteSessions
{
    public class Parametrized2Curve
    {
        public double L1 { get; private set; }
        public double L2 { get; private set; }
        public double B1 { get; private set; }
        public double B2 { get; private set; }
        public double S1 { get; private set; }
        public double S2 { get; private set; }
        public double S3 { get; private set; }
        public PolinomCoef PolinomCoefs
        {
            get
            {
                return new PolinomCoef(L1, L2, B1, B2, S1, S2, S3, 0);
            }
        }
        private double begLat, begLon;

        public Parametrized2Curve(GeoPoint start, double l1, double l2, double b1, double b2, double s1, double s2, double s3)
        {
            this.begLat = AstronomyMath.ToRad(start.Latitude);
            this.begLon = AstronomyMath.ToRad(start.Longitude);

            L1 = l1;
            L2 = l2;
            B1 = b1;
            B2 = b2;
            S1 = s1;
            S2 = s2;
            S3 = s3;
        }

        public Parametrized2Curve(GeoPoint start, PolinomCoef coefs)
            : this(start, coefs.L1, coefs.L2, coefs.B1, coefs.B2, coefs.S1, coefs.S2, coefs.S3) { }


        public double GetDistance(double dt)
        {
            return dt * (S1 + dt * (S2 + dt * S3));
        }

        public double GetLatitude(double dt)
        {
            double d = GetDistance(dt);
            return begLat + d * (B1 + d * B2);
        }

        public double GetLongitude(double dt)
        {
            double d = GetDistance(dt);
            return begLon + d * (L1 + d * L2);
        }

        public GeoPoint GetPoint(double dt)
        {
            return new GeoPoint(AstronomyMath.ToDegrees(GetLatitude(dt)), AstronomyMath.ToDegrees(GetLongitude(dt)));
        }

        public Vector3D GetUnitVector(double dt)
        {
            return GeoPoint.ToCartesian(GetPoint(dt), 1);
        }
    }

    public class PolinomCoef
    {
        public double L1 { get; private set; }
        public double L2 { get; private set; }
        public double B1 { get; private set; }
        public double B2 { get; private set; }
        public double S1 { get; private set; }
        public double S2 { get; private set; }
        public double S3 { get; private set; }
        public double WD_K { get; private set; }

        public PolinomCoef(double l1, double l2, double b1, double b2, double s1, double s2, double s3, double wd_k = 0)
        {
            L1 = l1;
            L2 = l2;
            B1 = b1;
            B2 = b2;
            S1 = s1;
            S2 = s2;
            S3 = s3;
            WD_K = wd_k;
        }
    }

    public class CoridorParams
    {
        public PolinomCoef CoridorCoefs { get; private set; }
        public double StartRoll { get; private set; }
        public double StartPitch { get; private set; }
        public DateTime StartTime { get; private set; }
        public DateTime EndTime { get; private set; }
        /// <summary>
        /// In seconds.
        /// </summary>
        public double Duration { get { return (EndTime - StartTime).TotalSeconds; } }
        
        public Polygon Coridor { get; private set; }
        public double AbsMaxRequiredRoll { get; private set; }
        public double AbsMaxRequiredPitch { get; private set; }

        public CoridorParams(double l1, double l2, double b1, double b2, double s1, double s2, double s3, double wd_k, double startRoll, double startPitch, DateTime startTime, DateTime endTime)
            : this(new PolinomCoef(l1, l2, b1, b2, s1, s2, s3, wd_k), startRoll, startPitch, startTime, endTime) { }

        public CoridorParams(PolinomCoef coridorCoefs, double startRoll, double startPitch, DateTime startTime, DateTime endTime)
        {
            CoridorCoefs = coridorCoefs;
            StartRoll = startRoll;
            StartPitch = startPitch;
            StartTime = startTime;
            EndTime = endTime;
        }

        /// <summary>
        /// </summary>
        /// <param name="traj">Отрезок траектории КА покрывающий время съемки коридора</param>
        /// <param name="pointsPerSide">Число точек на боковинах коридора</param>
        public void ComputeCoridorPolygon(Trajectory traj, int pointsPerSide = 10)
        {
            int vertNum = pointsPerSide * 2 + 2;
            Vector3D[] points = new Vector3D[vertNum];

            TrajectoryPoint curTP = traj.GetPoint(StartTime);
            SatelliteCoordinates curKaPos = new SatelliteCoordinates(curTP, StartRoll, StartPitch);
            GeoPoint nexCurveGeo = GeoPoint.FromCartesian(curKaPos.MidViewPoint);
            Vector3D nexCurveVec = GeoPoint.ToCartesian(nexCurveGeo, 1);
            Parametrized2Curve curve = new Parametrized2Curve(nexCurveGeo, CoridorCoefs);

            GeoPoint curCurveGeo;
            Vector3D curCurveVec;

            double dt = Duration / pointsPerSide;
            double rollAngle, pitchAngle;
            AbsMaxRequiredRoll = 0;
            AbsMaxRequiredPitch = 0;
            for (int i = 0; i < pointsPerSide; i++)
            {
                curCurveVec = nexCurveVec;
                curCurveGeo = nexCurveGeo;

                double t = dt * i;
                curTP = traj.GetPoint(StartTime.AddSeconds(t));
                Routines.GetRollPitch(curTP, curCurveGeo, out rollAngle, out pitchAngle);

                AbsMaxRequiredRoll = Math.Max(AbsMaxRequiredRoll, Math.Abs(rollAngle));
                AbsMaxRequiredPitch = Math.Max(AbsMaxRequiredPitch, Math.Abs(pitchAngle));

                curKaPos = new SatelliteCoordinates(curTP);
                Vector3D rAxis = curKaPos.RollAxis;
                Vector3D pAxis = curKaPos.PitchAxis;
                curKaPos.addRollPitchRot(rollAngle, pitchAngle);

                nexCurveGeo = curve.GetPoint(t + dt);
                nexCurveVec = GeoPoint.ToCartesian(nexCurveGeo, 1);
                Vector3D dVec = nexCurveVec - curCurveVec;

                double along = Vector3D.DotProduct(dVec, rAxis);
                double perp = Vector3D.DotProduct(dVec, pAxis);
                bool forward = Comparison.IsPositive(along);
                bool left = !Comparison.IsPositive(perp);

                if (i == 0)
                {
                    // points[0] is an extra beggining point
                    if (forward && left)
                    {
                        points[0] = curKaPos.BotRightViewPoint;
                        points[1] = curKaPos.BotLeftViewPoint;
                        points[vertNum - 1] = curKaPos.TopRightViewPoint;
                    }
                    else if (forward && !left)
                    {
                        points[0] = curKaPos.BotLeftViewPoint;
                        points[1] = curKaPos.TopLeftViewPoint;
                        points[vertNum - 1] = curKaPos.BotRightViewPoint;
                    }
                    else if (!forward && left)
                    {
                        points[0] = curKaPos.TopRightViewPoint;
                        points[1] = curKaPos.BotRightViewPoint;
                        points[vertNum - 1] = curKaPos.TopLeftViewPoint;
                    }
                    else // if (!forward && !left)
                    {
                        points[0] = curKaPos.TopLeftViewPoint;
                        points[1] = curKaPos.TopRightViewPoint;
                        points[vertNum - 1] = curKaPos.BotLeftViewPoint;
                    }
                }
                else if (i == pointsPerSide - 1)
                {
                    // points[pointsPerSide + 1] is an extra ending point
                    if (forward && left)
                    {
                        points[pointsPerSide] = curKaPos.BotLeftViewPoint;
                        points[pointsPerSide + 1] = curKaPos.TopLeftViewPoint;
                        points[pointsPerSide + 2] = curKaPos.TopRightViewPoint;
                    }
                    else if (forward && !left)
                    {
                        points[pointsPerSide] = curKaPos.TopLeftViewPoint;
                        points[pointsPerSide + 1] = curKaPos.TopRightViewPoint;
                        points[pointsPerSide + 2] = curKaPos.BotRightViewPoint;
                    }
                    else if (!forward && left)
                    {
                        points[pointsPerSide] = curKaPos.BotRightViewPoint;
                        points[pointsPerSide + 1] = curKaPos.BotLeftViewPoint;
                        points[pointsPerSide + 2] = curKaPos.TopLeftViewPoint;
                    }
                    else // if (!forward && !left)
                    {
                        points[pointsPerSide] = curKaPos.TopRightViewPoint;
                        points[pointsPerSide + 1] = curKaPos.BotRightViewPoint;
                        points[pointsPerSide + 2] = curKaPos.BotLeftViewPoint;
                    }
                }
                else
                {
                    if (forward && left)
                    {
                        points[i + 1] = curKaPos.BotLeftViewPoint;
                        points[vertNum - 1 - i] = curKaPos.TopRightViewPoint;
                    }
                    else if (forward && !left)
                    {
                        points[i + 1] = curKaPos.TopLeftViewPoint;
                        points[vertNum - 1 - i] = curKaPos.BotRightViewPoint;
                    }
                    else if (!forward && left)
                    {
                        points[i + 1] = curKaPos.BotRightViewPoint;
                        points[vertNum - 1 - i] = curKaPos.TopLeftViewPoint;
                    }
                    else // if (!forward && !left)
                    {
                        points[i + 1] = curKaPos.TopRightViewPoint;
                        points[vertNum - 1 - i] = curKaPos.BotLeftViewPoint;
                    }
                }
            }

            Coridor = new Polygon(points.ToList());
        }
    }
}
