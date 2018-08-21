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


        public void ComputeCoridorPolygon(Trajectory traj, int pointsPerSide = 20)
        {
            TrajectoryPoint prevTP = traj.GetPoint(StartTime);
            SatelliteCoordinates prevKaPos = new SatelliteCoordinates(prevTP, StartRoll, StartPitch);
            GeoPoint prevCurveGeo = GeoPoint.FromCartesian(prevKaPos.MidViewPoint);
            Vector3D prevCurveVec = GeoPoint.ToCartesian(prevCurveGeo, 1);
            Parametrized2Curve curve = new Parametrized2Curve(prevCurveGeo, CoridorCoefs);

            double dt = Duration / pointsPerSide;
            double rollAngle, pitchAngle;
            AbsMaxRequiredRoll = 0;
            AbsMaxRequiredPitch = 0;

            SphericalVector[] rightLines = new SphericalVector[pointsPerSide - 1]; 
            SphericalVector[] leftLines = new SphericalVector[pointsPerSide - 1]; 
           
            Coridor = prevKaPos.ViewPolygon;

            for (int i = 1; i < pointsPerSide; i++)
            {
                double t = dt * i;
                TrajectoryPoint curTP = traj.GetPoint(StartTime.AddSeconds(t));
                GeoPoint curCurveGeo = curve.GetPoint(t);
                Routines.GetRollPitch(curTP, curCurveGeo, out rollAngle, out pitchAngle);

                AbsMaxRequiredRoll = Math.Max(AbsMaxRequiredRoll, Math.Abs(rollAngle));
                AbsMaxRequiredPitch = Math.Max(AbsMaxRequiredPitch, Math.Abs(pitchAngle));

                SatelliteCoordinates curKaPos = new SatelliteCoordinates(curTP);
                curKaPos.addRollPitchRot(rollAngle, pitchAngle);        

                Polygon curPol = prevKaPos.getOptimalStrip(curKaPos);
                Coridor.Add(curPol);
                Coridor.Add(curKaPos.ViewPolygon);
                
                prevKaPos = curKaPos;
            }             
            if (!Coridor.IsValid())
            {
                throw new Exception("Corridor formation error");
            }
        }       
    }
}

