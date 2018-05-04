using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Common;
using Astronomy;
using System.Windows.Media.Media3D;
using SphericalGeom;

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
                return new PolinomCoef { L1 = L1, L2 = L2, B1 = B1, B2 = B2, S1 = S1, S2 = S2, S3 = S3, WD_K = 0 };
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
        public double L1 { get; set; }
        public double L2 { get; set; }
        public double B1 { get; set; }
        public double B2 { get; set; }
        public double WD_K { get; set; }
        public double S1 { get; set; }
        public double S2 { get; set; }
        public double S3 { get; set; }
    }

    public class CoridorParams
    {
        public PolinomCoef CoridorCoefs { get; set; }
        public double AbsMaxRequiredRoll { get; set; }
        public double AbsMaxRequiredPitch { get; set; }
        public double StartRoll { get; set; }
        public double StartPitch { get; set; }
        public DateTime StartTime { get; set; }
        public DateTime EndTime { get; set; }
        /// <summary>
        /// In seconds.
        /// </summary>
        public double Duration { get { return (EndTime - StartTime).TotalSeconds; } }
        public Polygon Coridor { get; set; }
    }
}
