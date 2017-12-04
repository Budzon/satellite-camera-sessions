using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;
using Common;

namespace Astronomy
{
    public static class AstronomyMath
    {
        #region Quaternion conversions

        /// <summary>
        /// Converts Krylov angles in radians to quaternion.
        /// </summary>
        /// <param name="heading">Курс (радианы)</param>
        /// <param name="roll">Крен (радианы)</param>
        /// <param name="pitch">Тангаж (радианы)</param>
        /// <returns></returns>
        public static Quaternion KrylovToQuaternion(double heading, double roll, double pitch)
        {
            double cosH = Math.Cos(heading / 2.0);
            double sinH = Math.Sin(heading / 2.0);
            double cosR = Math.Cos(roll / 2.0);
            double sinR = Math.Sin(roll / 2.0);
            double cosP = Math.Cos(pitch / 2.0);
            double sinP = Math.Sin(pitch / 2.0);

            double w = cosH * cosP * cosR - sinH * sinP * sinR;
            double x = sinH * sinP * cosR + cosH * cosP * sinR;
            double y = sinH * cosP * cosR + cosH * sinP * sinR;
            double z = cosH * sinP * cosR - sinH * cosP * sinR;

            return new Quaternion(x, y, z, w);
        }

        /// <summary>
        /// Converts quaternion into Krylov angles in radians.
        /// </summary>
        /// <param name="q"></param>
        /// <param name="heading">Курс (радианы)</param>
        /// <param name="roll">Крен (радианы)</param>
        /// <param name="pitch">Тангаж (радианы)</param>
        public static void QuaternionToKrylov(Quaternion q, out double heading, out double roll, out double pitch)
        {
            double kyrs = GetKyrs(q);
            double tongazh = GetTongazh(q);
            double kren = GetRoll(q);

            heading = kyrs;
            roll = kren;
            pitch = tongazh;
        }

        private static double GetKyrs(Quaternion orientation)
        {
            double kyrs;
            double kyrsznam = orientation.W * orientation.W + orientation.X * orientation.X - 0.5;
            double kyrschisl = orientation.W * orientation.Y - orientation.X * orientation.Z;
            kyrs = Math.Atan2(kyrschisl, kyrsznam);

            if (kyrs < 0)
                kyrs += 2 * Math.PI;

            return kyrs;
        }

        private static double GetTongazh(Quaternion orientation)
        {
            double tongazh = Math.Asin(2 * (orientation.X * orientation.Y + orientation.W * orientation.Z));

            if (tongazh < 0)
                tongazh += 2 * Math.PI;

            return tongazh;
        }

        /// <summary>
        /// Строит угол крена (рад.) по кватерниону.
        /// </summary>
        /// <param name="orientation"></param>
        /// <returns></returns>
        public static double GetRoll(Quaternion orientation)
        {
            double kren;
            double krenchisl = orientation.W * orientation.X - orientation.Y * orientation.Z;
            double krenznam = orientation.W * orientation.W + orientation.Y * orientation.Y - 0.5;
            kren = Math.Atan2(krenchisl, krenznam);

            if (kren < 0)
                kren += 2 * Math.PI;

            return kren;
        }

        #endregion

        /// <summary>
        /// Строит матрицу перехода из СК2 в СК1. Параметры: базисные вектора СК2 в координатах CК1.
        /// </summary>
        /// <param name="newX">Базисный вектор СК2 X в координатах CК1.</param>
        /// <param name="newY">Базисный вектор СК2 Y в координатах CК1.</param>
        /// <param name="newZ">Базисный вектор СК2 Z в координатах CК1.</param>
        /// <returns>Матрица перехода из СК2 в СК1.</returns>
        public static Matrix3D CS2toCS1(Vector3D newX, Vector3D newY, Vector3D newZ)
        {
            return new Matrix3D(
                newX.X, newX.Y, newX.Z, 0,
                newY.X, newY.Y, newY.Z, 0,
                newZ.X, newZ.Y, newZ.Z, 0,
                0, 0, 0, 1);
        }

        /// <summary>
        /// Строит матрицу перехода из СК2 в СК1. Параметры: базисные вектора СК2 в координатах CК1.
        /// </summary>
        /// <param name="newX">Базисный вектор СК2 X в координатах CК1.</param>
        /// <param name="newY">Базисный вектор СК2 Y в координатах CК1.</param>
        /// <param name="newZ">Базисный вектор СК2 Z в координатах CК1.</param>
        /// <param name="offset">Смещение начала СК2 в координатах СК1</param>
        /// <returns>Матрица перехода из СК2 в СК1.</returns>
        public static Matrix3D CS2toCS1(Vector3D newX, Vector3D newY, Vector3D newZ, Point3D offset)
        {
            return new Matrix3D(
                newX.X, newX.Y, newX.Z, 0,
                newY.X, newY.Y, newY.Z, 0,
                newZ.X, newZ.Y, newZ.Z, 0,
                offset.X, offset.Y, offset.Z, 1);
        }


        /// <summary>
        /// Converts a velocity vectory from Absolute CS to Greenwich CS.
        /// vAbs in km/s, locGcs in km.
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public static Vector3D VelocityACStoGCS(Vector3D vAbs, Point3D locGcs)
        {
            Vector3D w = new Vector3D(0, 0, 0.072921151467);
            Vector3D vGcs = vAbs - Vector3D.CrossProduct(w, 0.001 * locGcs.ToVector());
            return vGcs;
        }

        public static double GetStarTime(DateTime time)
        {
            JulianDate jd = new JulianDate(time.Year, time.Month, time.Day);
            var julian_days = jd.GetDays();

            double s = time.Second + time.Millisecond / 1000.0;
            double DJ = julian_days - 2415020.5;
            double TIN = 0.001 * s + 0.06 * time.Minute + 3.6 * time.Hour;
            double SZ0 = 0.276919398 + 2.73790926493e-3 * DJ +
                         1.075231E-6 * (DJ / 36525) * (DJ / 36525) +
                         1.0027379093 * (TIN / 86.4 /*- 0.125*/);
            return 2 * Math.PI * (SZ0 - Math.Truncate(SZ0));
        }

        /// <summary>
        /// Transforms a quaternion from Inertial Coordinate System to Greenwich Coordinate System.
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public static Quaternion ICStoGCS(Quaternion q, double starTime)
        {
            Quaternion ics = new Quaternion(new Vector3D(0, 0, 1), -starTime / Math.PI * 180.0);
            q = q * ics;
            q.Normalize();
            return q;
        }

        public static Quaternion GCStoICS(Quaternion q, double starTime)
        {
            Quaternion ics = new Quaternion(new Vector3D(0, 0, 1), starTime / Math.PI * 180.0);
            q = q * ics;
            q.Normalize();
            return q;
        }

        /// <summary>
        /// Transforms a vector from Inertial Coordinate System to Greenwich Coordinate System.
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public static Vector3D ICStoGCS(Vector3D v, double starTime)
        {
            double cw = Math.Cos(starTime);
            double sw = Math.Sin(starTime);

            Vector3D u = new Vector3D(
                cw * v.X + sw * v.Y,
                -sw * v.X + cw * v.Y,
                v.Z);
            return u;
        }

        /// <summary>
        /// Transforms a point from Inertial Coordinate System to Greenwich Coordinate System.
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public static Point3D ICStoGCS(Point3D v, double starTime)
        {
            double cw = Math.Cos(starTime);
            double sw = Math.Sin(starTime);

            Point3D u = new Point3D(
                cw * v.X + sw * v.Y,
                -sw * v.X + cw * v.Y,
                v.Z);
            return u;
        }

        /// <summary>
        /// Transform from Greenwich Coordinate System to Inertial Coordinate System with a specific star time
        /// </summary>
        /// <param name="starTime">star time of GCS</param>
        /// <returns></returns>
        public static Matrix3D GCStoICS(double starTime)
        {
            double cw = Math.Cos(starTime);
            double sw = Math.Sin(starTime);

            return new Matrix3D(
                cw, sw, 0, 0,
                -sw, cw, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1
                );
        }

        /// <summary>
        /// Transform from Inertial Coordinate System to Greenwich Coordinate System with a specific star time
        /// </summary>
        /// <param name="starTime">star time of GCS</param>
        /// <returns></returns>
        public static Matrix3D ICStoGCS(double starTime)
        {
            double cw = Math.Cos(starTime);
            double sw = Math.Sin(starTime);

            return new Matrix3D(
                cw, -sw, 0, 0,
                sw, cw, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1
                );
        }

        /// <summary>
        /// Transforms a point from Greenwich Coordinate System to Inertial Coordinate System.
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public static Point3D GCStoICS(Point3D v, double starTime)
        {
            double cw = Math.Cos(starTime);
            double sw = Math.Sin(starTime);

            Point3D u = new Point3D(
                cw * v.X + -sw * v.Y,
                sw * v.X + cw * v.Y,
                v.Z);
            return u;
        }

        public static Vector3D GetDirectionToSun(DateTime time)
        {
            var sun = Astronomy.SunPosition.GetPositionGreenwich(time).ToVector();
            return sun;
        }

        /// <summary>
        /// Transforms (lat,lon) into Greenwich coordinate system (Z is up, X to zero lon).
        /// </summary>
        /// <param name="lat">In degrees.</param>
        /// <param name="lon">In degrees.</param>
        /// <param name="rad"></param>
        /// <returns></returns>
        public static Point3D SphericalToGreenwich(double lat, double lon, double rad)
        {
            lon = Math.PI * lon / 180;
            lat = Math.PI * lat / 180;
            double xphi = rad * Math.Cos(lat);
            double x = xphi * Math.Cos(lon);
            double y = xphi * Math.Sin(lon);
            double z = rad * Math.Sin(lat);
            return new Point3D(x, y, z);
        }

        public static GeoPoint GreenwichToSpherical(Point3D p)
        {
            double rxy = Math.Sqrt(p.X * p.X + p.Y * p.Y);
            double len = Math.Sqrt(p.X * p.X + p.Y * p.Y + p.Z * p.Z);
            double lat = Math.Asin(p.Z / len);
            double lon = Math.Acos(p.X / rxy);
            if (p.Y < 0) lon = 2 * Math.PI - lon;
            return new GeoPoint(lat, lon, true, GeoPoint.Range.RangeLat90 | GeoPoint.Range.RangeLong180);
        }

        public static Vector3D ToVector(this Point3D p)
        {
            return new Vector3D(p.X, p.Y, p.Z);
        }

        public static Point3D ToPoint(this Vector3D v)
        {
            return new Point3D(v.X, v.Y, v.Z);
        }

        public static double ToDegrees(double rad)
        {
            return rad / Math.PI * 180.0;
        }

        public static double ToRad(double degrees)
        {
            return degrees / 180.0 * Math.PI;
        }

        /// <summary>
        /// Returns transform from inclined cs to normal (nip2 to nip1).
        /// </summary>
        /// <param name="azimuth"></param>
        /// <param name="inclination"></param>
        /// <returns></returns>
        public static Matrix3D RotateCS(double azimuth, double inclination)
        {
            var Rbeta = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), AstronomyMath.ToDegrees(inclination - Math.PI / 2))).Value;
            var Ralpha = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), AstronomyMath.ToDegrees(-azimuth))).Value;
            return Rbeta * Ralpha;
        }

        /// <summary>
        /// Gets the normalized vector that tangentially points from given origin to the North Pole (in GSK).
        /// </summary>
        /// <param name="origin"></param>
        /// <returns></returns>
        public static Vector3D GetZeroAzimuth(Point3D origin)
        {
            double c = 1 / (origin.X * origin.X + origin.Y * origin.Y);
            double nyz = Math.Sqrt(1 / (1 + origin.Z * origin.Z * c));
            double d = origin.Z * nyz * c;
            double nyx = -origin.X * d;
            double nyy = -origin.Y * d;
            Vector3D ny = new Vector3D(nyx, nyy, nyz);
            return ny;
        }

        /// <summary>
        /// Gets the matrix that transforms a local coordinate system to coordinate system where zero point is located in point <paramref name="origin"/>,
        /// with axis Y directed to North Pole, Z is a radial vector from zero point and X = Y x Z.
        /// </summary>
        /// <param name="origin"></param>
        /// <returns></returns>
        public static Matrix3D GetNPOrientedCS(Point3D origin)
        {
            // Transform from NIP CS to GSK
            Vector3D ny = GetZeroAzimuth(origin);
            Vector3D nz = new Vector3D(origin.X, origin.Y, origin.Z);
            nz.Normalize();
            Vector3D nx = Vector3D.CrossProduct(ny, nz);
            Matrix3D Pe = new Matrix3D( // nip1 -> GSK
                nx.X, nx.Y, nx.Z, 0,
                ny.X, ny.Y, ny.Z, 0,
                nz.X, nz.Y, nz.Z, 0,
                origin.X, origin.Y, origin.Z, 1);
            return Pe;
        }

        /// <summary>
        /// Gets the matrix that transforms local coordinate system of an inclined cone
        /// to coordinate system where the cone is located at the origin and inclined with given azimuth and vertical inclination angle.
        /// </summary>
        /// <param name="origin"></param>
        /// <param name="azimuth">From north, clockwise, in rad.</param>
        /// <param name="inclination">From vertical (OZ), clockwise, in rad.</param>
        /// <returns></returns>
        public static Matrix3D GetInclinedConeCoordinateSystemMatrix(Point3D origin, double azimuth, double inclination)
        {
            var Pe = GetNPOrientedCS(origin); // cs1 -> GSK
            var R = RotateCS(azimuth, inclination);
            Matrix3D Ptotal = R * Pe; // cs2 - > cs1 -> GSK
            return Ptotal;
        }
    }
}
