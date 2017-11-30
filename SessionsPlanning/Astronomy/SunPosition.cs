using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;

namespace Astronomy
{
    public class SunPosition
    {
        private long julian_days;

        public SunPosition(int y, int m, int d)
        {
            JulianDate jd = new JulianDate(y, m, d);
            julian_days = jd.GetDays();
        }

        /// <summary>
        /// Gets the sun position in ICS. See the AstronomyMath class to convert to GCS.
        /// </summary>
        /// <param name="h"></param>
        /// <param name="m"></param>
        /// <param name="s"></param>
        /// <returns></returns>
        public Point3D Compute(int h, int m, double s)
        {
            double t = (60 * h + m) * 60 + s;
            double grad = Math.PI / 180.0;
            //Old variant
            //double TE = (julian_days + t / 86400 - 2415020 - 5.0 / 8) / 36525;
            double TE = (julian_days + t / 86400 - 2415020 - 0.5) / 36525;
            double angCE = grad * (23.452294 - 0.0130125 * TE);
            double SE = Math.Sin(angCE);
            double CE = Math.Cos(angCE);
            double EE = 0.01675104 - 0.0000418 * TE;
            double angLS = grad * (279.69668 + TE * (36000.76892 + TE * 0.0003025));
            double angMS = grad * (358.475833 + TE * (35999.04975 - TE * 0.000150));
            angLS += 2 * EE * Math.Sin(angMS) + 1.25 * EE * EE * Math.Sin(2 * angMS);
            var X = Math.Cos(angLS);
            double LS = Math.Sin(angLS);
            var Y = CE * LS;
            var Z = SE * LS;
            return new Point3D(X, Y, Z);
        }

        public static Point3D GetPositionGreenwich(DateTime datetime)
        {
            var sp = new SunPosition(datetime.Year, datetime.Month, datetime.Day);
            var p = sp.Compute(datetime.Hour, datetime.Minute, datetime.Second + datetime.Millisecond / 1000.0);
            return AstronomyMath.ICStoGCS(p, AstronomyMath.GetStarTime(datetime));
        }
    }
}
