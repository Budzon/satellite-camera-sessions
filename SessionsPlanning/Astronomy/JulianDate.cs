using System;

namespace Astronomy
{
    public class JulianDate
    {
        long julian_days;

        public JulianDate(long y, long m, long d)
        {
            julian_days = d - 32075 + 1461 * (y + 4800 + (m - 14) / 12) / 4 +
            367 * (m - 2 - ((m - 14) / 12) * 12) / 12 - 3 * ((y + 4900 + (m - 14) / 12) / 100) / 4;
        }

        public JulianDate(long julian_days)
        {
            this.julian_days = julian_days;
        }

        public double GetJD_days(int h, int m, double sec)
        {
            double th_s = 0.001 * sec + 0.06 * m + 3.6 * h;
            return julian_days - 0.5 /*5 / 8*/ + th_s / 86.4;
        }

        // time is returned in 1000 seconds
        public double GetJD_sec(int h, int m, double sec)
        {
            double th_s = 0.001 * sec + 0.06 * m + 3.6 * h;
            return 86.4 * (julian_days - 0.5/*5.0 / 8.0*/) + th_s;
        }

        public long GetDays()
        {
            return julian_days;
        }

        public static DateTime ToDateTime(double julianDate)
        {
            julianDate = julianDate / (24 * 3.6);
            DateTime date;
            double dblA, dblB, dblC, dblD, dblE, dblF;
            double dblZ, dblW, dblX;
            int day, month, year;
            dblZ = Math.Floor(julianDate + 0.5);
            dblW = Math.Floor((dblZ - 1867216.25) / 36524.25);
            dblX = Math.Floor(dblW / 4);
            dblA = dblZ + 1 + dblW - dblX;
            dblB = dblA + 1524;
            dblC = Math.Floor((dblB - 122.1) / 365.25);
            dblD = Math.Floor(365.25 * dblC);
            dblE = Math.Floor((dblB - dblD) / 30.6001);
            dblF = Math.Floor(30.6001 * dblE);
            day = Convert.ToInt32(dblB - dblD - dblF);
            if (dblE > 13)
            {
                month = Convert.ToInt32(dblE - 13);
            }
            else
            {
                month = Convert.ToInt32(dblE - 1);
            }
            if ((month == 1) || (month == 2))
            {
                year = Convert.ToInt32(dblC - 4715);
            }
            else
            {
                year = Convert.ToInt32(dblC - 4716);
            }
            date = new DateTime(year, month, day);

            JulianDate jd = new JulianDate(year, month, day);

            date += TimeSpan.FromSeconds(julianDate * (24 * 3.6) * 1000.0 - jd.GetDays() * 3600 * 24.0);

            return date.AddHours(12);
        }
    }
}