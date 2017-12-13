using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace SatelliteSessions
{
    public class Sessions 
    {
        public static bool isRequestFeasible(DateTime dateFrom, DateTime dateTo, int Max_SOEN_anlge, double minCoverPerc,
                                             int Max_sun_angle, int Min_sun_angle, string wktPolygon)
        {
            return true;
        }

        public static IList<CaptureConf> getCaptureConfArray(int priority, DateTime dateFrom, DateTime dateTo, int Max_SOEN_anlge,
                                                             double minCoverPerc, int Max_sun_angle, int Min_sun_angle, string wktPolygon)
        {            
            return new List<CaptureConf>();
        }

        public struct CaptureConf
        {
            public DateTime dateFrom;
            public DateTime dateTo;
            public double rollAngle; 
        }
    }    
}
