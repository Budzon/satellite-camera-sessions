using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
 
namespace SatelliteSessions
{
    public class Sessions
    {
        public static bool isRequestFeasible(RequestParams request)
        {
            return true;
        }

        public static IList<CaptureConf> getCaptureConfArray(IList<RequestParams> requests)
        {                    
            return new List<CaptureConf>();
        }

        public struct RequestParams
        {
            public int id;
            public int priority;
            public DateTime dateFrom;
            public DateTime dateTo;
            public int Max_SOEN_anlge;
            public double minCoverPerc;
            public int Max_sun_angle;
            public int Min_sun_angle;
            public string wktPolygon; 
        }

        public struct CaptureConf
        {
            public RequestParams request;
            public double coveragePercent;
            public DateTime dateFrom;
            public DateTime dateTo;
            public double rollAngle;
            public double pitchAngle; 
            public string wktPolygon;
        }
    }    
}
