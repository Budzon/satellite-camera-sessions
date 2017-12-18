using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace OptimalChain
{
    public class CaptureConf
    {
        public DateTime dateFrom { get; set; }
        public DateTime dateTo { get; set; }
        public double tan { get; set; }
        public double rollAngle { get; set; }
        public double pitchAngle { get; set; }

        public string wktPolygon { get; set; }
        public double square { get; set;}
        public List<Order> orders { get; set; }

        public CaptureConf()
        {
            orders = new List<Order>();
        }

        public double reConfigureMilisecinds(CaptureConf s2)
        {
            double a1 = this.rollAngle;
            double b1 = this.tan;
            double a2 = s2.rollAngle;
            double b2 = s2.tan;

            double c_gamma = (1 + Math.Tan(a1)*Math.Tan(a2) + Math.Tan(b1) * Math.Tan(b2)) / (Math.Sqrt(1 + Math.Tan(a1) * Math.Tan(a1) + Math.Tan(b1) * Math.Tan(b1))* Math.Sqrt(1 + Math.Tan(a2) * Math.Tan(a2) + Math.Tan(b2) * Math.Tan(b2)));
            double gamma = Math.Acos(c_gamma);

            if (gamma < Constants.min_degree)
                return Constants.minDeltaT;

            double ms = ((gamma - Constants.min_degree)/Constants.angle_velocity_max)*1000  + Constants.minDeltaT;

            return ms;
        }
    }

    public class RequestParams
    {
        public int id { get; set; }
        public int priority { get; set; }
        public DateTime dateFrom { get; set; }
        public DateTime dateTo { get; set; }
        public int Max_SOEN_anlge { get; set; }
        public double minCoverPerc { get; set; }
        public int Max_sun_angle { get; set; }
        public int Min_sun_angle { get; set; }
        public string wktPolygon { get; set; }
    }

    public class Order
    {
        public RequestParams request { get; set; }
        public double intersection_coeff { get; set; }
    }
}
