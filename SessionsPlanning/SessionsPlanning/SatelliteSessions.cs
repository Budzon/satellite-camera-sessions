using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using DataParsers;
using SatelliteTrajectory;
using Astronomy;


namespace SatelliteSessions
{
    public class Sessions
    {
        public static bool isRequestFeasible(RequestParams request)
        {
            SatTrajectory trajectory = DatParser.getTrajectoryFromDatFile("trajectory.dat"); // @todo временно
            //double viewAngle = AstronomyMath.ToRad(request.max_roll_angle);  // @todo так будем задавать предел по качеству (если оно будет задаваться максимальным углом крена)
            double viewAngle = Math.PI / 2;  // пока берем полосу максимальной ширины

            SatLane viewLane = trajectory.getCaptureLane(0, viewAngle);            
            List<CaptureConf> confs = viewLane.getCaptureConf(request);

            double summ = 0;
            foreach (var conf in confs)
            {
                foreach (var order in conf.orders)
                {
                    summ += order.intersection_coeff;
                }                
            }
            return (summ >= request.minCoverPerc);
        }

        public static IList<CaptureConf> getCaptureConfArray(IList<RequestParams> requests)
        {
            List <CaptureConf>  captureConfs = new List<CaptureConf>();

            SatTrajectory trajectory = DatParser.getTrajectoryFromDatFile("trajectory.dat"); // @todo временно

            // @todo вынести это в константы
            //double viewAngle = AstronomyMath.ToRad(0.952); // угол обзора камеры
            double viewAngle = AstronomyMath.ToRad(15); // на время тестирования
            double angleStep = viewAngle; // шаг равен углу обзора
            double min_roll_angle = AstronomyMath.ToRad(-45); // @todo пока нету предела по качеству (если оно будет задаваться максимальным углом крена)
            double max_roll_angle = AstronomyMath.ToRad(45);
            for (double dirAngle = min_roll_angle; dirAngle <= max_roll_angle; dirAngle += angleStep)
            {
                SatLane viewLane = trajectory.getCaptureLane(dirAngle, viewAngle);
                foreach (var request in requests)
                {
                    List<CaptureConf> confs = viewLane.getCaptureConf(request);
                    captureConfs.AddRange(confs);
                }                
            }

            return captureConfs;
        }
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

    public class CaptureConf
    {
        public List<Order> orders;
        public DateTime dateFrom;
        public DateTime dateTo;
        public double rollAngle;
        public double pitchAngle;
        public string wktPolygon;

        public CaptureConf()
        {
            orders = new List<Order>();
        }
    }

    public class Order
    {
        public RequestParams request { get; set; }
        public double intersection_coeff { get; set; }

        // ? public double square { get; set; }
    }
}
