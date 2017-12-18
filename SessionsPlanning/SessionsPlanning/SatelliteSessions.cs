using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using DataParsers;
using SatelliteTrajectory;
using Astronomy;
using OptimalChain;

namespace SatelliteSessions
{
    public class Sessions
    {
        public static bool isRequestFeasible(RequestParams request)
        {
            string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "trajectory_1day.dat";
            SatTrajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName); // @todo временно 
            //double viewAngle = AstronomyMath.ToRad(request.max_roll_angle);  // @todo так будем задавать предел по качеству (если оно будет задаваться максимальным углом крена) 
            double viewAngle = Math.PI / 2;  // пока берем полосу максимальной ширины 

            SatLane viewLane = trajectory.getCaptureLane(0, viewAngle); 
            List<CaptureConf> confs = viewLane.getCaptureConfs(request); 
            
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
            List<CaptureConf> captureConfs = new List<CaptureConf>();

            string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "trajectory_1day.dat";
            SatTrajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName); // @todo временно
            // @todo вынести это в константы
            double viewAngle = AstronomyMath.ToRad(0.952); // угол обзора камеры
            //double viewAngle = AstronomyMath.ToRad(15); // на время тестирования
            double angleStep = viewAngle; // шаг равен углу обзора
            double min_roll_angle = AstronomyMath.ToRad(-45); // @todo пока нету предела по углу крена
            double max_roll_angle = AstronomyMath.ToRad(45);
            for (double rollAngle = min_roll_angle; rollAngle <= max_roll_angle; rollAngle += angleStep)
            {
                List<CaptureConf> laneCaptureConfs = new List<CaptureConf>(); // участки захвата для текущий линии захвата
                SatLane viewLane = trajectory.getCaptureLane(rollAngle, viewAngle);
                foreach (var request in requests)
                {
                    List<CaptureConf> confs = viewLane.getCaptureConfs(request);
                    compressCConfArray(ref confs);
                    compressTwoCConfArrays(ref laneCaptureConfs, ref confs);                    
                    laneCaptureConfs.AddRange(confs);                        
                }

                foreach (var conf in laneCaptureConfs)
                {
                    conf.wktPolygon = viewLane.getSegment(conf.dateFrom, conf.dateTo).ToWtk();
                    conf.rollAngle = rollAngle;
                    conf.pitchAngle = 0;
                }

                captureConfs.AddRange(laneCaptureConfs);
            }

            Graph g = new Graph(captureConfs);
            List<CaptureConf> optimalChain = g.findOptimalChain();
            return optimalChain;
        }

        //public static IList<CaptureConf> getOptimalChain(List<CaptureConf> strips)
        //{
        //    Graph g = new Graph(strips);
        //    List<CaptureConf> optimalChain = g.findOptimalChain();         
        //    return optimalChain;
        //}

        private static CaptureConf unitCaptureConfs(CaptureConf confs1, CaptureConf confs2)
        {
            CaptureConf newConf = new CaptureConf();
            newConf.pitchAngle = confs1.pitchAngle;
            newConf.rollAngle = confs1.rollAngle;
            newConf.dateFrom = (confs1.dateFrom < confs2.dateFrom) ? confs1.dateFrom : confs2.dateFrom;
            newConf.dateTo = (confs1.dateTo > confs2.dateTo) ? confs1.dateTo : confs2.dateTo;            
            newConf.orders.AddRange(confs1.orders);
            newConf.orders.AddRange(confs2.orders);
            return newConf;
        }

        private static bool isNeedUnit(CaptureConf confs1, CaptureConf confs2)
        {
            /// @todo добавить минимально допустимое расстояние (по времени)
            return ((confs1.dateFrom < confs2.dateTo && confs1.dateFrom > confs2.dateFrom)
                   || (confs1.dateFrom < confs2.dateTo && confs1.dateFrom > confs2.dateFrom));
        }

        private static void compressCConfArray(ref List<CaptureConf> confs)
        {
            for (int i = 0; i < confs.Count; i++)
            {
                for (int j = i; j < confs.Count; j++)
                {
                    if (isNeedUnit(confs[i], confs[j]))
                    {
                        CaptureConf comConf = unitCaptureConfs(confs[i], confs[j]);
                        confs[i] = comConf;
                        confs.RemoveAt(j);
                    }
                }
            }
        }


        private static void compressTwoCConfArrays(ref List<CaptureConf> confs1, ref List<CaptureConf> confs2)
        {
            for (int i = 0; i < confs1.Count; i++)
            {
                for (int j = 0; j < confs2.Count; j++)
                {
                    if (isNeedUnit(confs1[i], confs2[j]))
                    {
                        CaptureConf unitConf = unitCaptureConfs(confs1[i], confs2[j]);
                        confs1[i] = unitConf;
                        confs2.RemoveAt(j);
                    }
                }
            }             
        }
        
    }

    /*
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
    */
}
