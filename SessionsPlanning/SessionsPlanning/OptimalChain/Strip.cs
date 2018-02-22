using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace OptimalChain
{
    public class StaticConf
    {
        public int id;

        public int type { get; set; }// 0-- съемка, 1 -- сброс, 2 -- удаление, 3 -- съемка со сброосом
        public DateTime dateFrom { get; set; }
        public DateTime dateTo { get; set; }

        public double pitch { get; set; }
        public double roll { get; set; }

         public double square { get; set; }//площадь полосы
        public List<Order> orders { get; set; }

        public string wktPolygon { get; set; }
        /// <summary>
        /// Контруктор статической полосы
        /// </summary>
        /// <param name="i"> id конфигурации. Один для одинаковых конфигураций с разными углами</param>
        /// <param name="d1">время начала для съемки </param>
        /// <param name="d2">время конца для съемкир</param>
        /// <param name="t">тангаж</param>
        /// <param name="r">крен</param>
        /// <param name="s">площадь</param>
        /// <param name="o">список заказов</param>
        /// <param name="polygon">полигон в формет WKT</param>
        public StaticConf(int i, DateTime d1, DateTime d2, double t, double r, double s, List<Order> o, string polygon, int T = 0)
        {
            id = i;
            dateFrom = d1;
            dateTo = d2;
            roll = r;
            pitch = t;

            square = s;
            orders = o;
            wktPolygon = polygon;
            type = T;
        }

        public double reConfigureMilisecinds(StaticConf s2)
        {
            double a1 = this.roll;
            double b1 = this.pitch;
            double a2 = s2.roll;
            double b2 = s2.pitch;

            double c_gamma = (1 + Math.Tan(a1)*Math.Tan(a2) + Math.Tan(b1) * Math.Tan(b2)) / (Math.Sqrt(1 + Math.Tan(a1) * Math.Tan(a1) + Math.Tan(b1) * Math.Tan(b1))* Math.Sqrt(1 + Math.Tan(a2) * Math.Tan(a2) + Math.Tan(b2) * Math.Tan(b2)));
            double gamma = Math.Acos(c_gamma);

            if (gamma < Constants.min_degree)
                return Constants.minDeltaT;

            double ms = ((gamma - Constants.min_degree)/Constants.angle_velocity_max)*1000  + Constants.minDeltaT;

            return ms;
        }
    }

    public class CaptureConf
    {
        public int id;
        public int type { get; set; }// 0-- съемка, 1 -- сброс, 2 -- удаление, 3 -- съемка со сброосом
        public DateTime dateFrom { get; set; }//время начала для съемки в надир
        public DateTime dateTo { get; set; }//время окончания для съемки в надир

        public double timeDelta { get; set; }// возможный модуль отклонения по времени от съемки в надир. 

        public Dictionary<double, double> pitchArray { get; set; } //  Массив, ставящий в соответствие упреждение по времени значению угла тангажа

        public double rollAngle { get; set; }//крен для съемки в надир

        public double square { get; set; }//площадь полосы
        public string wktPolygon { get; set; }
        public List<Order> orders { get; set; }

        /// <summary>
        /// Конструктор для создания конфигурации
        /// </summary>
        /// <param name="T">тип конфигурации: 0-- съемка, 1 -- сброс, 2 -- удаление, 3 -- съемка со сброосом</param>
        /// <param name="d1">время начала для съемки в надир</param>
        /// <param name="d2">время конца для съемки в надир</param>
        /// <param name="delta">возможный модуль отклонения по времени от съемки в надир</param>
        /// <param name="r">крен для съемки в надир</param>
        /// <param name="pA">Массив, ставящий в соответствие упреждение по времени значению угла тангажа</param>
        /// <param name="s">площадь полосы</param>
        /// <param name="o">список заказов</param>
        public CaptureConf( DateTime d1, DateTime d2, double delta, double r,  Dictionary<double, double> pA, double s, List<Order> o,int T=0)
        {
            id = -1;
            dateFrom = d1;
            dateTo = d2;
            rollAngle = r;
            pitchArray = pA;

            type = T;
            timeDelta = delta;
            square = s;

            orders = o;

        }

        public CaptureConf()
        {
            orders = new List<Order>();
            pitchArray = new Dictionary<double, double>();
        }

        public StaticConf DefaultStaticConf()
        {
            return new StaticConf(id, dateFrom, dateTo, 0, rollAngle, square, orders, wktPolygon, type);
        }



        public StaticConf CreateStaticConf(int delta, int sign)
        {
            try{
                double r = rollAngle;
                double t = 0.750491578357562;
                DateTime d1 = dateFrom.AddSeconds(delta*sign);
                DateTime d2 = dateTo.AddSeconds(delta * sign);

                return new StaticConf(id, d1, d2, t, r, square, orders, wktPolygon, type);
            }
            catch{
                return null;
            }
            
        }

       
    }

    public class RequestParams
    {
        public int id { get; set; }
        public int priority { get; set; }
        public DateTime dateFrom { get; set; }
        public DateTime dateTo { get; set; }
        public double Max_SOEN_anlge { get; set; }
        public double minCoverPerc { get; set; }
        public int Max_sun_angle { get; set; }
        public int Min_sun_angle { get; set; }
        public string wktPolygon { get; set; }

        /// <summary>
        /// Конструктор параметров заказа
        /// </summary>
        /// <param name="i"> идентификатор</param>
        /// <param name="p"> приоритет</param>
        /// <param name="d1">Дата возможной съемки -- начало</param>
        /// <param name="d2">Дата возможной съемки -- конец</param>
        /// <param name="max_a">Максимально допустимый угол отклонения оси ОСЭН от надира</param>
        /// <param name="min_p">Миимальный допустимый процент покрытия региона заказа</param>
        /// <param name="max_s_a">Максимальный допусмтимый угол солнца над горизонтом</param>
        /// <param name="min_s_a">Минимальный допусмтимый угол солнца над горизонтом</param>
        /// <param name="polygon">Полигон заказа в формае WKT</param>
        public RequestParams(int i, int p, DateTime d1, DateTime d2, int max_a, double min_p, int max_s_a,int min_s_a, string polygon)
        {
            id = i;
            priority = p;
            dateFrom = d1;
            dateTo = d2;
            Max_SOEN_anlge = max_a;
            minCoverPerc = min_p;
            Max_sun_angle = max_s_a;
            Min_sun_angle = min_s_a;
            wktPolygon = polygon;
        }

        public RequestParams() { }
    }

    public class Order
    {
        public RequestParams request { get; set; }
        public SphericalGeom.Polygon captured { get; set; }
        public double intersection_coeff { get; set; }
    }
}
