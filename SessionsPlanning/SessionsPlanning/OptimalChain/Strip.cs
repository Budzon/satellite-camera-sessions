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

        public string shooting_channel { get; set; }// pk, mk, cm

        public int shooting_type { get; set; }//0 -- обычная съемка, 1-- стерео, 2 -- коридорная

        public DateTime dateFrom { get; set; }
        public DateTime dateTo { get; set; }

        /// <summary>
        /// В радианах.
        /// </summary>
        public double pitch { get; set; }
        /// <summary>
        /// В радианах.
        /// </summary>
        public double roll { get; set; }

        public double square { get; set; }//площадь полосы
        public List<Order> orders { get; set; }

        public Tuple<int, int> connected_route { get; set; }//связанные маршруты. Список непустой только для маршрутов на удаление и сброс.

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
        public StaticConf(int i, DateTime d1, DateTime d2, double t, double r, double s, List<Order> o, string polygon, int T = 1, string channel = "pk", int stype = 0, Tuple<int, int> CR = null)
        {
            id = i;
            dateFrom = d1;
            dateTo = d2;
            roll = r;
            pitch = t;

            square = s;
            orders = o;
            connected_route = CR;
            wktPolygon = polygon;
            type = T;
            shooting_type = stype;
            shooting_channel = channel;
        }

        public double reConfigureMilisecinds(StaticConf s2)
        {
            double a1 = this.roll;
            double b1 = this.pitch;
            double a2 = s2.roll;
            double b2 = s2.pitch;

            double c_gamma = (1 + Math.Tan(a1) * Math.Tan(a2) + Math.Tan(b1) * Math.Tan(b2)) / (Math.Sqrt(1 + Math.Tan(a1) * Math.Tan(a1) + Math.Tan(b1) * Math.Tan(b1)) * Math.Sqrt(1 + Math.Tan(a2) * Math.Tan(a2) + Math.Tan(b2) * Math.Tan(b2)));
            double gamma = Math.Acos(c_gamma);

            if (gamma < Constants.min_degree)
                return Constants.minDeltaT;

            double ms = ((gamma - Constants.min_degree) / Constants.angle_velocity_max) * 1000 + Constants.minDeltaT;

            return ms;
        }
    }

    public class CaptureConf
    {
        public int id { get; set; }
        public int confType { get { return mConfType; } }// 0-- съемка, 1 -- сброс, 2 -- удаление, 3 -- съемка со сброосом
        public string shootingChannel { get { return mShootingChannel; } }// pk, mk, cm  - панхроматический канал, многозанальный канал, мультиспектральный
        public int shootingType { get { return mShootingType; } }//0 -- обычная съемка, 1-- стерео, 2 -- коридорная;
        public DateTime dateFrom { get { return mDateFrom; } }//время начала для съемки в надир
        public DateTime dateTo { get { return mDateTo; } }//время окончания для съемки в надир       
        public double rollAngle { get { return mRollAngle; } }//крен для съемки c нулевым тангажом
        public double square { get { return mSquare; } }//площадь полосы
        public string wktPolygon { get { return mWktPolygon; } } //полигон съемки, который захватывается этой конфигураций. Непуст только для маршрутов на съемку и съемку со сбросом.
        public List<Order> orders { get { return mOrders; } }//cвязанные заказы. Список пуст только для маршрута на удаление
        public Tuple<int, int> connectedRoute { get { return mConnectedRoute; } }//связанные маршруты. Список непустой только для маршрутов на удаление и сброс.
        
        public double timeDelta { get; set; }// возможный модуль отклонения по времени от съемки в надир. 
        public Dictionary<double, double> pitchArray { get; set; } //  Массив, ставящий в соответствие упреждение по времени значению угла тангажа        

        private List<Order> mOrders;
        private double mSquare;
        private string mWktPolygon;
        private string mShootingChannel;
        private int mShootingType;
        private int mConfType;
        private Tuple<int, int> mConnectedRoute;
        private DateTime mDateFrom;
        private DateTime mDateTo; 
        private double mRollAngle;

        public void setPolygon(SphericalGeom.Polygon pol)
        {
            mSquare = pol.Area;
            mWktPolygon = pol.ToWtk();
        }

        public void setPitchDependency(Dictionary<double, double> _pitchArray, double _timeDelta)
        {
            pitchArray = _pitchArray;
            timeDelta = _timeDelta;
        }

        /// <summary>
        /// Конструктор для создания конфигурации
        /// </summary>        
        /// <param name="_dateFrom">время начала для съемки в надир</param>
        /// <param name="_dateTo">время конца для съемки в надир</param> 
        /// <param name="_rollAngle">крен для съемки c нулевым тангажом</param>  
        /// <param name="_orders">список заказов</param>
        /// <param name="_confType">тип конфигурации: 0-- съемка, 1 -- сброс, 2 -- удаление, 3 -- съемка со сброосом</param>        
        /// <param name="_connectedRoute">связанные мрашруты</param>
        public CaptureConf(
            DateTime _dateFrom,
            DateTime _dateTo,
            double _rollAngle,
            List<Order> _orders,
            int _confType,
            Tuple<int, int> _connectedRoute)
        {
            if (_orders == null)
                throw new ArgumentException("Orders array can not be empty");

            if (_orders.Count == 0)
                throw new ArgumentException("Orders array can not be empty");

            mShootingChannel = _orders[0].request.requestChannel;
            mShootingType = _orders[0].request.shootingType;

            foreach (var order in _orders)
            {
                if (order.request.shootingType != shootingType
                  || order.request.requestChannel != shootingChannel)
                    throw new ArgumentException("Orders array can not contain orders with different channels or types.");
            }

            id = -1;
            mOrders = _orders;
            mDateFrom = _dateFrom;
            mDateTo = _dateTo;
            mRollAngle = _rollAngle;
            mConfType = _confType;
            mConnectedRoute = _connectedRoute;
            pitchArray = new Dictionary<double, double>();
            timeDelta = 0;
        }


        public StaticConf DefaultStaticConf()
        {
            return new StaticConf(id, dateFrom, dateTo, 0, rollAngle, square, orders, wktPolygon, confType, shootingChannel, shootingType, connectedRoute);
        }

        public StaticConf CreateStaticConf(int delta, int sign)
        {
            try
            {
                //double pitch = pitchArray[delta];
                double pitch = pitchArray[0];

                var h = 720.330932208252;  // высота траектории, км.
                var w = 7.2921158533E-05;  // скорость вращения земли в радианах
                var b = 0.00225708715578222;  // широта подспутниковой точки в радианах
                var v = 0.0010139813837136; // скорость подспутниковой точки в радианах

                double I = 1.7104; // наклонение орбиты в радианах. OptimalChain.Constants.orbital_inclination;
                double R = 6371.3; // радиус в км.  Astronomy.Constants.EarthRadius;
                double bm = b + Math.Sin(I) * (Math.Acos(Math.Sqrt(1 - Math.Pow((R + h) / R * Math.Sin(pitch), 2))) - pitch);

                //Разница между двумя позициями спутника
                double b2 = Math.Acos(Math.Sqrt(1 - Math.Pow((R + h) / R * Math.Sin(pitch), 2))) - pitch;
                
                double d = Math.Cos(bm) * w / v * b2 * Math.Sin(I);
                double sinRoll = R * Math.Sin(d) / Math.Sqrt(Math.Pow(R, 2) + Math.Pow(R + h, 2) - 2 * R * (R + h) * Math.Cos(d));

                double r = Math.Asin(sinRoll); ;

                DateTime d1 = dateFrom.AddSeconds(delta * sign);
                DateTime d2 = dateTo.AddSeconds(delta * sign);

                return new StaticConf(id, d1, d2, pitch, r, square, orders, wktPolygon, confType, shootingChannel, shootingType, connectedRoute);
            }
            catch
            {
                return null;
            }

        }


        public static CaptureConf unitCaptureConfs(CaptureConf confs1, CaptureConf confs2)
        {
            if (confs1.shootingType != confs2.shootingType ||
                confs1.shootingChannel != confs2.shootingChannel ||
                confs1.confType != confs2.confType)
                throw new ArgumentException("it is impossible to unite confs with different channels or types.");

            var dateFrom = (confs1.dateFrom < confs2.dateFrom) ? confs1.dateFrom : confs2.dateFrom;
            var dateTo = (confs1.dateTo > confs2.dateTo) ? confs1.dateTo : confs2.dateTo;
            var orders = new List<Order>();
            orders.AddRange(confs1.orders);
            orders.AddRange(confs2.orders);
            Tuple<int, int> newConnectedRoute = confs1.connectedRoute;
            CaptureConf newConf = new CaptureConf(dateFrom, dateTo, confs1.rollAngle, orders, confs1.confType, newConnectedRoute);
            
            /*
            for (int i = 0; i < confs1.orders.Count; i++)
            {
                newConf.orders.Add(confs1.orders[i]);
                for (int j = 0; j < confs2.orders.Count; j++)
                {
                    if (confs1.orders[i].request.id != confs2.orders[j].request.id)
                    {                       
                        newConf.orders.Add(confs2.orders[j]);                        
                    }
                    else
                    {
                        var order = new Order();
                        
                    }                    
                }
            }*/

            return newConf;
        }

        public static bool isNeedUnit(CaptureConf c1, CaptureConf c2)
        {
            /// @todo добавить минимально допустимое расстояние (по времени)
            return ((c1.dateFrom <= c2.dateTo && c2.dateTo <= c1.dateTo) || (c1.dateFrom <= c2.dateFrom && c2.dateFrom <= c1.dateTo)
                  || (c2.dateFrom <= c1.dateTo && c1.dateTo <= c2.dateTo) || (c2.dateFrom <= c1.dateFrom && c1.dateFrom <= c2.dateTo));
        }

        public static void compressCConfArray(ref List<CaptureConf> confs)
        {
            for (int i = 0; i < confs.Count; i++)
            {
                for (int j = i + 1; j < confs.Count; j++)
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

        public static void compressTwoCConfArrays(ref List<CaptureConf> confs1, ref List<CaptureConf> confs2)
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

    public class RequestParams
    {
        public int id { get; set; }
        public int priority { get; set; }
        public DateTime timeFrom { get; set; }
        public DateTime timeTo { get; set; }
        public double Max_SOEN_anlge { get; set; }
        public double minCoverPerc { get; set; }
        public int Max_sun_angle { get; set; }
        public int Min_sun_angle { get; set; }
        public string wktPolygon { get; set; }
        public string requestChannel { get; set; }
        public int shootingType { get; set; }

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
        public RequestParams(int i, int p, DateTime d1, DateTime d2, int max_a, double min_p, int max_s_a, int min_s_a, string polygon)
        {
            id = i;
            priority = p;
            timeFrom = d1;
            timeTo = d2;
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
