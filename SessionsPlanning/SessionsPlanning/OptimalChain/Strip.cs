using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using System.Linq;
using Astronomy;
using Common;

using SphericalGeom;
using SessionsPlanning;

namespace OptimalChain
{
    public class StaticConf
    {
        public int id;

        public WorkingType type { get; set; } // тип работы

        public ShootingChannel shooting_channel { get; set; } // канал

        public ShootingType shooting_type { get; set; } // тип съемки

        public DateTime dateFrom { get; set; }
        public DateTime dateTo { get; set; }
        public SatelliteSessions.PolinomCoef poliCoef { get; set; }

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

        public int MinCompression { get; set; }
        public double AverAlbedo { get; set; }

        public RouteAddress connected_route { get; set; }//связанные маршруты. Список непустой только для маршрутов на удаление и сброс.

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

        public StaticConf(
            int i,
            DateTime d1,
            DateTime d2,
            double t,
            double r,
            double s,
            List<Order> o,
            string polygon,
            int comp,
            double alb,
            WorkingType T = WorkingType.Downloading,
            ShootingChannel channel = ShootingChannel.pk,
            ShootingType stype = ShootingType.Normal,
            RouteAddress CR = null,
            SatelliteSessions.PolinomCoef _poliCoef = null)
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
            AverAlbedo = alb;
            MinCompression = comp;
            poliCoef = _poliCoef;
        }

        public double reConfigureMilisecinds(StaticConf s2)
        {
            return reConfigureMilisecinds(this.roll, this.pitch, s2.roll, s2.pitch);           
        }
        
        public static double reConfigureMilisecinds(double r1, double p1,  double r2,  double p2)
        { 
            double c_gamma = (1 + Math.Tan(r1) * Math.Tan(r2) + Math.Tan(p1) * Math.Tan(p2)) 
                / (Math.Sqrt(1 + Math.Tan(r1) * Math.Tan(r1) + Math.Tan(p1) * Math.Tan(p1))
                   *Math.Sqrt(1 + Math.Tan(r2) * Math.Tan(r2) + Math.Tan(p2) * Math.Tan(p2)));
            double gamma = Math.Acos(c_gamma);

            if (gamma < Constants.min_degree || double.IsNaN(gamma))
                return Constants.minDeltaT;

            double ms = ((gamma - Constants.min_degree) / Constants.angle_velocity_max) * 1000 + Constants.minDeltaT;

            return ms;
        }

    }

 
    public class CaptureConf : SatelliteSessions.TimePeriod
    {
        public int id { get; set; }
        public WorkingType confType { get; set; } // 0— съемка, 1 — сброс, 2 -- удаление, 3 -- съемка со сброосом
        public ShootingChannel shootingChannel {  get; private set; } // канал 
        public ShootingType shootingType {  get; private set; } // тип съемки
        public double rollAngle {  get; private set; }//крен для съемки c нулевым тангажом
        public double square { get; private set; }//площадь полосы
        public string wktPolygon {  get; private set; } //полигон съемки, который захватывается этой конфигураций. Непуст только для маршрутов на съемку и съемку со сбросом.
        public List<Order> orders { get; private set; } //cвязанные заказы. Список пуст только для маршрута на удаление
        public RouteAddress connectedRoute { get; private set; }//связанные маршруты. Список непустой только для маршрутов на удаление и сброс.
        public double timeDelta { get; private set;}// возможный модуль отклонения по времени от съемки в надир. 
        public Dictionary<double, Tuple<double, double>> pitchArray {  get; private set; } //  Массив, ставящий в соответствие упреждение по времени значению угла тангажа и упреждение по крену      
        public int MinCompression {  get; private set; }
        public double AverAlbedo {  get; private set;}
        public double SunAngle { get; private set; } // угол солнца снимаемой сцены
        public SatelliteSessions.PolinomCoef poliCoef { get; private set; }

        public void setPolygon(SphericalGeom.Polygon pol)
        {
            square = pol.Area;
            wktPolygon = pol.ToWtk();
        }

        /// <summary>
        /// рассчитаем угол солнца для центра кадра
        /// </summary>
        /// <param name="sunPos">координаты солнца относительно центра земли</param>
        /// <param name="centrePos"> координаты центра кадра </param>
        public void setSun(Vector3D sunPos, Vector3D centrePos)
        {
            Vector3D toSun = sunPos - centrePos; // вектор на солнце от центра кадра
            SunAngle = Math.PI/2 - Vector3D.AngleBetween(toSun, centrePos);
        }
        
        public void setPitchDependency(Dictionary<double, Tuple<double, double>> _pitchArray, double _timeDelta)
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
        /// <param name="_confType">тип конфигурации</param>        
        /// <param name="_connectedRoute">связанные мрашруты</param>
        public CaptureConf(
            DateTime _dateFrom,
            DateTime _dateTo,
            double _rollAngle,
            List<Order> _orders,
            WorkingType _confType,
            RouteAddress _connectedRoute,
            SatelliteSessions.PolinomCoef _poliCoef  = null)
            : base(_dateFrom, _dateTo)
        {
            if (_dateFrom > _dateTo)
                throw new ArgumentException("Incorrect time interval");

            if (_orders == null)
                throw new ArgumentException("Orders array can not be empty");

            if (_orders.Count == 0)
                throw new ArgumentException("Orders array can not be empty");

            shootingChannel = _orders[0].request.requestChannel;
            shootingType = _orders[0].request.shootingType;

            foreach (var order in _orders)
            {
                if (order.request.shootingType != shootingType
                  || order.request.requestChannel != shootingChannel)
                    throw new ArgumentException("Orders array can not contain orders with different channels or types.");
            }

            id = -1;
            confType = _confType;
            orders = _orders;
            rollAngle = _rollAngle;            
            connectedRoute = _connectedRoute;
            pitchArray = new Dictionary<double, Tuple<double, double>>();
            timeDelta = 0;
            MinCompression = orders.Min(order => order.request.compression);
            AverAlbedo = orders.Average(order => order.request.albedo);
            poliCoef = _poliCoef;
        }


        public StaticConf DefaultStaticConf()
        {
            return new StaticConf(id, dateFrom, dateTo, 0, rollAngle, square, orders, wktPolygon, MinCompression, AverAlbedo, confType, shootingChannel, shootingType, connectedRoute, poliCoef);
        }

        public StaticConf CreateStaticConf(double delta, int sign)
        {
            try
            {
                
                double p =  pitchArray[delta].Item1;
                double r =  pitchArray[delta].Item2;

                if((confType == WorkingType.Shooting) && (shootingType != ShootingType.StereoTriplet))
                {
                    p = p*sign;
                 //   r = r * sign;
                }
                DateTime d1 = dateFrom.AddSeconds(delta * sign);
                DateTime d2 = dateTo.AddSeconds(delta * sign);

                return new StaticConf(id, d1, d2, p, r, square, orders, wktPolygon, MinCompression, AverAlbedo, confType, shootingChannel, shootingType, connectedRoute, poliCoef);
            }
            catch
            {
                return null;
            }

        }


        /// <summary>
        /// объеднить (по времени) 
        /// </summary>
        /// <param name="confs"></param>
        public static List<CaptureConf> compressCConfArray(List<CaptureConf> confs)
        {
            if (confs.Count < 1)
                return confs;

            var res = SatelliteSessions.TimePeriod.compressTimePeriods<CaptureConf>(confs, OptimalChain.Constants.maxCConfInterval);

            return res;
        }

        public override SatelliteSessions.TimePeriod Unite(SatelliteSessions.TimePeriod period)    
        {
            CaptureConf confs2 = (CaptureConf)period;
            if (shootingType != confs2.shootingType ||
                shootingChannel != confs2.shootingChannel ||
                confType != confs2.confType)
                throw new ArgumentException("it is impossible to unite confs with different channels or types.");

            var newDateFrom = (dateFrom < confs2.dateFrom) ? dateFrom : confs2.dateFrom;
            var newDateTo = (dateTo > confs2.dateTo) ? dateTo : confs2.dateTo;
            var orders = new List<Order>();
            orders.AddRange(orders);
            orders.AddRange(confs2.orders);            
            CaptureConf newConf = new CaptureConf(newDateFrom, newDateTo, rollAngle, orders, confType, connectedRoute);

            return newConf;
        }


        /// <summary>
        /// Создать стереотриплет из текущей конфигурации
        /// </summary>
        /// <param name="pointFrom">положение КА в момент съемки</param>
        /// <param name="availableRanges">доступные для съемки интервалы времени</param>
        /// <returns>false, если создание не удалось </returns>
        public bool converToStereo(Astronomy.TrajectoryPoint pointFrom, List<SatelliteSessions.TimePeriod> availableRanges, ShootingType type)
        {
            double pitchAngle = OptimalChain.Constants.stereoPitchAngle;
            double deflectTimeDelta = getTimeDeltaFromPitch(pointFrom, this.rollAngle, pitchAngle);
            DateTime dtFrom = this.dateFrom.AddSeconds(-deflectTimeDelta);
            DateTime dtTo = this.dateTo.AddSeconds(deflectTimeDelta);

            if ((this.dateTo - this.dateFrom).TotalSeconds > deflectTimeDelta)
                return false; // полоса слишком длинная. Мы не успеваем отснять с углом -30 до того, как начнём снимать с углом 0

            if (!SatelliteSessions.TimePeriod.isPeriodInPeriods(new SatelliteSessions.TimePeriod(dtFrom, dtTo), availableRanges))
                return false;  // мы не попадаем в разрешенные промежутки времени

            Dictionary<double, Tuple<double, double>> timeAngleArray = new Dictionary<double, Tuple<double, double>>();

            timeAngleArray[-deflectTimeDelta] = Tuple.Create(pitchAngle, 0.0);
            if (ShootingType.StereoTriplet == type)
                timeAngleArray[0] = Tuple.Create(0.0, 0.0);            
            timeAngleArray[deflectTimeDelta] = Tuple.Create(-pitchAngle, 0.0);

            this.setPitchDependency(timeAngleArray, deflectTimeDelta);

            return true;
        }
         
        /// <summary>
        /// расчёт поправки по крену
        /// </summary>
        /// <param name="height">высота ка в км</param>
        /// <param name="velo">скорость подспутниковой точки в радианах</param>
        /// <param name="bKa">широта подспутниковой точки в радианах </param>
        /// <param name="pitchAngle">угол тангажа</param>
        /// <returns>поправка по крену</returns>
        public static double getRollCorrection(double height, double velo, double bKa, double pitch)
        {
            double wEarth = OptimalChain.Constants.earthRotSpeed;
            double I = OptimalChain.Constants.orbital_inclination;
            double R = Astronomy.Constants.EarthRadius;
            double bm = bKa + Math.Sin(I) * (Math.Acos(Math.Sqrt(1 - Math.Pow((R + height) / R * Math.Sin(pitch), 2))) - pitch);
            //Разница между двумя позициями спутника
            double b2 = Math.Acos(Math.Sqrt(1 - Math.Pow((R + height) / R * Math.Sin(pitch), 2))) - Math.Abs(pitch);
            double d = Math.Cos(bm) * wEarth / velo * b2 * Math.Sin(I);
            double sinRoll = R * Math.Sin(d) / Math.Sqrt(Math.Pow(R, 2) + Math.Pow(R + height, 2) - 2 * R * (R + height) * Math.Cos(d));
            return Math.Asin(sinRoll);
        }
        
        public static double getTimeDeltaFromPitch(Astronomy.TrajectoryPoint pointFrom, double rollAngle, double pitchAngle)
        {
            Vector3D rollPoint = SatelliteTrajectory.LanePos.getSurfacePoint(pointFrom, rollAngle, 0);
            Vector3D PitchRollPoint = SatelliteTrajectory.LanePos.getSurfacePoint(pointFrom, rollAngle, pitchAngle);
            rollPoint.Normalize();
            PitchRollPoint.Normalize();
            // расстояние в километрах между точкой c нулевым тангажом и точкой, полученной при максимальном угле тангажа
            double dist = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(rollPoint), GeoPoint.FromCartesian(PitchRollPoint)) * Astronomy.Constants.EarthRadius;
            // время, за которое спутник преодалевает dist по поверхности земли.
            return Math.Abs(dist / pointFrom.Velocity.Length);
        }
        
        public void calculatePitchArrays(Astronomy.TrajectoryPoint pointFrom)
        {
            double pitchAngleLimit = orders.Min(order => order.request.Max_SOEN_anlge);

            if (pitchAngleLimit > OptimalChain.Constants.max_pitch_angle)
                pitchAngleLimit = OptimalChain.Constants.max_pitch_angle;

            double maxPitchAngle = Math.Abs(pitchAngleLimit) - Math.Abs(rollAngle);

            if (maxPitchAngle < 0) // если rollAngle больше (по модулю) 30 градусов (максимальны тангаж)
                maxPitchAngle = 0;

            double timeDelta;
            if (0 == maxPitchAngle)
                timeDelta = 0;
            else
                timeDelta = getTimeDeltaFromPitch(pointFrom, rollAngle, maxPitchAngle);

            pitchArray[0] = Tuple.Create(0.0, 0.0);

            Dictionary<double, double> angleTimeArray = new Dictionary<double, double>();
            angleTimeArray[0] = 0;

            Vector3D dirRollPoint = SatelliteTrajectory.LanePos.getSurfacePoint(pointFrom, rollAngle, 0);
            int pitchStep = 1; // угол изменения тангажа в градусах.
           
            for (int pitch_degr = pitchStep; pitch_degr <= AstronomyMath.ToDegrees(maxPitchAngle); pitch_degr += pitchStep)
            { 
                double pitch = AstronomyMath.ToRad(pitch_degr);
                Vector3D dirPitchPoint = SatelliteTrajectory.LanePos.getSurfacePoint(pointFrom, rollAngle, pitch);
                double distOverSurf = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(dirPitchPoint), GeoPoint.FromCartesian(dirRollPoint)) * Astronomy.Constants.EarthRadius;
                double t = distOverSurf / pointFrom.Velocity.Length;
                angleTimeArray[pitch] = t;
            }

            LinearInterpolation pitchInterpolation = new LinearInterpolation(angleTimeArray.Values.ToArray(), angleTimeArray.Keys.ToArray());

            Dictionary<double, Tuple<double, double>> timeAngleArray = new Dictionary<double, Tuple<double, double>>();
            for (int t = 0; t <= (int)timeDelta; t++)
            {
                double pitch = pitchInterpolation.GetValue(t);
                double height = pointFrom.Position.ToVector().Length - Astronomy.Constants.EarthRadius;
                double velo = pointFrom.Velocity.Length / pointFrom.Position.ToVector().Length;
                GeoPoint kaGeoPoint = GeoPoint.FromCartesian(pointFrom.Position.ToVector());
                var rollCorrection = getRollCorrection(height, velo, AstronomyMath.ToRad(kaGeoPoint.Latitude), pitch);
                timeAngleArray[t] = Tuple.Create(pitch, rollCorrection);
            }

            setPitchDependency(timeAngleArray, timeDelta);
        }


    }

    public class RequestParams
    {
        public int id { get; private set; }
        public int priority { get; private set; }
        public DateTime timeFrom { get; private set; }
        public DateTime timeTo { get; private set; }
        public double Max_SOEN_anlge { get; private set; }
        public double minCoverPerc { get; private set; }
        public int Max_sun_angle { get; private set; }
        public int Min_sun_angle { get; private set; }
        public string wktPolygon { get; private set; }
        public List<string> polygonToSubtract { get; private set; }
        public ShootingChannel requestChannel { get; private set; } // канал
        public ShootingType shootingType { get; private set; } // тип съемки заказа
        public int compression { get; private set; } // коэффициент сжатия заказа 0 - сжатие без потерь, 1 - без сжатия, 2-10 - сжатие с потерями
        public double albedo { get; private set; } //  характеристика отражательной способности поверхности. 
        public List<Polygon> polygons { get; private set; } // полигоны, которые необходимо покрыть в рамках этого заказа   
        public double Square { get; private set; }
        public double Cloudiness { get; private set; }
        /// <summary>
        /// разделим заказы на группы по признаку совместимых CaptureConf-ов
        /// </summary>
        /// <param name="requests"> несортированные заказы, все вместе </param>
        /// <returns> разделённые заказы </returns>
        public static List<List<RequestParams>> breakRequestsIntoGroups(List<RequestParams> requests)
        {
            var breakingRequests = new Dictionary<Tuple<ShootingType, ShootingChannel, bool>, List<RequestParams>>();

            foreach(var request in requests)
            {
                Tuple<ShootingType, ShootingChannel, bool> key = Tuple.Create(request.shootingType, request.requestChannel, request.compression == OptimalChain.Constants.compressionDropCapture);
                if (!breakingRequests.ContainsKey(key))
                    breakingRequests[key] = new List<RequestParams>();

                breakingRequests[key].Add(request);
            }
            return breakingRequests.Values.ToList();
        }

       
        /// <summary>
        /// Конструктор параметров заказа
        /// </summary>
        /// <param name="_id"> идентификатор</param>
        /// <param name="_priority"> приоритет</param>
        /// <param name="_timeFrom">Дата возможной съемки -- начало</param>
        /// <param name="_timeTo">Дата возможной съемки -- конец</param>
        /// <param name="_Max_SOEN_anlge">Максимально допустимый угол отклонения оси ОСЭН от надира</param>
        /// <param name="_minCoverPerc">Миимальный допустимый процент покрытия региона заказа</param>
        /// <param name="_Max_sun_angle">Максимальный допусмтимый угол солнца над горизонтом</param>
        /// <param name="_Min_sun_angle">Минимальный допусмтимый угол солнца над горизонтом</param>
        /// <param name="_wktPolygon">Полигон заказа в формае WKT</param>
        /// <param name="_albedo">  характеристика отражательной способности поверхности. </param>
        ///  <param name="_shootingType"> тип съемка </param>
        /// <param name="_compression"> коэффициент сжатия заказа 0 - сжатие без потерь, 1 - без сжатия, 2-10 - сжатие с потерями</param>
        public RequestParams(int _id,
            int _priority,
            DateTime _timeFrom,
            DateTime _timeTo,
            double _Max_SOEN_anlge,
            double _minCoverPerc,
            int _Max_sun_angle, 
            int _Min_sun_angle,
            string _wktPolygon,
            List<string> _polygonToSubtract = null,
            double _albedo = 0.36,
            int _compression = 0,
            ShootingType _shootingType = ShootingType.Normal,
            ShootingChannel _requestChannel = ShootingChannel.mk,
            double _cloudiness = 0
            )
        {
            id = _id;
            priority = _priority;
            timeFrom = _timeFrom;
            timeTo = _timeTo;
            Max_SOEN_anlge = _Max_SOEN_anlge;
            minCoverPerc = _minCoverPerc;
            Max_sun_angle = _Max_sun_angle;
            Min_sun_angle = _Min_sun_angle;
            wktPolygon = _wktPolygon;
            albedo = _albedo;
            compression = _compression;            
            shootingType = _shootingType;
            requestChannel = _requestChannel;
            Cloudiness = _cloudiness;
            polygonToSubtract = _polygonToSubtract;
            Polygon comPolygon = new Polygon(wktPolygon);
            if (_polygonToSubtract != null)
            {
                Tuple<List<Polygon>, List<Polygon>> res = Polygon.IntersectAndSubtract(comPolygon, _polygonToSubtract.Select(str => new Polygon(str)).ToList());
                polygons = res.Item2;
            }
            else
            {
                polygons = new List<Polygon>() { comPolygon };
            }

            Square = polygons.Sum(pol => pol.Area);
        }

        public RequestParams(RequestParams copyed)
        {
            id = copyed.id;
            priority = copyed.priority;
            timeFrom = copyed.timeFrom;
            timeTo = copyed.timeTo;
            Max_SOEN_anlge = copyed.Max_SOEN_anlge;
            minCoverPerc = copyed.minCoverPerc;
            Max_sun_angle = copyed.Max_sun_angle;
            Min_sun_angle = copyed.Min_sun_angle;
            wktPolygon = copyed.wktPolygon;
            albedo = copyed.albedo;
            compression = copyed.compression;
            shootingType = copyed.shootingType;
            requestChannel = copyed.requestChannel;
            polygonToSubtract = copyed.polygonToSubtract; 
            polygons = copyed.polygons;            
        }
         
    }

    public class Order
    {
        /// <summary>
        /// параметры заказа
        /// </summary>
        public RequestParams request { get; set; }

        /// <summary>
        /// захваченная (заснятая) область
        /// </summary>
        public SphericalGeom.Polygon captured { get; set; }
        public double intersection_coeff { get; set; }
    }
}