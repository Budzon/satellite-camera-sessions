using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SessionsPlanning;
using System.Collections.ObjectModel;
using System.Collections.Specialized;

namespace OptimalChain
{

    public struct FormattingOptions
    { 
        public bool Bank1Cell1 {get;set;}
        public bool Bank1Cell2 {get;set;}
        public bool Bank1Cell3 {get;set;}
        public bool Bank2Cell1 {get;set;}
        public bool Bank2Cell2 {get;set;}
        public bool Bank2Cell3 {get;set;}

        public bool Bank1 { get { return Bank1Cell1 || Bank1Cell2 || Bank1Cell3; } }
        public bool Bank2 { get { return Bank2Cell1 || Bank2Cell2 || Bank2Cell3; } }
    };

    public class MPZParams
    {
        public int id { get; set; }
        public bool PWR_ON { get; set; }
        public DateTime start { get; set; }
        public DateTime end { get; set; }
        public int N_routes { get { return routes.Count; } }
        public bool is_reserve_conf { get { return false; } }
       // public List<RouteParams> routes { get; set; }
        public CommunicationSessionStation? Station { get; set; }
        public ObservableCollection<RouteParams> routes { get; set; }

        public MPZParams(MPZParams copyed)
        {
            id = copyed.id;
            PWR_ON = copyed.PWR_ON;
            start = copyed.start;
            end = copyed.end; 
            routes = new ObservableCollection<RouteParams>(copyed.routes.Select(r => new RouteParams(r)));
            routes.CollectionChanged += renumerateRoutes;
            Station = copyed.Station;
        }

        public MPZParams(int ID)
        {
            id = ID;
            PWR_ON = false;
            routes = new ObservableCollection<RouteParams>();
            routes.CollectionChanged += renumerateRoutes;
        }

        public MPZParams(int ID, RouteParams r) 
            : this(ID)
        {                       
            routes.Add(r);            
            start = r.start.AddMilliseconds(-Constants.SOEN_turning_on_Time);
            end = r.end.AddMilliseconds(Constants.MPZ_ending_Time);            
        }

        /// <summary>
        /// Конструктор МПЗ на основе готовых маршрутов (без пересчёта времён)
        /// </summary>
        /// <param name="ID">NPZ, номер МПЗ</param>
        /// <param name="routesList">массив маршрутов</param>
        /// <param name="PWR_ON"></param>
        public MPZParams(int ID, List<RouteParams> routesList, bool PWR_ON, CommunicationSessionStation mpzStation)
            : this(ID)
        {            
            routes = new ObservableCollection<RouteParams>(routesList);
            start = routesList.First().start.AddMilliseconds(-Constants.MPZ_starting_Time);
            end = routesList.Last().end.AddMilliseconds(Constants.MPZ_ending_Time);
            Station = mpzStation;
        }
        
      
        public bool AddRoute(RouteParams r)
        {
            if (N_routes > 11)
                return false;

            routes.Add(r);            

            if (N_routes < 2)
                start = r.start.AddMilliseconds(-Constants.SOEN_turning_on_Time);

            end = r.end.AddMilliseconds(Constants.MPZ_ending_Time);
            return true;
        }

        public RouteParams GetLastRoute()
        {
            if (routes.Count > 0)
                return routes.Last();

            return null;
        }


        /// <summary>
        /// Проверка соместимости МПЗ и заданного маршрута 
        /// </summary>
        /// <param name="r">Маршрут, который требует проверки на совместимось с данным МПЗ</param>
        /// <returns>Код проверки: true--маршрут и МПЗ совместимы, false--маршрут и МПЗ несовместимы</returns>
        public bool isCompatible(RouteParams r)
        {
            foreach (RouteParams route in routes)
            {
                if (!route.isCompatible(r))
                    return false;
            }
            return this.isCompatibleWithMPZ(r);
        }

        /// <summary>
        /// Проверка совместимости маршрута с самим МПЗ
        /// </summary>
        /// <param name="r"></param>
        /// <returns></returns>
        public bool isCompatibleWithMPZ(RouteParams r)
        {
            if (N_routes < 12)
            {
                double lasting = ((r.end - this.start).TotalMilliseconds + Constants.MPZ_ending_Time_PWRON);
                return lasting < Constants.MPZ_max_lasting_time;
            }

            double delta_time = (r.start - this.end).TotalMilliseconds;
            double dt_mpz = delta_time - Constants.SOEN_turning_on_Time - Constants.MPZ_init_Time;

            return (dt_mpz > 0);
        }

        public static MPZParams createMPZbetween(MPZParams m1, MPZParams m2, List<RouteParams> r)
        {
            if(r==null)
            {
                if((m2.start- m1.end).TotalMilliseconds > Constants.MPZ_ending_Time_PWRON + Constants.SOEN_turning_on_Time + 2 * Constants.MPZ_delta)
                {
                    MPZParams m = new MPZParams(0);
                    m.start = m1.end.AddMilliseconds(Constants.SOEN_turning_on_Time + Constants.MPZ_delta);
                    m.end = m.start;
                    return m;
                }
            }
            else
            {
                double Tmpz = (r.Last().end - r[0].start).TotalMilliseconds + Constants.MPZ_ending_Time_PWRON + Constants.SOEN_turning_on_Time + 2 * Constants.MPZ_delta;
                if ((m2.start - m1.end).TotalMilliseconds < Tmpz)
                    return null;

                MPZParams m = new MPZParams(0,r[0]);
                foreach(RouteParams i in r)
                {
                    m.AddRoute(i);
                }
                return m;
            }
            return null;
        }

        public void renumerateRoutes(object sender, NotifyCollectionChangedEventArgs e)
        {            
            for (int i = 0; i < routes.Count; i++)
            {
                routes[i].NRoute = i;
                routes[i].NPZ = this.id;
            }
        }

        public bool InsertRoute(RouteParams r, DateTime insert_start, DateTime insert_end, MPZParams m_previous=null, MPZParams m_next = null)
        {           
            if (this.N_routes > 11) return false;

            double dmpz = Double.MaxValue;
            if(m_previous!=null)
            {
                dmpz = (start - m_previous.end).TotalMilliseconds;
            }

            RouteParams r0 = routes[0];
            if ((dmpz - Constants.MPZ_delta - r.duration - Constants.CountMinPause(r.type, r.shooting_type, r.shooting_channel, r0.type, r0.shooting_type, r0.shooting_channel) > 0))
            {
                DateTime s_new = start.AddMilliseconds(Constants.MPZ_init_Time - r.duration - Constants.CountMinPause(r.type, r.shooting_type, r.shooting_channel, r0.type, r0.shooting_type, r0.shooting_channel));
                if (s_new > insert_start && s_new.AddMilliseconds(r.duration) < insert_end)
                {
                    r.start = s_new;
                    r.end = r.start.AddMilliseconds(r.duration);
                    start = r.start.AddMilliseconds(-Constants.MPZ_init_Time);
                    routes.Insert(0, r);
                    return true;
                }                
            }

            double lasting = ((r.end - this.start).TotalMilliseconds + Constants.MPZ_ending_Time_PWRON);
            if(lasting < Constants.MPZ_max_lasting_time) return false;
            
            
            for(int i=0;i<N_routes;i++)
            {
                RouteParams r1 = routes[i];
                if(i == (N_routes -1))
                {
                    if (!r.isCompatible(r)) return false;

                    double dt = Constants.CountMinPause(r1.type,r1.shooting_type,r1.shooting_channel, r.type, r.shooting_type, r.shooting_channel);

                    if(m_next!=null)
                    {
                        if (m_next.start < end.AddMilliseconds(dt + r.duration + Constants.MPZ_delta))
                            return false;
                    }

                    if (end.AddMilliseconds(dt + r.duration) > insert_end)
                        return false;

                    DateTime s_new = r1.start.AddMilliseconds(dt);
                    if (s_new > insert_start && s_new.AddMilliseconds(r.duration) < insert_end)
                    {
                        r.start = s_new;
                        r.end = r.start.AddMilliseconds(r.duration);
                        return this.AddRoute(r);
                    }                   
                }

                RouteParams r2 = routes[i + 1];
                double dt1 = Constants.CountMinPause(r1.type, r1.shooting_type, r1.shooting_channel, r.type, r.shooting_type, r.shooting_channel);

                if (r1.end.AddMilliseconds(r.duration + dt1) > insert_end) return false;

                double dt2 = Constants.CountMinPause(r.type, r.shooting_type, r.shooting_channel, r2.type, r2.shooting_type, r2.shooting_channel);
                double dt_r1_r2 = (r2.start - r1.end).TotalMilliseconds;

                if(dt_r1_r2> (dt1 + dt2 + r.duration))
                {
                    DateTime s_new = r1.end.AddMilliseconds(dt1);
                    if (s_new > insert_start && s_new.AddMilliseconds(r.duration) < insert_end)
                    {
                        r.start = r1.end.AddMilliseconds(dt1);
                        r.end = r.start.AddMilliseconds(r.duration);
                        routes.Insert(i + 1, r);                        
                        return true;
                    }                   
                }
            }

            return false;
        }

        public static List<MPZParams> FillMPZ(List<RouteParams> routes, int maxMpzNum = 0)
        {
            List<MPZParams> FTAs = new List<MPZParams>();
            routes.Sort((x, y) => DateTime.Compare(x.start, y.start));
            MPZParams currentMPZ = null;
            int N = maxMpzNum+1;
            foreach(RouteParams r in routes)
            {
                if(currentMPZ==null)
                {
                    if ((FTAs.Count == 0) || (FTAs.Last().isCompatible(r)))
                    {
                        currentMPZ = new MPZParams(N, r);
                        N++;
                    }
                   
                }
                else
                {
                    if (currentMPZ.N_routes == 12)
                    {
                        FTAs.Add(currentMPZ);
                        currentMPZ = null;
                        if((FTAs.Count==0)||(FTAs.Last().isCompatible(r)))
                        {
                            currentMPZ = new MPZParams(N, r);   
                        }
                        
                        N++;
                    }

                    else
                    {
                        if (currentMPZ.isCompatible(r)||currentMPZ.GetLastRoute()==null)
                        {
                             currentMPZ.AddRoute(r);
                        }

                        else
                        {
                            if(currentMPZ.GetLastRoute().isCompatible(r))
                            {
                                FTAs.Add(currentMPZ);
                                currentMPZ = null;
                                currentMPZ = new MPZParams(N, r);
                                N++;
                            }
                        }
                    }
                }
            }

            if (currentMPZ!=null)
                FTAs.Add(currentMPZ);                      
            return FTAs;
        }
    }

 

    public class RouteParams
    {

        /// <summary>
        /// Конструктор для создания параметров маршрута на съемку
        /// </summary>
        /// <param name="_type">тип работы (съемка или съемка со сбросом)</param>
        /// <param name="_shooting_type">тип съемки</param>
        /// <param name="_shooting_channel">канал </param>
        /// <param name="_start">время начала маршрута</param>
        /// <param name="_end">время конца маршрута</param>
        /// <param name="_roll">крен</param>
        /// <param name="_pitch">тангаж</param>        
        /// <param name="_wktPolygon">полигон, снимаемый этим маршрутом</param>        
        /// <param name="_poli_coef">полиноминальные коэффициенты (если крен)</param>
        public RouteParams(
            WorkingType _type,
            ShootingType _shooting_type,
            ShootingChannel _shooting_channel,
            DateTime _start,
            DateTime _end,
            double _roll,
            double _pitch,
            string _wktPolygon,
            SatelliteSessions.PolinomCoef _poli_coef = null)
        {
            if (!(_type == WorkingType.Shooting || _type == WorkingType.ShootingSending))
                throw new ArgumentException("This ctr can create only shooting route");

            if (_shooting_type == ShootingType.Coridor && _poli_coef == null)
                throw new ArgumentException("PolinomCoef must be != null for Coridor shoooting ");

            type = _type;
            start = _start;
            end = _end;
            roll = _roll;
            pitch = _pitch;
            shooting_channel = _shooting_channel;
            shooting_type = _shooting_type;
            poli_coef = _poli_coef;
            wktPolygon = _wktPolygon;
            fileSize = new Lazy<int>(() => calculateFileSize());
        }


        public RouteParams(StaticConf conf)
            : this(conf.type,
                   conf.shooting_type,
                   conf.shooting_channel,
                   conf.dateFrom,
                   conf.dateTo,
                   conf.roll,
                   conf.pitch,
                   conf.wktPolygon,
                   conf.poliCoef)
        {
            id = conf.id;
            wktPolygon = conf.wktPolygon;
            Requests = conf.orders.Select(o => o.request).ToList();
        }

        public RouteParams(
            WorkingType t,
            double dur,
            RouteParams _binded_route,
            double roll,
            double pitch)
        {
            if (t == WorkingType.Shooting || t == WorkingType.ShootingSending)
                throw new ArgumentException("Cannot create shooting route whit this ctr");

            type = t;
            binded_route = _binded_route;
            duration = dur;
            NRoute = -1;
            NPZ = -1;
            this.roll = roll;
            this.pitch = pitch;
            fileSize = new Lazy<int>(() => calculateFileSize());
        }

        public RouteParams(
            WorkingType t,
            DateTime d1,
            DateTime d2,
            RouteParams _binded_route,
            double roll,
            double pitch)
            : this(t, (d2 - d1).TotalSeconds, _binded_route, roll, pitch)
        {
            start = d1;
            end = d2;
        }


        public RouteParams(RouteParams copyed)
        {
            id = copyed.id;
            NRoute = copyed.NRoute;
            NPZ = copyed.NPZ;
            type = copyed.type;
            start = copyed.start;
            end = copyed.end;
            binded_route = copyed.binded_route;
            shooting_channel = copyed.shooting_channel;
            shooting_type = copyed.shooting_type;
            albedo = copyed.albedo;
            zipMK = copyed.zipMK;
            zipPK = copyed.zipPK;
            duration = copyed.duration;
            energo_save_mode = copyed.energo_save_mode;
            TNPos = copyed.TNPos;
            Delta_T = copyed.Delta_T;
            coridorLength = copyed.coridorLength;
            coridorAzimuth = copyed.coridorAzimuth;
            roll = copyed.roll;
            pitch = copyed.pitch;
            poli_coef = copyed.poli_coef;
            formatting_options = copyed.formatting_options;
            wktPolygon = copyed.wktPolygon;
            fileSize = copyed.fileSize;
        }

        static RouteParams()
        {
            dem = new DemHandler();
        }

        public static DemHandler dem { get; private set; }

        /// <summary>
        /// номер маршрута в мпз
        /// </summary>
        public int NRoute { get; set; }
        /// <summary>
        /// номер родительского мпз
        /// </summary>
        public int NPZ { get; set; }

        /// <summary>
        /// флаги форматирования ячеек памяти
        /// </summary>
        public FormattingOptions formatting_options { get; set; }

        public WorkingType type { get; set; } // тип работы
        public ShootingChannel shooting_channel { get; set; } // канал

        public ShootingType shooting_type { get; set; } // тип съемки

        public bool energo_save_mode { get; set; }
        public DateTime start { get; set; }
        public DateTime end { get; set; }

        public RouteParams binded_route { get; set; }

        public List<RequestParams> Requests { get; private set; }

        public double roll { get; set; }
        public double pitch { get; set; }
        public string wktPolygon { get; set; }
        private Lazy<int> fileSize;
        
        /// <summary>
        /// объем файла в Мб
        /// </summary>
        public int File_Size {
            get { return fileSize.Value; }
        }
        
        public byte Hroute
        {
            get
            {
                if ((type == WorkingType.Shooting) || (type == WorkingType.ShootingSending))
                    return (byte)(200 + dem.GetAverageHeight(new SphericalGeom.Polygon(wktPolygon)) / 250);
                else
                    return 0;                  
            }
        }
        
        /// <summary>
        /// внутренний id маршрута для графа (не NRoute)
        /// </summary>
        public int id;

        /// <summary>
        /// Длительность в милисекундах
        /// </summary>
        public double duration { get; set; }//длительность в милисекундах. Задается явно для маршрутов сброса и удаления, когда их время исполнения еще не определено

        /// <summary>
        /// Смещение от начала файла при сбросе инфо (от 0 до 99).
        /// </summary>
        public int TNPos { get; set; }

        /// <summary>
        /// Поправка времени включения блоков СОЭН в suspend mode (от -15 до 15)
        /// </summary>
        public int Delta_T { get; set; }

        /// <summary>
        /// Длина коридора съемки [м]
        /// </summary>
        public double coridorLength { get; set; }

        /// <summary>
        /// Азимут коридора съемки [рад]
        /// </summary>
        public double coridorAzimuth { get; set; }

        /// <summary>
        /// Параметр сжатия ПК:
        /// 0 -- без потерь
        /// 1 -- без сжатия
        /// 2-10 -- сжатие с потерями
        /// </summary>
        public int zipPK { get; set; }

        /// <summary>
        /// Параметр сжатия МК:
        /// 0 -- без потерь
        /// 1 -- без сжатия
        /// 2-10 -- сжатие с потерями
        /// </summary>
        public int zipMK { get; set; }

        /// <summary>
        /// Альбедо
        /// </summary>
        public double albedo { get; set; }

        /// <summary>
        /// полиноминальные коэффициенты
        /// </summary>
        public SatelliteSessions.PolinomCoef poli_coef { get; set; }

        public TimeSpan getDropTime(CommunicationSessionStation station)
        {
            double speed = station == CommunicationSessionStation.FIGS ? 1024.0 : 512.0; // Mb per sec
            return new TimeSpan(0, 0, 0, (int)(File_Size / speed * 8 + 1)); // время на сброс этого роута
        }

         

        /// <summary>
        /// Проверка совместимоси двух маршрутов
        /// </summary>
        /// <param name="r">Маршрут, который нужно рассмотреть</param>
        /// <returns>Код проверки: true--маршруты совместимы, false--маршруты несовместимы</returns>
        public bool isCompatible(RouteParams r)
        {
            RouteParams r1, r2;

            if (r.start > this.start)
            {
                r2 = r;
                r1 = this;
            }
            else
            {
                r1 = r;
                r2 = this;
            }

            if (r1 == null || r2 == null)
                return true;

            double ms = r1.reConfigureMilisecinds(r2);
            if (r.type != 0 || this.type != 0) ms = 0;
            //double min_pause = Constants.CountMinPause(c1.type, c1.shooting_type, c1.shooting_channel, c2.type, c2.shooting_type, c2.shooting_channel);
            double dms = (r2.start - r1.end).TotalMilliseconds;

            return (ms < dms);
        }

        public double reConfigureMilisecinds(RouteParams r2)
        {
            return StaticConf.reConfigureMilisecinds(this.roll, this.pitch, r2.roll, r2.pitch);
        }


        /// <summary>
        /// Суммарный поток информации от ПК и МК [бит/с].
        /// </summary>
        /// <param name="roll">Крен в радианах.</param>
        /// <param name="pitch">Тангаж в радианах.</param>
        /// <param name="hroute">Код средней высоты местности на маршруте.</param>
        /// <param name="codVznCalibr">Поправка длительности времени накопления на шаг ВЗН.</param>
        /// <param name="Nm">Количество незамаскированных спектральных зон МК.</param>
        /// <param name="Zm">Коэффициент сжатия МК.</param>
        /// <param name="Np">Флаг включения ПК: 1 (вкл) или 0 (выкл).</param>
        /// <param name="Zp">Коэффициент сжатия ПК.</param>
        /// <returns></returns>
        public static double InformationFluxInBits(double roll, double pitch, int hroute, int codVznCalibr, int Nm, int Zm, int Np, int Zp)
        {
            /// (N_bit * N_pix * N_channel * N_fpzs + S_tm) / L_pc, где
            ///     N_bit = 12 -- разрядность данных ПК,
            ///     N_pix = 768 -- число пикселов в подканале ПК,
            ///     N_channel = 2 -- число каналов в одном ФПЗС ПК,
            ///     N_fpzs = 12 -- число микросхем ФПЗС в ПК,
            ///     S_tm = 3105 -- число бит телеметрической информации на одну строку изображения ПК
            ///     L_pc = 0.9722 -- отношение размера пиксела ПК к фокусному расстоянию * 1e6.
            double factor_p = 224289 / 0.9722;
            /// (N_bit * N_pix * N_fpzs + S_tm) / 2 / L_mc, где
            ///     N_bit = 12 -- разрядность данных МК,
            ///     N_pix = 384 -- число пикселов в подканале МК,
            ///     N_fpzs = 12 -- число микросхем ФПЗС в МК,
            ///     S_tm = 3105 -- число бит телеметрической информации на одну строку изображения МК,
            ///     L_mc = 1.9445 -- отношение размера пиксела МК к фокусному расстоянию * 1e6.
            double factor_m = 58401 / 1.9445 / 2;

            return 1e6 * WD(roll, pitch) * (0.9 + hroute / 2000.0) * (Np * (factor_p / Zp) + Nm * (factor_m / Zm)) / codVznCalibr;
        }

        /// <summary>
        /// Скорость бега изображения [1/с].
        /// </summary>
        /// <param name="roll">Крен в радианах.</param>
        /// <param name="pitch">Тангаж в радианах.</param>
        /// <returns></returns>
        public static double WD(double roll, double pitch)
        {
            double mu = 398603; // [km^3 / s^2] -- гравитационный параметр Земли
            double R = Astronomy.Constants.EarthRadius;
            double H = OptimalChain.Constants.orbit_height;
            double WD0 = R / H * Math.Sqrt(mu / Math.Pow(R + H, 3)); // [1 / s] -- WD в надир

            return WD0 * Math.Pow(Math.Cos(pitch), 2) * Math.Cos(roll);
        }




        private int calculateFileSize()
        {
            int CodVznCalibr = 1;
            //if (RegimeType == RegimeTypes.ZI_cal)
            //    if (REGta_Param_bytes[1] == 0)
            //        CodVznCalibr = 1;
            //    else
            //        CodVznCalibr = REGta_Param_bytes[1];
            //else
            //    CodVznCalibr = 1;
            int Nm = shooting_channel == ShootingChannel.pk ? 0 : 4;
            int Np = shooting_channel == ShootingChannel.mk ? 0 : 1;
            int zipmk = Math.Max(1, zipMK);
            int zippk = Math.Max(1, zipPK);
            int res = (int)(InformationFluxInBits(roll, pitch,
                Hroute, CodVznCalibr, Nm, zipmk, Np, zippk) * (end - start).TotalSeconds / (1 << 23));
            return res;
        }

    }
}