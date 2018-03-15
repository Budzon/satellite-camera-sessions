using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace OptimalChain
{
    
    public class MPZParams
    {
        public int id { get; set; }
        public bool PWR_ON { get; set; }
        public DateTime start { get; set; }
        public DateTime end { get; set; }
        public int N_routes { get; set; }
        public List<RouteParams> routes { get; set; }

        public MPZParams(int i)
        {
            id = i;
            PWR_ON = false;
            N_routes = 0;
            routes = new List<RouteParams>();

        }

        public MPZParams(int i, RouteParams r)
        {
            id = i;
            PWR_ON = false;
            N_routes = 1;
            routes = new List<RouteParams>();
            routes.Add(r);

            start = r.start.AddMilliseconds(-Constants.MPZ_starting_Time);
            end = r.end.AddMilliseconds(Constants.MPZ_ending_Time);
        }

        public bool AddRoute(RouteParams r)
        {
            if (N_routes > 11)
                return false;
            
            routes.Add(r);
            N_routes++;

            if (N_routes < 2)
                start = r.start.AddMilliseconds(-Constants.MPZ_starting_Time);
            
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

            if (N_routes < 12)
            {
                return true;
            }


            double delta_time = (r.start - this.end).TotalMilliseconds;
            double dt_mpz = delta_time - Constants.MPZ_starting_Time - Constants.MPZ_init_Time;

            return (dt_mpz > 0);
            
        }
        
    }

    public class RouteParams
    {
        public int id { get; set; }
        public int type { get; set; }//0 -- удаление, 1 -- съемка, 2 -- сброс, 3 -- съемка со сбросом
        public string shooting_channel { get; set; }// pk, mk, cm

        public int shooting_type { get; set; }//0 -- обычная съемка, 1-- стерео, 2 -- коридорная;

        public bool energo_save_mode { get; set; }
        public DateTime start { get; set; }
        public DateTime end { get; set; }

        public StaticConf ShootingConf { get; set; }

        public Tuple<int,int> binded_route { get; set; }

        public int File_Size { get; set; } //объем файла в Мб

        public double getDropTime()
        {
            return (double)File_Size / 300 * 8 + 1; // время на сброс этого роута
        }

        public RouteParams(int t, DateTime d1, DateTime d2, int st=0, string channel ="pk", int fs = 1000)
        {
            type = t;
            shooting_channel = channel;
            shooting_type = st;
            start = d1;
            end = d2;
            binded_route = null;
            File_Size = fs;
        }

        public RouteParams(int t, DateTime d1, DateTime d2,Tuple<int,int>  br, int st = 0, string channel = "pk", int fs = 1000)
        {
            type = t;
            shooting_channel = channel;
            shooting_type = st;
            start = d1;
            end = d2;
            binded_route = br;
            File_Size = fs;
        }

        public RouteParams( StaticConf c)
        {
            type = c.type;
            start = c.dateFrom;
            end = c.dateTo;
            binded_route = c.connected_route;
            ShootingConf = c;
            shooting_channel = c.shooting_channel;
            shooting_type = c.shooting_type;
        }

        public RouteParams(StaticConf c, int fs = 1000)
        {
            type = c.type;
            start = c.dateFrom;
            end = c.dateTo;
            binded_route = c.connected_route;
            ShootingConf = c;
            shooting_channel = c.shooting_channel;
            shooting_type = c.shooting_type;
            File_Size = fs;
        }

        
        /// <summary>
        /// Проверка совместимоси двух маршрутов
        /// </summary>
        /// <param name="r">Маршрут, который нужно рассмотреть</param>
        /// <returns>Код проверки: true--маршруты совместимы, false--маршруты несовместимы</returns>
        public bool isCompatible(RouteParams r)
        {
            StaticConf c1, c2;
            if(r.start>this.start)
            {
                c2 = r.ShootingConf;
                c1 = this.ShootingConf;
            }
            else
            {
                c1 = r.ShootingConf;
                c2 = this.ShootingConf;
            }
            double ms = c1.reConfigureMilisecinds(c2);
            double min_pause = Constants.CountMinPause(c1.type, c1.shooting_type, c1.shooting_channel, c2.type, c2.shooting_type, c2.shooting_channel);
            double dms = (c2.dateFrom - c1.dateTo).TotalMilliseconds;

            return (ms < dms);
            
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
    }
}
