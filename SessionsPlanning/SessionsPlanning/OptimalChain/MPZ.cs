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

        public List<RouteParams> binded_routes { get; set; }

        public int File_Size { get; set; } //объем файла в Мб

        public RouteParams(int t, DateTime d1, DateTime d2, int st=0, string channel ="pk", int fs = 0)
        {
            type = t;
            shooting_channel = channel;
            shooting_type = st;
            start = d1;
            end = d2;
            binded_routes = null;
            File_Size = fs;
        }

        public RouteParams(int t, DateTime d1, DateTime d2, List<RouteParams> br, int st = 0, string channel = "pk", int fs = 0)
        {
            type = t;
            shooting_channel = channel;
            shooting_type = st;
            start = d1;
            end = d2;
            binded_routes = br;
            File_Size = fs;
        }

        public RouteParams( StaticConf c)
        {
            type = c.type;
            start = c.dateFrom;
            end = c.dateTo;
            binded_routes = null;
            ShootingConf = c;
            shooting_channel = c.shooting_channel;
            shooting_type = c.shooting_type;
        }
    }
}
