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
        public int type { get; set; }
        public bool energo_save_mode { get; set; }
        public DateTime start { get; set; }
        public DateTime end { get; set; }

        public StaticConf ShootingConf { get; set; }

        public List<RouteParams> binded_routes { get; set; }

        public RouteParams(int t, DateTime d1, DateTime d2)
        {
            type = t;
            start = d1;
            end = d2;
            binded_routes = null;
        }

        public RouteParams(int t, DateTime d1, DateTime d2, List<RouteParams> br)
        {
            type = t;
            start = d1;
            end = d2;
            binded_routes = br;
        }

        public RouteParams( StaticConf c)
        {
            type = c.type;
            start = c.dateFrom;
            end = c.dateTo;
            binded_routes = null;

        }
    }
}
