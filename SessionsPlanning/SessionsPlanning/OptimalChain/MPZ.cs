using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace OptimalChain
{
    
    public class fakeMPZ
    {
        public int id { get; set; }
        public bool PWR_ON { get; set; }
        public DateTime start { get; set; }
        public DateTime end { get; set; }
        public int N_routes { get; set; }
        public List<Route> routes { get; set; }

        public fakeMPZ(int i)
        {
            id = i;
            PWR_ON = false;
            N_routes = 0;
            routes = new List<Route>();

        }

        public fakeMPZ(int i, Route r)
        {
            id = i;
            PWR_ON = false;
            N_routes = 1;
            routes = new List<Route>();
            routes.Add(r);

            start = r.start.AddMilliseconds(-Constants.MPZ_starting_Time);
            end = r.end.AddMilliseconds(Constants.MPZ_ending_Time);
        }

        public bool AddRoute(Route r)
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

        public Route GetLastRoute()
        {
            if (routes.Count > 0)
                return routes.Last();
            
            return null;
        }
        
    }

    public class Route
    {
        public int id { get; set; }
        public int type { get; set; }
        public bool energo_save_mode { get; set; }
        public DateTime start { get; set; }
        public DateTime end { get; set; }

        public StaticConf ShootingConf { get; set; }

        public List<Route> binded_routes { get; set; }

        public Route(int t, DateTime d1, DateTime d2)
        {
            type = t;
            start = d1;
            end = d2;
            binded_routes = null;
        }

        public Route(int t, DateTime d1, DateTime d2, List<Route> br)
        {
            type = t;
            start = d1;
            end = d2;
            binded_routes = br;
        }

        public Route( StaticConf c)
        {
            type = c.type;
            start = c.dateFrom;
            end = c.dateTo;
            binded_routes = null;

        }
    }
}
