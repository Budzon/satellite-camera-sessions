using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Concurrent;


namespace OptimalChain
{
    public class Graph
    {
        public List<Vertex> vertices { get; set; }
        public List<Vertex> additional_vertices { get; set; }
        
        public void CreateEdges()
        {
            Vertex A = new Vertex("A");
            Vertex B = new Vertex("B");

           // Console.WriteLine("Vertices number = " + vertices.Count);
            int count = 1;

            foreach (Vertex v1 in vertices)
            {
               // Console.WriteLine("Processing vertex = " + count);
                count++;
                A.addEdge(v1, v1.value);

                foreach (Vertex v2 in vertices.Where(i => (i.cs.dateFrom.AddSeconds(i.cs.timeDelta) >= v1.cs.dateTo.AddSeconds(Constants.min_Delta_time/1000 - v1.cs.timeDelta))))
                {
                    if((v1.s.shooting_type==1)&&(v1.s.pitch>=-0.0872665))
                    {
                        if((v2.s.id == v1.s.id))
                        {
                            double w = this.countEdgeWeight(v1, v2, false);

                            if (w > 0)
                            {
                                // Console.WriteLine("OOOOO");
                                v1.addEdge(v2, w);
                            }
                        }
                    }
                    else
                    {
                        if((v2.s.id != v1.s.id))
                        {
                            double w = this.countEdgeWeight(v1, v2,false);

                             if (w > 0)
                             {
                                // Console.WriteLine("OOOOO");
                                 v1.addEdge(v2, w);
                             }
                        }
                    }
                    
                    
                }

                v1.addEdge(B, 0);

            }

            //Console.WriteLine("Additional vertices NUM = " + additional_vertices.Count);
            //int nnn = 1;
            //while (additional_vertices.Count > 0)
            //{
            //    Console.WriteLine("Iteration " + nnn);
            //    nnn++;
            //    List<Vertex> TempList = new List<Vertex>(additional_vertices);
            //    additional_vertices.Clear();
            //    foreach (Vertex v1 in TempList)
            //    {
            //        A.addEdge(v1, v1.value);


            //        foreach (Vertex v2 in TempList.Where(i => (i.cs.dateFrom.AddSeconds(i.cs.timeDelta) > v1.cs.dateTo.AddSeconds(Constants.min_Delta_time / 1000 - v1.cs.timeDelta)) && (i.s.id != v1.s.id)))
            //        {
            //            double w = this.countEdgeWeight(v1, v2);

            //            if (w > 0)
            //                v1.addEdge(v2, w);
            //        }
            //        foreach (Vertex v2 in vertices.Where(i => (i.s.id != v1.s.id)))
            //        {
            //            if (v2.cs.dateFrom.AddSeconds(v2.cs.timeDelta) > v1.cs.dateTo.AddSeconds(Constants.min_Delta_time / 1000 - v1.cs.timeDelta))
            //            {
            //                double w = this.countEdgeWeight(v1, v2,false);

            //                if (w > 0)
            //                    v1.addEdge(v2, w);
            //            }

            //            if (v1.cs.dateFrom.AddSeconds(v1.cs.timeDelta) > v2.cs.dateTo.AddSeconds(Constants.min_Delta_time / 1000 - v2.cs.timeDelta))
            //            {
            //                double w = this.countEdgeWeight(v2, v1,false);

            //                if (w > 0)
            //                    v2.addEdge(v1, w);
            //            }

            //        }

            //        v1.addEdge(B, 0);
            //        vertices.Add(v1);
            //    }

            //    Console.WriteLine("Additional vertices NUM = " + additional_vertices.Count);
            //}


            vertices.Insert(0, A);
            vertices.Add(B);
        }
        public Graph (List<CaptureConf> strips)
        {
            vertices = new List<Vertex>();
            additional_vertices = new List<Vertex>();
            

            foreach(CaptureConf s in strips)
            {
            //    Console.WriteLine("Conf " + s.rollAngle + " TimeStart " + s.dateFrom + " pitch[1] " + s.pitchArray[1]);
                if(s.shootingType!=1)
                {
                    vertices.Add(new Vertex(s.DefaultStaticConf(), s));
                    for (int i = 0; i < s.timeDelta; i++)
                    {
                        vertices.Add(new Vertex(s.CreateStaticConf(i, 1), s));
                        vertices.Add(new Vertex(s.CreateStaticConf(i, -1), s));
                    }
                }
                if (s.shootingType == 1)
                {
                  foreach (KeyValuePair<double, Tuple<double,double>> p in s.pitchArray)
                  {
                      vertices.Add(new Vertex(s.CreateStaticConf(p.Key, 1), s));
                  }
                }
              
                                    
            }
            Console.WriteLine("Number of Verices = " + vertices.Count);
            CreateEdges();

        }


        public Vertex GenerateNewConf(CaptureConf s1, StaticConf s2, bool forward_delta)
        {
            int direction = -1;
            if (forward_delta)
                direction = 1;
            for (int dt = 0; dt < s1.timeDelta; dt++)
            {
                StaticConf new_conf = s1.CreateStaticConf(dt,direction);
                if(new_conf!=null)
                {
                 bool go = forward_delta ? CheckPossibility(s2, new_conf) : CheckPossibility(new_conf, s2);
                     if (go)
                     {
                         return new Vertex(new_conf, s1);
                     }
                    
                }
                else
                {
                    return null;
                }
               
                   
            }
                return null;
        }


        public bool CheckPossibility(StaticConf c1, StaticConf c2 )
        {

            if (c1 == null || c2 == null) return false;
            
            double ms = c1.reConfigureMilisecinds(c2);
            double min_pause = Constants.CountMinPause(c1.type,c1.shooting_type,c1.shooting_channel, c2.type, c2.shooting_type,c2.shooting_channel);
            double dms = (c2.dateFrom - c1.dateTo).TotalMilliseconds;

            return (ms < dms);

        }
        public double countEdgeWeight(Vertex v1, Vertex v2, bool change_strips = true)
        {
            if (v2.s == null) return 0;
            if (v1.s == null) return v2.value;

            if (CheckPossibility(v1.s, v2.s))
                    return v2.value;

            if(!change_strips)
               return -1;

            Vertex v3 = null;
            if(v1.s.shooting_type!=1)
                    v3= GenerateNewConf(v1.cs, v2.s, false);

            Vertex v4 = null;
            if (v2.s.shooting_type != 1) 
                v4=GenerateNewConf(v2.cs, v1.s, true);
            
            if (v3 != null)
               additional_vertices.Add(v3);
            
            if (v4 != null)
                additional_vertices.Add(v4);
            return -1;
            

            //НИЖЕ ИДЕТ СЛУЧАЙ, КОГДА МОЖНО ПЕРЕКЛЮЧАТЬСЯ ПОСЕРЕДИНЕ ПОЛОСЫ

            //double dt = 0;

            //if (dms>0)
            //    dt = ms - dms;
            //else
            //    dt = ms + Math.Abs(dms);

            
            //Vertex v = v1.value > v2.value ? v2 : v1;
            //double l = (v.s.dateTo - v.s.dateFrom).TotalMilliseconds;

            //if (dt >= l)
            //    return -1;

            //if ((l - dt) < Constants.min_shooting_time)
            //    return -1;

            //weight = v2.value - v.value * ( dt/l);

            //return weight;
        }

        public List<Vertex> deepGo(Vertex a)
        {
            List<Vertex> res = new List<Vertex>();
            a.color = 1;
            if(a.edges!=null)
            {
                foreach(Edge e in a.edges)
                {
                    if(e.v2.color<2)
                    {
                        List<Vertex> res_ch = this.deepGo(e.v2);
                        foreach (Vertex v in res_ch)
                        {
                                res.Add(v);
                        }
                    }
                    
                }
                
            }
            a.color = 2;
            res.Add(a);

            return res;
        }

        public List<MPZParams> findOptimalChain()
        {
            List<Vertex> sorted = this.deepGo(vertices[0]);
          
            sorted.Reverse();
            
            sorted[0].mark = 0;
            foreach(Vertex v in sorted)
            {
                if(v.in_edges!=null)
                {
                    foreach(Edge e in v.in_edges)
                    {
                        double mark_new = e.weight + e.v1.mark;
                        List<int> ids = new List<int>();
                        foreach (MPZParams m in e.v1.path)
                        {
                            foreach (RouteParams r in m.routes)
                            {
                                ids.Add(r.ShootingConf.id);
                            }
                        }

                        if(mark_new>v.mark)
                        {
                            if (v.s == null) 
                            {
                                v.mark = mark_new;
                                v.path = e.v1.path.ToList();
                            }
                            else
                            {
                                if((!ids.Contains(v.s.id))||v.s.shooting_type==1)
                                {
                                    if ((e.v1.path.Count > 0))
                                        {
                                            MPZParams lastMPZ = e.v1.path.Last();
                                            if (lastMPZ != null)
                                            {
                                                RouteParams newRoute = new RouteParams(v.s);
                                                if ((lastMPZ.N_routes < 12)&&(lastMPZ.isCompatible(newRoute)))
                                                {
                                                            v.mark = mark_new;
                                                            v.path = e.v1.path.ToList();
                                                            v.path.Last().AddRoute(new RouteParams(v.s));
                                                            ids.Add(v.s.id);

                                                }
                                                else
                                                {
                                                    if (lastMPZ.GetLastRoute().isCompatible(newRoute))
                                                    {
                                                        v.mark = mark_new;
                                                        v.path = e.v1.path.ToList();
                                                        int N = v.path.Count;
                                                        v.path.Add(new MPZParams(N, new RouteParams(v.s)));
                                                        ids.Add(v.s.id);
                                                    }
                                                }
                                        
                                            }
                                        }

                                        else
                                        {
                                            v.mark = mark_new;
                                            v.path.Add(new MPZParams(0, new RouteParams(v.s)));
                                            ids.Add(v.s.id);
                                        }
                                }
                               
                            }
                        }
                    }
                }

            }
            Console.WriteLine("MPZ num = " + vertices.Last().path.Count);
            foreach (MPZParams m in vertices.Last().path)
            {
                Console.WriteLine("**************************");
                Console.WriteLine("Routes num = "+m.routes.Count);
                foreach (RouteParams r in m.routes)
                            {

                                Console.WriteLine("-------------------------");
                                Console.WriteLine( r.ShootingConf.id + " " + r.start + "  " + r.ShootingConf.roll + "  " + r.ShootingConf.pitch);
                            }
            }

            Console.WriteLine("Graph did his very best ");
            return vertices.Last().path;
        }
    }

    public class Vertex
    {
        public StaticConf s { get; set; }

        public CaptureConf cs{ get; set; }
        public int color { get; set; }
        public double value { get; set; }

        public string key { get; set; }
        public bool isVisited { get; set; }

        public List<Edge> edges { get; set; }

        public List<Edge> in_edges { get; set; }

        public double mark { get; set; }

        public List<MPZParams> path { get; set; }

        public Vertex(string k)
        {
            s = null;
            key = k;
            color = 0;
            value = 0;
            isVisited = false;

            mark = -1;
            edges = null;
            path = new List<MPZParams>();
        }
        public Vertex(StaticConf ss,CaptureConf c)
        {
            s = ss;
            cs = c;
            key = s.roll.ToString() + "/" + s.pitch.ToString();
            color = 0;
            value = countVertexPrice();
            isVisited = false;

            mark = -1;
            edges = null;
            path = new List<MPZParams>();
        }

        public double countVertexPrice()
        {
            double sum = 0;
            if (s.type != 2)
            {
             int[] pr_coef = new int[]{ 10, 100, 1000};
             foreach(Order o in s.orders)
             {
                 sum += s.square * o.intersection_coeff*pr_coef[o.request.priority-1];
             }
             return sum;
            }

            return 1;
        }

        public void addEdge(Vertex v2,double w)
        {
            if (edges==null)
            {
                edges = new List<Edge>();
            }
            Edge e = new Edge(this, v2, w);
            edges.Add(e);
            v2.addInEdge(e);
           // Console.WriteLine("New Edge");
        }

        public void addInEdge(Edge e)
        {
            if (in_edges == null)
            {
                in_edges = new List<Edge>();
            }

            in_edges.Add(e);

        }
    }

    public class Edge
    {
        public Vertex v1 { get; set; }
        public Vertex v2 { get; set; }
        public double weight { get; set; }

        public Edge(Vertex n1,Vertex n2, double w)
        {
            v1 = n1;
            v2 = n2;
            weight = w;

        }

    }
}

