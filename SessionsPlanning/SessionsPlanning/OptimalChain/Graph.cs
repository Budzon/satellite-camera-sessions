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

                foreach (Vertex v2 in vertices.Where(i => (i.cs.dateFrom.AddSeconds(i.cs.timeDelta) > v1.cs.dateTo.AddSeconds(Constants.min_Delta_time - v1.cs.timeDelta)) && (i.s.id != v1.s.id)))
                {
                    double w = this.countEdgeWeight(v1, v2);

                    if (w > 0)
                    {
                       // Console.WriteLine("OOOOO");
                        v1.addEdge(v2, w);
                    }
                }

                v1.addEdge(B, 0);

            }

            Console.WriteLine("Additional vertices NUM = " + additional_vertices.Count);

            foreach(Vertex v1 in additional_vertices)
                {
                    A.addEdge(v1, v1.value);


                    foreach (Vertex v2 in additional_vertices.Where(i => (i.cs.dateFrom.AddSeconds(i.cs.timeDelta) > v1.cs.dateTo.AddSeconds(Constants.min_Delta_time - v1.cs.timeDelta)) && (i.s.id != v1.s.id)))
                    {
                        double w = this.countEdgeWeight(v1, v2, false);

                        if (w > 0)
                            v1.addEdge(v2, w);
                    }
                    foreach (Vertex v2 in vertices.Where(i => (i.s.id != v1.s.id)))
                    {
                        if(v2.s.dateFrom > v1.s.dateTo)
                        {
                            double w = this.countEdgeWeight(v1, v2, false);

                            if (w > 0)
                                v1.addEdge(v2, w);
                        }

                        if (v1.s.dateFrom > v2.s.dateTo)
                        {
                            double w = this.countEdgeWeight(v2, v1, false);

                            if (w > 0)
                                v2.addEdge(v1, w);
                        }
                        
                    }

                    v1.addEdge(B, 0);
                    vertices.Add(v1);
                }


            vertices.Insert(0, A);
            vertices.Add(B);
        }
        public Graph (List<CaptureConf> strips)
        {
            vertices = new List<Vertex>();
            additional_vertices = new List<Vertex>();


            foreach(CaptureConf s in strips)
            {
             //   Console.WriteLine(s.id);
                vertices.Add(new Vertex(s.DefaultStaticConf(), s));

                //for (int dt = 1; dt < s.timeDelta;dt++ )
                //{
                //    StaticConf s1 = s.CreateStaticConf(dt);
                //    StaticConf s2 = s.CreateStaticConf(-dt);

                //    if (s1 != null) vertices.Add(new Vertex(s1,s));
                //    if (s2 != null) vertices.Add(new Vertex(s2,s));
                //}
                    
                                    
            }

            CreateEdges();

        }


        //public Graph(List<StaticConf> strips)
        //{
        //    vertices = new List<Vertex>();

        //    foreach (StaticConf s in strips)
        //    {
        //        vertices.Add(new Vertex(s));
        //    }

        //    CreateEdges();
        //}

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
                        // Console.WriteLine("Delta = " + dt);
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


        public int CountMinPause(int t1, int t2)
        {
            return 12000;
        }

        public bool CheckPossibility(StaticConf c1, StaticConf c2 )
        {

            if (c1 == null || c2 == null) return false;
            
            double ms = c1.reConfigureMilisecinds(c2);
            double min_pause = CountMinPause(c1.type, c2.type);
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

            Vertex v3 = GenerateNewConf(v1.cs, v2.s, false);
            Vertex v4 = GenerateNewConf(v2.cs, v1.s, true);
            
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

        public List<fakeMPZ> findOptimalChain()
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
                        if(mark_new>v.mark)
                        {
                          //  Console.WriteLine("Bingo!");
                            if (v.s == null) 
                            {
                               //; Console.WriteLine("V.s == null!");
                                v.mark = mark_new;
                                v.path = e.v1.path.ToList();
                            }
                            else
                            {
                                if (e.v1.path.Count > 0)
                                {
                                    fakeMPZ lastMPZ = e.v1.path.Last();
                                    if (lastMPZ != null)
                                    {
                                        if (lastMPZ.N_routes < 12)
                                        {
                                            Route lastRoute = lastMPZ.GetLastRoute();
                                            int min_t = CountMinPause(lastRoute.type, v.s.type);
                                            
                                            if (lastRoute.end.AddMilliseconds(min_t) < v.s.dateFrom)
                                                {
                                                   // Console.WriteLine("New route");
                                                    v.mark = mark_new;
                                                    v.path = e.v1.path.ToList();
                                                    v.path.Last().AddRoute(new Route(v.s));

                                                }

                                        }
                                        else
                                        {
                                            if (lastMPZ.end.AddMilliseconds(Constants.MPZ_delta) < v.s.dateFrom)
                                            {
                                              //  Console.WriteLine("New MPZ");
                                                v.mark = mark_new;
                                                v.path = e.v1.path.ToList();
                                                int N = v.path.Count;
                                                v.path.Add(new fakeMPZ(N, new Route(v.s)));
                                            }
                                        }
                                        
                                    }
                                }

                                else
                                {
                                   // Console.WriteLine("START");
                                    v.mark = mark_new;
                                    v.path.Add(new fakeMPZ(0, new Route(v.s)));
                                }
                            }
                           
                            
                            

                        }
                    }
                }

            }

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

        public List<fakeMPZ> path { get; set; }

        public Vertex(string k)
        {
            s = null;
            key = k;
            color = 0;
            value = 0;
            isVisited = false;

            mark = -1;
            edges = null;
            path = new List<fakeMPZ>();
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
            path = new List<fakeMPZ>();
        }

        public double countVertexPrice()
        {
            double sum = 0;
            int[] pr_coef = new int[]{ 10, 100, 1000};
            foreach(Order o in s.orders)
            {
                sum += s.square * o.intersection_coeff*pr_coef[o.request.priority-1];
            }
            return sum;
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

