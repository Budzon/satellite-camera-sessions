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
        
        public void CreateEdges()
        {
            Vertex A = new Vertex("A");
            Vertex B = new Vertex("B");

            foreach (Vertex v1 in vertices)
            {
                A.addEdge(v1, v1.value);

                foreach (Vertex v2 in vertices.Where(i => (i.s.dateFrom > v1.s.dateFrom) && (i.s.id != v1.s.id)))
                {
                    double w = this.countEdgeWeight(v1, v2);

                    if (w > 0)
                        v1.addEdge(v2, w);
                }

                v1.addEdge(B, 0);

            }

            vertices.Insert(0, A);
            vertices.Add(B);
        }
        public Graph (List<CaptureConf> strips)
        {
            vertices = new List<Vertex>();

            foreach(CaptureConf s in strips)
            {
                Console.WriteLine(s.id);
                vertices.Add(new Vertex(s.CreateStaticConf(0)));

                for (int dt = 1; dt < s.timeDelta;dt++ )
                {
                    StaticConf s1 = s.CreateStaticConf(dt);
                    StaticConf s2 = s.CreateStaticConf(-dt);

                    if (s1 != null) vertices.Add(new Vertex(s1));
                    if (s2 != null) vertices.Add(new Vertex(s2));
                }
                    
                
                    
            }

            CreateEdges();

        }


        public Graph(List<StaticConf> strips)
        {
            vertices = new List<Vertex>();

            foreach (StaticConf s in strips)
            {
                vertices.Add(new Vertex(s));
            }

            CreateEdges();
        }

        public double countEdgeWeight(Vertex v1, Vertex v2)
        {
            if (v2.s == null) return 0;
            if (v1.s == null) return v2.value;

            double weight = 0;
            double ms = v1.s.reConfigureMilisecinds(v2.s);

            double dms = (v2.s.dateFrom - v1.s.dateTo).TotalMilliseconds;

            if (ms < dms)
                return v2.value;

            double dt = 0;

            if (dms>0)
                dt = ms - dms;
            else
                dt = ms + Math.Abs(dms);

            
            Vertex v = v1.value > v2.value ? v2 : v1;
            double l = (v.s.dateTo - v.s.dateFrom).TotalMilliseconds;

            if (dt >= l)
                return -1;

            if ((l - dt) < Constants.min_shooting_time)
                return -1;

            weight = v2.value - v.value * ( dt/l);

            return weight;
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

        public List<StaticConf> findOptimalChain()
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
                            v.mark = mark_new;
                            v.path = e.v1.path.ToList();
                            if(v.s!=null)
                                    v.path.Add(v.s);

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

        public int color { get; set; }
        public double value { get; set; }

        public string key { get; set; }
        public bool isVisited { get; set; }

        public List<Edge> edges { get; set; }

        public List<Edge> in_edges { get; set; }

        public double mark { get; set; }

        public List<StaticConf> path { get; set; }

        public Vertex(string k)
        {
            s = null;
            key = k;
            color = 0;
            value = 0;
            isVisited = false;

            mark = -1;
            edges = null;
            path = new List<StaticConf>();
        }
        public Vertex(StaticConf ss)
        {
            s = ss;
            key = s.roll.ToString();
            color = 0;
            value = countVertexPrice();
            isVisited = false;

            mark = -1;
            edges = null;
            path = new List<StaticConf>();
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

