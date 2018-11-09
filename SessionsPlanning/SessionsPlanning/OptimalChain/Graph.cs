using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Concurrent;
using SessionsPlanning;

namespace OptimalChain
{
    public class Graph
    {
        public List<Vertex> vertices { get; set; }

        /// <summary>
        /// Контруктор, создающий граф для планирования по набору возможных конфигураций
        /// </summary>
        /// <param name="strips">Набор возможных конфигураций съемки</param>
        public Graph(List<CaptureConf> strips)
        {
            vertices = new List<Vertex>();

            foreach (CaptureConf s in strips)
            {
                //Для всех не-стерео конфигураций создается много вершин: для всех возможных времен съемки ( 0<|t|<timeDelta)
                if ((s.shootingType != ShootingType.StereoTriplet) && (s.shootingType != ShootingType.Stereo))
                {
                    vertices.Add(new Vertex(s.DefaultStaticConf(), s));
                    for (int i = 0; i <= s.timeDelta; i++)
                    {
                        vertices.Add(new Vertex(s.CreateStaticConf(i, 1), s));
                        vertices.Add(new Vertex(s.CreateStaticConf(i, -1), s));
                    }
                }

                //Для стерео вершины гененрируются с одинаковыми Id, потому что их надо снять все вместе.
                //Для стерео еще и смещение по времени задается по ключу в  pitchArray
                if ((s.shootingType == ShootingType.StereoTriplet) || (s.shootingType == ShootingType.Stereo))
                {
                    foreach (KeyValuePair<double, Tuple<double, double>> p in s.pitchArray)
                    {
                        //когда добираемся до последнего стереокадра (там p.Key>0), то в контруктор отправляется true 
                        vertices.Add(new Vertex(s.CreateStaticConf(p.Key, 1), s, p.Key>0? true:false));
                    }
                }


            }
            
            //Надо между вершинами проложить ребра графа, где это возможно
            CreateEdges();

        }

        /// <summary>
        /// Для всех пар вершин, для которых возоможно, создается ребро и ему присваивается вес, в зависимости от ценности съемки
        /// </summary>
        public void CreateEdges()
        {
            //стартовая и финальная вершина графа, между которыми будет проложен оптимальный путь
            Vertex A = new Vertex("A");
            Vertex B = new Vertex("B");

            //для каждой вершины графа....
            foreach (Vertex v1 in vertices)
            {
                A.addEdge(v1, v1.value);

                //ищущтся все возможные вершины, с которыми ее теоретически можно соединить
                foreach (Vertex v2 in vertices.Where(i => (i.route.start >= v1.route.end.AddSeconds(Constants.minReconfStabilizationT))))
                {
                    //если вершина -- сетерео, причем не последний кадр -- то надо искать ей пару с таким же id
                    if (((v1.s.shooting_type == ShootingType.StereoTriplet) || (v1.s.shooting_type == ShootingType.Stereo))&&(!v1.lastStereoFrame))
                    {
                        if((v2.s.id == v1.s.id))
                        {
                            double w = this.countEdgeWeight(v1, v2);

                            if (w > 0)
                            {
                                v1.addEdge(v2, w);
                            }
                        }
                    }
                    //для всех остальных вершин обязательно нужно искать пару с другим id
                    else
                    {
                        if((v2.s.id != v1.s.id))
                        {
                            double w = this.countEdgeWeight(v1, v2);

                             if (w > 0)
                             {
                                 v1.addEdge(v2, w);
                             }
                        }
                    }
                    
                    
                }

                v1.addEdge(B, 0);

            }

           


            vertices.Insert(0, A);
            vertices.Add(B);

                  //  Console.WriteLine("Add new verties pack");
        }

        /// <summary>
        /// Проверка совместимости двух вершин-съемок
        /// </summary>
        /// <param name="v1">вершина, откуда совершается переход</param>
        /// <param name="v2">вершина, куда совершается переход</param>
        /// <returns></returns>
        public bool CheckPossibility(Vertex v1, Vertex v2 )
        {
            
            if (v1.s == null || v2.s == null) return false;
            
            double reconf_s = v1.s.reConfigureSeconds(v2.s);
            double needded_pause = reconf_s + Constants.minReconfStabilizationT;
            double dms = (v1.route.start - v2.route.end).TotalSeconds;

            if(c2.dateFrom<c1.dateTo)
            {
                Console.WriteLine("ms = " + ms);
                Console.WriteLine("min_pause = " + min_pause);
                Console.WriteLine("needded_pause = " + needded_pause);
                Console.WriteLine("dms = " + dms);

            }

            if(needded_pause < dms)
                 Console.WriteLine("Possible edge from {0}-{1} to {2}-{3}", c1.dateFrom, c1.dateTo, c2.dateFrom, c2.dateTo);

            return (needded_pause < dms);

        }

        /// <summary>
        /// Подсчет ценности перехода между вершинами
        /// </summary>
        /// <param name="v1">вершина, откуда совершается переход</param>
        /// <param name="v2">вершина, куда совершается переход</param>
        /// <returns></returns>
        public double countEdgeWeight(Vertex v1, Vertex v2)
        {
            if (v2.s == null) return 0;
            if (v1.s == null) return v2.value;

            //переход не имеет ценности, если нельзя перестроится между двумя вершинами-съемками вовремя
            if (CheckPossibility(v1, v2))
                    return v2.value;

            return -1;
                       
        }

        /// <summary>
        /// Топологическая сортировка вершин графа
        /// </summary>
        /// <param name="a">Стартовая вершина</param>
        /// <returns></returns>
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

        public List<MPZParams> findOptimalChain(int maxMpzNum = 0)
        {
            //Топологически сортируем вершины, берем обратный порядок
            List<Vertex> sorted = this.deepGo(vertices[0]);
            sorted.Reverse();
            sorted[0].mark = 0;

            foreach (Vertex v in sorted)
            {
                //Рассматриваем все входящие ребра
                if(v.in_edges!=null)
                {
                    //Для каждой вершины, из которой исходит такое ребро считаем новую метку
                    foreach(Edge e in v.in_edges)
                    {
                        double mark_new = e.weight + e.v1.mark;
                        if((e.v1.s!=null)&&(e.v2.s!=null))
                        {
                            Console.WriteLine("using edge {0}-{1}  --  {2}-{3}", e.v1.s.dateFrom, e.v1.s.dateTo, e.v2.s.dateFrom, e.v2.s.dateTo);
                        }
                        List<int> ids = new List<int>();

                        //Если новая метка больше, то надо оптимальный путь до текущей вершины обновить
                        if(mark_new>v.mark)
                        {
                            //Запоминаем, какие конфигурации уже участвуют в этом пути. Чтобы второй раз не снять то же самое
                            foreach (MPZParams m in e.v1.path)
                            {
                                foreach (RouteParams r in m.routes)
                                {
                                    ids.Add(r.id);
                                }
                            }

                            //Если текущая вершина не имеет конфигурации съемки, т.е. последняя или первая( второе -- вряд ли)
                            if (v.s == null) 
                            {
                                v.mark = mark_new;
                                v.path = e.v1.path.ToList();

                                Console.WriteLine("Final mark " +v.mark);
                                Console.WriteLine("Final path "+ v.path.Last().N_routes);
                            }

                            //Если вершина содержит съемку
                            else
                            {
                                //Надо проверить, не снимали ли мы уже такую конфигурацию, если только это не стереотриплет. Если стереотриплет -- то пофиг
                                if ((!ids.Contains(v.s.id)) || (v.s.shooting_type == ShootingType.StereoTriplet || v.s.shooting_type == ShootingType.Stereo))
                                {
                                    //Если путь не пустой
                                    if ((e.v1.path.Count > 0))
                                        {
                                            //Берем последнее МПЗ из этого пути
                                            MPZParams lastMPZ = e.v1.path.Last();
                                            if (lastMPZ != null)
                                            {
                                                RouteParams newRoute = v.route;
                                                //если можно, добавляем маршрут в последнее МПЗ, перезаписываем метку вершины
                                                if ((lastMPZ.N_routes < 12)&&(lastMPZ.isCompatible(newRoute)))
                                                {
                                                            v.mark = mark_new;
                                                            v.path = MPZParams.CopyMPZList(e.v1.path.ToList());
                                                            v.path.Last().AddRoute(new RouteParams(v.s));

                                                            ids.Add(v.s.id);

                                                }
                                                else
                                                {
                                                    //Если в последнее нелья, но в принципе маршрут совместим с последним добавленным маршрутом, то создаем новое МПЗ.
                                                    if (lastMPZ.GetLastRoute()!=null)
                                                    {
                                                     if (lastMPZ.GetLastRoute().isCompatible(newRoute))
                                                        {
                                                            v.mark = mark_new;

                                                            v.path = MPZParams.CopyMPZList(e.v1.path.ToList());
                                                            int N = lastMPZ.id+1;
                                                            v.path.Add(new MPZParams(N, new RouteParams(v.s)));

                                                        ids.Add(v.s.id);
                                                        }
                                                    }
                                                    
                                                }
                                        
                                            }
                                        }
                                        //Если путь еще пустой, просто добавляем новое МПЗ
                                        else
                                        {
                                            v.mark = mark_new;
                                            v.path.Add(new MPZParams(maxMpzNum+1, new RouteParams(v.s)));
                                            ids.Add(v.s.id);
                                        }
                                }
                               
                            }
                        }
                    }
                }

            }
#if DEBUG
            Console.WriteLine("MPZ num = " + vertices.Last().path.Count);
            foreach (MPZParams m in vertices.Last().path)
            {
                Console.WriteLine("**************************");
                Console.WriteLine("Routes num = "+m.routes.Count);
                foreach (RouteParams r in m.routes)
                {
                    Console.WriteLine("-------------------------");
                    Console.WriteLine( r.id + " " + r.start + "  " + r.roll + "  " + r.pitch);
                }
            }

            Console.WriteLine("Graph did his very best ");
#endif
            return vertices.Last().path;
        }
    }

    public class Vertex
    {
        public StaticConf s { get; set; }

        public CaptureConf cs{ get; set; }

        public RouteParams route { get; set; }

        public int color { get; set; }
        public double value { get; set; }

        public string key { get; set; }
        public bool isVisited { get; set; }

        public bool lastStereoFrame { get; set; }

        public List<Edge> edges { get; set; }

        public List<Edge> in_edges { get; set; }

        public double mark { get; set; }

        public List<MPZParams> path { get; set; }

        public Vertex(string k, bool lasStereo = false)
        {
            s = null;
            key = k;
            color = 0;
            value = 0;
            isVisited = false;

            lastStereoFrame = lasStereo;

            mark = -1;
            edges = null;
            path = new List<MPZParams>();
        }
        public Vertex(StaticConf ss,CaptureConf c, bool lasStereo = false)
        {
            s = ss;
            cs = c;
            route = new RouteParams(s);
            key = s.roll.ToString() + "/" + s.pitch.ToString();
            color = 0;
            value = countVertexPrice();
            isVisited = false;

            lastStereoFrame = lasStereo;

            mark = -1;
            edges = null;
            path = new List<MPZParams>();
        }

        public double countVertexPrice()
        {
            double sum = 0;

            if (s.type != WorkingType.Removal)
            {
             int[] pr_coef = new int[]{ 10, 100, 1000};

             double angle_coeff = 1 - Math.Acos(Math.Cos(s.pitch) * Math.Cos(s.roll)) * 180 /(100* Math.PI);// при съемке в надир = 1, при съемке с углом 55 = 0.45
             foreach(Order o in s.orders)
             {
                sum += s.square * o.intersection_coeff*pr_coef[o.request.priority-1];
             }
             return angle_coeff*sum;
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

