using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using Microsoft.SqlServer.Types;
using System.Data.SqlTypes;

using Common;

namespace SphericalGeom
{
    public class Polygon
    {
        #region Polygon private fields
        private static double rotationAngleIfOverlap = 1; // degrees

        // The n-th apex corresponds to the (n, n+1) arc (including (-1, 0)).
        // len(vertices) == len(apexes)
        private List<Vector3D> apexes;
        private List<Vector3D> vertices;

        private static Random rand;

        private bool knowArcs;
        private List<Arc> arcs;

        private bool knowWtk;
        private string wtk;

        private bool knowArea;
        private double area;

        private bool knowCounterclockwise;
        private bool counterclockwise;
        private bool knowAreaOrientation;

        private bool knowMiddle;
        private Vector3D middle;
        #endregion

        #region Polygon properties
        /// <summary>
        /// Whether has zero vertices.
        /// </summary>
        public bool IsEmpty { get { return vertices.Count == 0; } }
        /// <summary>
        /// Number of vertices (and apexes).
        /// </summary>
        public int Count { get { return vertices.Count; } }

        public IList<Vector3D> Apexes { get { return apexes; } }
        public IList<Vector3D> Vertices { get { return vertices; } }
        public IList<Arc> Arcs 
        {
            get
            {
                if (!knowArcs)
                {
                    arcs.Clear();
                    for (int i = 0; i < Count; ++i)
                        arcs.Add(new Arc(vertices[i], vertices[(i + 1) % Count], apexes[i]));
                    knowArcs = true;
                }
                return arcs;
            }
        }

        public double Area
        {
            get
            {
                if (!knowAreaOrientation)
                    SetAreaOrientation();
                return area;
                //if (!knowArea)
                //{
                //    area = 0;
                //    if (Count > 2)
                //    {
                //        double ang;
                //        for (int i = 0; i < Count; ++i)
                //        {
                //            ang = RotationAngleWithNextArc(i);
                //            if (!IsCounterclockwise)
                //                ang *= -1;
                //            area += ang;
                //            if (IsCounterclockwise == arcs[i].Counterclockwise)
                //                area += arcs[i].CentralAngle * arcs[i].Radius * arcs[i].GeodesicCurvature;
                //            else
                //                area -= arcs[i].CentralAngle * arcs[i].Radius * arcs[i].GeodesicCurvature;
                //        }
                //        area = 2 * Math.PI - area;
                //        //area = Math.Min(2 * Math.PI - area, 2 * Math.PI + area);
                //    }
                //    knowArea = true;
                //}
                //return area;
            }
        }
        public Vector3D Middle
        {
            get
            {
                if (!knowMiddle)
                {
                    if (IsEmpty)
                        throw new DivideByZeroException("Polygon is empty.");

                    middle = vertices.Aggregate((prev, cur) => prev + cur) / vertices.Count;
                    middle.Normalize();
                    knowMiddle = true;
                }
                return middle;
            }
            private set // USE WITH CARE ONLY FOR BIG POLYS LIKE HEMISPHERES
            {
                knowMiddle = true;
                middle = value;
            }
        }
        public bool IsCounterclockwise
        {
            get
            {
                if (!knowAreaOrientation)
                    SetAreaOrientation();
                return counterclockwise;
                //if (!knowCounterclockwise)
                //{
                //    if (Count < 3)
                //        // Convention.
                //        counterclockwise = true;
                //    else
                //    {
                //        /// Pick a vertex.
                //        /// We want to choose a search direction and check, whether
                //        /// a semiarc from the vertex in the direction intersects the polygon's interior.
                //        /// Two distinct cases exist:
                //        /// 
                //        /// 1. If there is a nonzero rotation at the vertex,
                //        /// then take the angle's bisector.
                //        /// 2. If there is no rotation (i.e. curve is smooth at the vertex),
                //        /// we pick CrossProduct(tangent, normal).
                //        double exteriorAngle = RotationAngleWithNextArc(0);

                //        Vector3D dir;
                //        bool rotation = !Comparison.IsZero(exteriorAngle);
                //        if (rotation)
                //            dir = Arcs[1].TangentA - Arcs[0].TangentB;
                //        else
                //            dir = Vector3D.CrossProduct(Arcs[0].TangentB, vertices[1]);
                //        dir.Normalize();
                //        /// 1e-10 precision is more than enough for real Earth regions
                //        /// (corresponds to cm)
                //        double t = 1e-10;
                //        Vector3D pointInside = vertices[1] + t * dir;
                //        pointInside.Normalize();
                //        bool inside = Contains(pointInside);
                //        if (rotation)
                //            counterclockwise = Comparison.IsPositive(exteriorAngle) == inside;
                //        else
                //            counterclockwise = inside;
                //    }
                //    knowCounterclockwise = true;
                //}
                //return true;// counterclockwise;
            }
        }
        #endregion

        #region Polygon constructors
        public Polygon()
        {
            apexes = new List<Vector3D>();
            vertices = new List<Vector3D>();
            arcs = new List<Arc>();
            rand = new Random();
            knowWtk = false;
            knowArea = false;
            knowCounterclockwise = false;
            knowAreaOrientation = false;
            knowMiddle = false;
        }
        public Polygon(IEnumerable<Vector3D> vertices, IEnumerable<Vector3D> apexes)
            : this()
        {
            foreach (Vector3D vertex in vertices)
                this.vertices.Add(vertex);
            foreach (Vector3D apex in apexes)
                this.apexes.Add(apex);

            if (this.vertices.Count != this.apexes.Count)
                throw new ArgumentException("Number of points and arcs is incosistent.");
        }
        public Polygon(IEnumerable<Vector3D> vertices, Vector3D apex)
            : this()
        {
            foreach (Vector3D vertex in vertices)
                this.vertices.Add(vertex);
            for (int i = 0; i < this.vertices.Count; ++i)
                apexes.Add(apex);
        }
        public Polygon(IEnumerable<Vector3D> vertices) : this(vertices, new Vector3D(0, 0, 0)) { }
        public Polygon(GeoRect rect) : this(rect.Points.Select(gp => GeoPoint.ToCartesian(gp, 1.0))) { }

        public Polygon(string wtkPolygon)
            : this()
        {
            SqlGeography geom = SqlGeography.STGeomFromText(new SqlChars(wtkPolygon), 4326);
            List<Vector3D> points = new List<Vector3D>();
            var apex = new Vector3D(0, 0, 0);
            for (int i = 1; i < geom.STNumPoints(); i++)
            {
                double lat = (double)geom.STPointN(i).Lat;
                double lon = (double)geom.STPointN(i).Long;
                Vector3D point = GeoPoint.ToCartesian(new GeoPoint(lat, lon), 1);
                this.apexes.Add(apex);
                this.vertices.Add(point);
            }
        }
        #endregion

        public static Polygon Hemisphere(Vector3D capCenter)
        {
            double ang = 1e-12, c = Math.Cos(ang), s = Math.Sin(ang);
            Polygon basic = new Polygon(new List<Vector3D>{
                new Vector3D(c, 0, s),
                new Vector3D(0, c, s),
                new Vector3D(-c, 0, s),
                new Vector3D(0, -c, s)
            });

            Vector3D basisVec = Vector3D.CrossProduct(capCenter,
                new Vector3D(rand.NextDouble(), rand.NextDouble(), rand.NextDouble()));
            ReferenceFrame capFrame = new ReferenceFrame(
                basisVec,
                Vector3D.CrossProduct(basisVec, capCenter),
                capCenter);
            basic.FromThisFrame(capFrame);
            return basic;
        }

        /// <summary>
        /// Builds a rectangular bounding box for the polygon,
        /// assuming that it does not contain poles and does not cross the 180 meridian.
        /// </summary>
        /// <returns></returns>
        public GeoRect BoundingBox()
        {
            double maxLat = -90.0, minLat = 90.0, maxLon = -180.0, minLon = 180.0;
            foreach (Arc arc in Arcs)
            {
                maxLat = Math.Max(maxLat, arc.MaxLatitudeDeg());
                minLat = Math.Min(minLat, arc.MinLatitudeDeg());
                maxLon = Math.Max(maxLon, arc.MaxLongitudeDeg());
                minLon = Math.Min(minLon, arc.MinLongitudeDeg());
            }

            //return new GeoRect(minLon, maxLon, minLat, maxLat);
            return GeoRect.Inflate(new GeoRect(minLon, maxLon, minLat, maxLat), 1 + 1e-3, 1 + 1e-3);
        }

        /// <summary>
        /// Breaks the polygon into Count triangles.
        /// Each triangle consits of two consecutive polygon vertices and <paramref name="mid"/>.
        /// </summary>
        /// <param name="mid"></param>
        /// <returns></returns>
        public IList<Polygon> Shatter(Vector3D mid)
        {
            var res = new List<Polygon>();
            res.Add(new Polygon(new List<Vector3D> { mid, vertices[vertices.Count - 1], vertices[0] },
                                new List<Vector3D> { new Vector3D(0, 0, 0), apexes[apexes.Count - 1], new Vector3D(0, 0, 0) }));
            for (int i = 0; i < vertices.Count - 1; ++i)
                res.Add(new Polygon(new List<Vector3D> { mid, vertices[i], vertices[i + 1] },
                                    new List<Vector3D> { new Vector3D(0, 0, 0), apexes[i], new Vector3D(0, 0, 0) }));
            return res;
        }

        public List<Polygon> BreakIntoLobes()
        {
            Polygon[] lobes = new Polygon[4];

            double h = 1e-2, z = Math.Sqrt(1 - 2*h*h);
            Vector3D N = new Vector3D(Math.Sqrt(2)*h, 0, z);
            Vector3D S = new Vector3D(Math.Sqrt(2) * h, 0, -z);
            Vector3D X = new Vector3D(1, 0, 0);
            Vector3D Yp = new Vector3D(0, 1, 0);
            Vector3D Ym = new Vector3D(0, -1, 0);
            Vector3D N_Yp = new Vector3D(-h, h, z);
            Vector3D N_Ym = new Vector3D(-h, -h, z);
            Vector3D S_Yp = new Vector3D(-h, h, -z);
            Vector3D S_Ym = new Vector3D(-h, -h, -z);
            Vector3D Xm_Yp = new Vector3D(-Math.Sqrt(1 - h * h), h, 0);
            Vector3D Xm_Ym = new Vector3D(-Math.Sqrt(1 - h * h), -h, 0);

            lobes[0] = new Polygon(
                new List<Vector3D> { N, Yp, S, S_Yp, Xm_Yp, N_Yp },
                new List<Vector3D> 
                { 
                    new Vector3D(),
                    new Vector3D(),
                    new Vector3D(0, 0, -z),
                    new Vector3D(0, h, 0),
                    new Vector3D(0, h, 0),
                    new Vector3D(0, 0, z)
                });
            lobes[1] = new Polygon(
                new List<Vector3D> { N, Ym, S, S_Ym, Xm_Ym, N_Ym },
                new List<Vector3D>
                { 
                    new Vector3D(),
                    new Vector3D(),
                    new Vector3D(0, 0, -z),
                    new Vector3D(0, -h, 0),
                    new Vector3D(0, -h, 0),
                    new Vector3D(0, 0, z)
                });
            lobes[2] = new Polygon(new List<Vector3D> { N, X, S, Yp });
            lobes[3] = new Polygon(new List<Vector3D> { N, X, S, Ym });

            List<Polygon> res = new List<Polygon>();
            for (int i = 0; i < lobes.Length; ++i)
                res.AddRange(Polygon.Intersect(this, lobes[i]));

            return res;
        }

        public bool Contains(Vector3D point)
        {
            if (vertices.Count < 3 || Vector3D.DotProduct(Middle, point) < 0)
                return false;

            // Is point on the boundary?
            for (int i = 0; i < Count; ++i)
                if (Arcs[i].Contains(point))
                    return true;

            // Ray casting algorithm.
            Arc halfGreat;
            Vector3D displacement, point2;
            bool degenerate;

            /// Shoot a random arc such that it doesn't cross any polygon vertex
            /// nor does it begin on an edge. 
            /// Such arc exists, because point is not on the boundary.
            do
            {
                displacement = new Vector3D((rand.NextDouble() - 0.5) * 1e-1,
                                            (rand.NextDouble() - 0.5) * 1e-1,
                                            (rand.NextDouble() - 0.5) * 1e-1);
                point2 = displacement - point;
                point2.Normalize();
                halfGreat = new Arc(point, point2);

                degenerate = false;
                for (int i = 0; i < Count; ++i)
                    degenerate |= halfGreat.Contains(vertices[i]);
            } while (degenerate);

            // Count intersections.
            int numOfIntersections = 0;
            for (int j = 0; j < Arcs.Count; ++j)
                numOfIntersections += Arc.Intersect(Arcs[j], halfGreat).ToList().Count;
            return (numOfIntersections % 2) == 1;

            //int res = 0;
            //for (int i = 0; i < 10; ++i)
            //{
            //    var displacement = new Vector3D((rand.NextDouble() - 0.5) * 1e-1,
            //                                    (rand.NextDouble() - 0.5) * 1e-1,
            //                                    (rand.NextDouble() - 0.5) * 1e-1);
            //    var point2 = displacement - point;
            //    point2.Normalize();
            //    Arc halfGreat = new Arc(point, point2);

            //    int numOfIntersections = 0;
            //    for (int j = 0; j < Arcs.Count; ++j)
            //        numOfIntersections += Arc.Intersect(Arcs[j], halfGreat).ToList().Count;
            //    res += (numOfIntersections % 2);
            //}
            //return res >= 7;
        }

        /// <summary>
        /// Transforms vertices and apexes to the given frame.
        /// </summary>
        /// <param name="frame"></param>
        public void ToThisFrame(ReferenceFrame frame)
        {
            for (int i = 0; i < Count; ++i)
            {
                vertices[i] = frame.ToThisFrame(vertices[i]);
                apexes[i] = frame.ToThisFrame(apexes[i]);
            }
            // Force Arcs recomputation if needed.
            knowArcs = false;
        }

        /// <summary>
        /// Transforms vertices and apexes from the given frame.
        /// </summary>
        /// <remarks>Arcs are not recomputed!</remarks>
        /// <param name="frame"></param>
        public void FromThisFrame(ReferenceFrame frame)
        {
            for (int i = 0; i < Count; ++i)
            {
                vertices[i] = frame.ToBaseFrame(vertices[i]);
                apexes[i] = frame.ToBaseFrame(apexes[i]);
            }
            // Force Arcs recomputation if needed.
            knowArcs = false;
        }

        public static IList<Polygon> Intersect(Polygon p, Polygon q)
        {
            var intersection = new List<Polygon>();

            List<Vector3D>[] vert;
            List<pointType>[] type;
            List<Vector3D>[] apex;
            bool pInQ, qInP;

            PrepareForClipping(p, q, out vert, out type, out apex, out pInQ, out qInP);

            // No arc intersections, i.e. one is within the other or do not overlap at all
            if (type[0].All(t => t == pointType.vertex))
            {
                if (pInQ)
                    intersection.Add(p);
                else if (qInP)
                    intersection.Add(q);
            }
            else
                intersection.AddRange(Traverse(vert, type, apex, new bool[2] { true, true }));

            return intersection;
        }

        public static Tuple<IList<Polygon>, IList<Polygon>> IntersectAndSubtract(Polygon p, Polygon q)
        {
            var intersection = new List<Polygon>();
            var difference = new List<Polygon>();

            List<Vector3D>[] vert;
            List<pointType>[] type;
            List<Vector3D>[] apex;
            bool pInQ, qInP;

            PrepareForClipping(p, q, out vert, out type, out apex, out pInQ, out qInP);

            // No arc intersections, i.e. one is within the other or do not overlap at all
            if (type[0].All(t => t == pointType.vertex))
            {
                if (pInQ)
                    intersection.Add(p);
                else if (qInP)
                {
                    intersection.Add(q);
                    var pieces = p.Shatter(q.Middle);
                    foreach (var piece in pieces)
                    {
                        var intersectionAndDifference = Polygon.IntersectAndSubtract(piece, q);
                        //intersection.AddRange(intersectionAndDifference.Item1);
                        difference.AddRange(intersectionAndDifference.Item2);
                    }
                }
                else
                    difference.Add(p);
            }
            else
            {
                intersection.AddRange(Traverse(vert, type, apex, new bool[2] { true, true }));
                difference.AddRange(Traverse(vert, type, apex, new bool[2] { false, true }));
            }
            return new Tuple<IList<Polygon>, IList<Polygon>>(intersection, difference);
        }

        public string ToWtk()
        {
            if (!knowWtk)
            {
                Char separator = System.Globalization.CultureInfo.CurrentCulture.NumberFormat.CurrencyDecimalSeparator[0];

                if (0 == vertices.Count)
                    wtk = "";
                else
                {
                    wtk = "POLYGON ((";
                    if (IsCounterclockwise)
                    {
                        foreach (var ver in vertices)
                        {
                            GeoPoint gpoint = GeoPoint.FromCartesian(ver);
                            wtk += gpoint.Longitude.ToString().Replace(separator, '.') + " " + gpoint.Latitude.ToString().Replace(separator, '.') + ",";
                        }
                        GeoPoint firstpoint = GeoPoint.FromCartesian(vertices[0]);
                        wtk += firstpoint.Longitude.ToString().Replace(separator, '.') + " " + firstpoint.Latitude.ToString().Replace(separator, '.') + "))";
                    }
                    else
                    {
                        for (int i = vertices.Count - 1; i >= 0; i--)
                        {
                            GeoPoint gpoint = GeoPoint.FromCartesian(vertices[i]);
                            wtk += gpoint.Longitude.ToString().Replace(separator, '.') + " " + gpoint.Latitude.ToString().Replace(separator, '.') + ",";
                        }
                        GeoPoint firstpoint = GeoPoint.FromCartesian(vertices[vertices.Count - 1]);
                        wtk += firstpoint.Longitude.ToString().Replace(separator, '.') + " " + firstpoint.Latitude.ToString().Replace(separator, '.') + "))";
                    }
                }
                knowWtk = true;
            }
            return wtk;
        }
 
        #region Polygon private methods
        private void SetAreaOrientation()
        {
            area = 0;
            // Compute area as if counterclockwise.
            for (int i = 0; i < Count; ++i)
            {
                area += RotationAngleWithNextArc(i);
                if (arcs[i].Counterclockwise)
                    area += arcs[i].CentralAngle * arcs[i].Radius * arcs[i].GeodesicCurvature;
                else
                    area -= arcs[i].CentralAngle * arcs[i].Radius * arcs[i].GeodesicCurvature;
            }
            area = 2 * Math.PI - area;

            if (Comparison.IsSmaller(area, 2 * Math.PI))
                counterclockwise = true;
            else
            {
                area = 4 * Math.PI - area;
                counterclockwise = false;
            }
            knowAreaOrientation = true;
        }

        private double RotationAngleWithNextArc(int curArcInd)
        {
            int nextArcInd = (curArcInd == Count - 1) ? 0 : (curArcInd + 1);
            double sin = Vector3D.DotProduct(
                Vector3D.CrossProduct(Arcs[curArcInd].TangentB, Arcs[nextArcInd].TangentA),
                Arcs[nextArcInd].A);
            double cos = Vector3D.DotProduct(Arcs[curArcInd].TangentB, Arcs[nextArcInd].TangentA);
            return Math.Atan2(sin, cos);
        }

        private enum pointType { vertex, enter, exit };
        private static pointType Not(pointType t)
        {
            switch (t)
            {
                case pointType.enter:
                    return pointType.exit;
                case pointType.exit:
                    return pointType.enter;
                case pointType.vertex:
                    return pointType.vertex;
                default:
                    return pointType.vertex;
            }
        }
        private static int Dir(pointType t, bool traverseForward)
        {
            switch (t)
            {
                case pointType.enter:
                    return traverseForward ? 1 : -1;
                case pointType.exit:
                    return traverseForward ? -1 : 1;
                case pointType.vertex:
                    return 0;
                default:
                    return 0;
            }
        }
        private static int NextUnvisitedIntersection(List<pointType> type, List<bool> visited)
        {
            int ind = 0;
            while ((ind < type.Count) && ((type[ind] == pointType.vertex) || visited[ind]))
                ++ind;
            return ind == type.Count ? -1 : ind;
        }
        private static IList<Polygon> Traverse(List<Vector3D>[] vert,
                                               List<pointType>[] type,
                                               List<Vector3D>[] apex,
                                               bool[] traverseForward)
        {
            var resPoly = new List<Polygon>();
            var visited = new List<bool>[2];
            visited[0] = new List<bool>();
            visited[1] = new List<bool>();
            for (int i = 0; i < vert[0].Count; ++i)
                visited[0].Add(false);
            for (int i = 0; i < vert[1].Count; ++i)
                visited[1].Add(false);
            var resVert = new List<Vector3D>();
            var resApex = new List<Vector3D>();

            int curChain = 0, dir, curInd, newInd;
            curInd = NextUnvisitedIntersection(type[curChain], visited[curChain]);
            while (curInd != -1)
            {
                resVert.Add(vert[curChain][curInd]);
                visited[curChain][curInd] = true;
                visited[(curChain + 1) % 2][vert[(curChain + 1) % 2].IndexOf(vert[curChain][curInd])] = true;
                dir = Dir(type[curChain][curInd], traverseForward[curChain]);
                newInd = (curInd + dir + visited[curChain].Count) % visited[curChain].Count;

                while (!visited[curChain][newInd])
                {
                    resApex.Add(apex[curChain][dir > 0 ? curInd : newInd]);
                    curInd = newInd;
                    visited[curChain][curInd] = true;
                    resVert.Add(vert[curChain][curInd]);

                    if (type[curChain][curInd] != pointType.vertex)
                    {
                        // Slow search!
                        curInd = vert[(curChain + 1) % 2].IndexOf(vert[curChain][curInd]);
                        curChain = (curChain + 1) % 2;
                        visited[curChain][curInd] = true;
                        dir = Dir(type[curChain][curInd], traverseForward[curChain]);
                    }

                    newInd = (curInd + dir + visited[curChain].Count) % visited[curChain].Count;
                }
                resApex.Add(apex[curChain][dir > 0 ? curInd : newInd]);

                resPoly.Add(new Polygon(resVert, resApex));
                resVert.Clear();
                resApex.Clear();
                curChain = 0;
                curInd = NextUnvisitedIntersection(type[curChain], visited[curChain]);
            }
            return resPoly;
        }
        private static void PrepareForClipping(Polygon p, Polygon q, out List<Vector3D>[] vert, out List<pointType>[] type, out List<Vector3D>[] apex, out bool pInQ, out bool qInP)
        {
            var pArcs = p.Arcs;
            var qArcs = q.Arcs;

            ReferenceFrame rotate = new ReferenceFrame();
            rotate.RotateBy(q.Middle, rotationAngleIfOverlap);
            int rotationCount = 0;
            bool allGood = false;
            while (!allGood)
            {
                try
                {
                    foreach (var pArc in pArcs)
                        foreach (var qArc in qArcs)
                            Arc.UpdateWithIntersections(pArc, qArc);
                    allGood = true;
                }
                catch (ArgumentException ex)
                {
                    p.knowArcs = false;
                    q.knowArcs = false;

                    q.ToThisFrame(rotate);
                    rotationCount++;

                    pArcs = p.Arcs;
                    qArcs = q.Arcs;
                }
            }

            ArcChainWithLabels<pointType> pChain = pArcs.
                Select(arc => arc.EmplaceIntermediatePoints<pointType>(pointType.vertex, pointType.enter)).
                Aggregate((head, tail) => head.Connect(tail));
            pChain.Vertices.RemoveAt(pChain.Vertices.Count - 1);
            pChain.Labels.RemoveAt(pChain.Labels.Count - 1);

            ArcChainWithLabels<pointType> qChain = qArcs.
                Select(arc => arc.EmplaceIntermediatePoints<pointType>(pointType.vertex, pointType.enter)).
                Aggregate((head, tail) => head.Connect(tail));
            qChain.Vertices.RemoveAt(qChain.Vertices.Count - 1);
            qChain.Labels.RemoveAt(qChain.Labels.Count - 1);

            vert = new List<Vector3D>[2] { pChain.Vertices, qChain.Vertices };
            type = new List<pointType>[2] { pChain.Labels, qChain.Labels };
            apex = new List<Vector3D>[2] { pChain.Apexes, qChain.Apexes };

            pInQ = q.Contains(vert[0][0]);
            pointType curType = pInQ ? pointType.exit : pointType.enter;
            for (int i = 0; i < vert[0].Count; ++i)
                if (type[0][i] != pointType.vertex)
                {
                    type[0][i] = curType;
                    curType = Not(curType);
                }

            qInP = p.Contains(vert[1][0]);
            curType = qInP ? pointType.exit : pointType.enter;
            for (int i = 0; i < vert[1].Count; ++i)
                if (type[1][i] != pointType.vertex)
                {
                    type[1][i] = curType;
                    curType = Not(curType);
                }

            if (rotationCount > 0)
            {
                rotate = new ReferenceFrame();
                rotate.RotateBy(q.Middle, rotationCount * rotationAngleIfOverlap);
                q.FromThisFrame(rotate);
                q.knowArcs = false;
            }
        }
        #endregion
    }
}
