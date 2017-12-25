﻿using System;
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
        // The n-th apex corresponds to the (n, n+1) arc (including (-1, 0)).
        // len(vertices) == len(apexes)
        protected List<Vector3D> apexes;
        protected List<Vector3D> vertices;
        protected bool isCounterclockwise;

        public bool IsEmpty { get { return vertices.Count == 0; } }
        public IEnumerable<Vector3D> Apexes { get { return apexes; } }
        public IEnumerable<Vector3D> Vertices { get { return vertices; } }
        public IList<Arc> Arcs
        {
            get
            {
                var arcs = new List<Arc>();
                for (int i = 0; i < vertices.Count; ++i)
                    arcs.Add(new Arc(vertices[i], vertices[(i + 1) % vertices.Count], apexes[i]));
                return arcs;
            }
        }
        public Vector3D Middle
        {
            get
            {
                if (IsEmpty)
                    throw new DivideByZeroException("Polygon is empty.");
                return vertices.Aggregate((prev, cur) => prev + cur) / vertices.Count;
            }
        }
        public bool IsCounterclockwise { get { return isCounterclockwise; } }

        public Polygon()
        {
            apexes = new List<Vector3D>();
            vertices = new List<Vector3D>();
            isCounterclockwise = false;
        }
        public Polygon(ICollection<Vector3D> vertices, ICollection<Vector3D> apexes)
            : this()
        {
            if (vertices.Count != apexes.Count)
                throw new ArgumentException("Number of points and arcs is incosistent.");

            foreach (var point_apex in vertices.Zip(apexes, (point, apex) => Tuple.Create(point, apex)))
            {
                this.vertices.Add(point_apex.Item1);
                this.apexes.Add(point_apex.Item2);
            }
            isCounterclockwise = CheckIfCounterclockwise();
        }
        public Polygon(IEnumerable<Vector3D> vertices, Vector3D apex)
            : this()
        {
            foreach (var point in vertices)
            {
                this.apexes.Add(apex);
                this.vertices.Add(point);
            }
            isCounterclockwise = CheckIfCounterclockwise();
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
            isCounterclockwise = CheckIfCounterclockwise();
        }

        // Assume that polygon does not contain poles and does not cross the 180 meridian
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

            return GeoRect.Inflate(new GeoRect(minLon, maxLon, minLat, maxLat), 1 + 1e-3, 1 + 1e-3);
        }

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

        public bool Contains(Vector3D point)
        {
            if (vertices.Count == 0 || Vector3D.DotProduct(Middle, point) < 0)
                return false;

            // Ray casting algorithm. Shoot a great semiarc and count intersections.
            // Shoot an arc
            Random rand = new Random();
            int res = 0;
            for (int i = 0; i < 5; ++i)
            {
                var displacement = new Vector3D((rand.NextDouble() - 0.5) * 1e-1,
                                                (rand.NextDouble() - 0.5) * 1e-1,
                                                (rand.NextDouble() - 0.5) * 1e-1);
                var point2 = displacement - point;
                point2.Normalize();
                Arc halfGreat = new Arc(point, point2);

                int numOfIntersections = 0;
                foreach (Arc arc in Arcs)
                    numOfIntersections += Arc.Intersect(arc, halfGreat).Count;
                res += (numOfIntersections % 2);
            }
            return res >= 2;
        }

        public void ToThisFrame(ReferenceFrame frame)
        {
            vertices = vertices.Select(v => frame.ToThisFrame(v)).ToList();
            apexes = apexes.Select(a => frame.ToThisFrame(a)).ToList();
        }

        public void FromThisFrame(ReferenceFrame frame)
        {
            vertices = vertices.Select(v => frame.ToBaseFrame(v)).ToList();
            apexes = apexes.Select(a => frame.ToBaseFrame(a)).ToList();
        }

        public static Tuple<IList<Polygon>, IList<Polygon>> IntersectAndSubtract(Polygon p, Polygon q)
        {
            var intersection = new List<Polygon>();
            var difference = new List<Polygon>();
            var pArcs = p.Arcs;
            var qArcs = q.Arcs;

            foreach (var pArc in pArcs)
                foreach (var qArc in qArcs)
                    Arc.UpdateWithIntersections(pArc, qArc);

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

            var vert = new IList<Vector3D>[2] { pChain.Vertices, qChain.Vertices };
            var type = new IList<pointType>[2] { pChain.Labels, qChain.Labels };
            var apex = new IList<Vector3D>[2] { pChain.Apexes, qChain.Apexes };

            bool pInQ = q.Contains(vert[0][0]);
            pointType curType = pInQ ? pointType.exit : pointType.enter;
            for (int i = 0; i < vert[0].Count; ++i)
                if (type[0][i] != pointType.vertex)
                {
                    type[0][i] = curType;
                    curType = Not(curType);
                }

            bool qInP = p.Contains(vert[1][0]);
            curType = qInP ? pointType.exit : pointType.enter;
            for (int i = 0; i < vert[1].Count; ++i)
                if (type[1][i] != pointType.vertex)
                {
                    type[1][i] = curType;
                    curType = Not(curType);
                }

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
            Char separator = System.Globalization.CultureInfo.CurrentCulture.NumberFormat.CurrencyDecimalSeparator[0];

            if (0 == vertices.Count)
                return "";

            string wkt = "POLYGON ((";

            if (IsCounterclockwise)
            {
                foreach (var ver in vertices)
                {
                    GeoPoint gpoint = GeoPoint.FromCartesian(ver);
                    wkt += gpoint.Longitude.ToString().Replace(separator, '.') + " " + gpoint.Latitude.ToString().Replace(separator, '.') + ",";
                }
                GeoPoint firstpoint = GeoPoint.FromCartesian(vertices[0]);
                wkt += firstpoint.Longitude.ToString().Replace(separator, '.') + " " + firstpoint.Latitude.ToString().Replace(separator, '.') + "))";
            }
            else
            {
                wkt = "POLYGON ((";
                for (int i = vertices.Count - 1; i >= 0; i--)
                {
                    GeoPoint gpoint = GeoPoint.FromCartesian(vertices[i]);
                    wkt += gpoint.Longitude.ToString().Replace(separator, '.') + " " + gpoint.Latitude.ToString().Replace(separator, '.') + ",";
                }
                GeoPoint firstpoint = GeoPoint.FromCartesian(vertices[vertices.Count - 1]);
                wkt += firstpoint.Longitude.ToString().Replace(separator, '.') + " " + firstpoint.Latitude.ToString().Replace(separator, '.') + "))";
            }

            return wkt;
        }

        public double Square()
        {
            //var wtkstr = ToWtk();
            //SqlGeography geom = SqlGeography.STGeomFromText(new SqlChars(wtkstr), 4326);            
            //return (double)geom.STArea();
            return Area();
        }

        private bool CheckIfCounterclockwise()
        {
            if (vertices.Count < 3)
                return false;

            double t = 1;
            Vector3D pointInside;
            bool inside = false;

            Vector3D tangent1 = vertices[2] - vertices[1];
            tangent1.Normalize();
            Vector3D tangent2 = vertices[0] - vertices[1];
            tangent2.Normalize();
            Vector3D dir = tangent1 + tangent2;

            do
            {
                t /= 2;
                pointInside = vertices[1] + t * dir;
                pointInside.Normalize();
                inside = Contains(pointInside);
            } while (t > 1e-20 && !inside);

            if (!inside)
            {
                t = -1;
                do
                {
                    t /= 2;
                    pointInside = vertices[1] + t * dir;
                    pointInside.Normalize();
                    inside = Contains(pointInside);
                } while (t < -1e-20 && !inside);
            }

            if (!inside)
                throw new Exception("Too thin a polygon.");

            // If here, pointInside is indeed inside
            double angleSum = 0;
            Vector3D v1, v2, cross;
            for (int i = 0; i < vertices.Count - 1; ++i)
            {
                v1 = vertices[i] - pointInside;
                v2 = vertices[i + 1] - pointInside;
                v1.Normalize();
                v2.Normalize();
                cross = Vector3D.CrossProduct(v1, v2);
                angleSum += (Vector3D.DotProduct(cross, pointInside) > 0) ? cross.Length : -cross.Length;
            }
            v1 = vertices[vertices.Count - 1] - pointInside;
            v2 = vertices[0] - pointInside;
            v1.Normalize();
            v2.Normalize();
            cross = Vector3D.CrossProduct(v1, v2);
            angleSum += (Vector3D.DotProduct(cross, pointInside) > 0) ? cross.Length : -cross.Length;
            return angleSum > 0;
        }

        private double Area()
        {
            var arcs = Arcs;
            double angleSum = 0;
            double sin, cos, ang;

            for (int i = 0; i < arcs.Count - 1; ++i)
            {
                sin = Vector3D.DotProduct(Vector3D.CrossProduct(arcs[i + 1].TangentA, arcs[i].TangentB), arcs[i].B);
                cos = Vector3D.DotProduct(arcs[i].TangentB, arcs[i + 1].TangentA);
                ang = Math.Atan2(sin, cos);
                if (IsCounterclockwise && ang < 0)
                    ang += 2 * Math.PI;
                else if (!IsCounterclockwise && ang > 0)
                    ang -= 2 * Math.PI;
                angleSum += ang;
            }
            sin = Vector3D.DotProduct(Vector3D.CrossProduct(arcs[0].TangentA, arcs[arcs.Count - 1].TangentB), arcs[0].A);
            cos = Vector3D.DotProduct(arcs[arcs.Count - 1].TangentB, arcs[0].TangentA);
            ang = Math.Atan2(sin, cos);
            if (IsCounterclockwise && ang < 0)
                ang += 2 * Math.PI;
            else if (!IsCounterclockwise && ang > 0)
                ang -= 2 * Math.PI;
            angleSum += ang;

            double area;
            if (IsCounterclockwise)
                area = angleSum - (vertices.Count - 2) * Math.PI;
            else
                area = angleSum + (vertices.Count - 2) * Math.PI;
            return Math.Abs(area);
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
        private static int NextUnvisitedIntersection(IList<pointType> type, IList<bool> visited)
        {
            int ind = 0;
            while ((ind < type.Count) && ((type[ind] == pointType.vertex) || visited[ind]))
                ++ind;
            return ind == type.Count ? -1 : ind;
        }
        private static IList<Polygon> Traverse(IList<Vector3D>[] vert,
                                               IList<pointType>[] type,
                                               IList<Vector3D>[] apex,
                                               bool[] traverseForward)
        {
            var resPoly = new List<Polygon>();
            var visited = new List<bool>[2] { vert[0].Select(v => false).ToList(), vert[1].Select(v => false).ToList() };
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
    }
}
