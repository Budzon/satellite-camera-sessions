using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SphericalGeom
{
    public static class Comparison
    {
        private static double precision = 1e-7;

        public static bool IsPositive(double a)
        {
            return a > precision;
        }

        public static bool IsNegative(double a)
        {
            return a < -precision;
        }

        public static bool IsZero(double a)
        {
            return !IsPositive(a) && !IsNegative(a);
        }

        public static bool IsBigger(double a, double b)
        {
            return IsPositive(a - b);
        }

        public static bool IsSmaller(double a, double b)
        {
            return IsNegative(a - b);
        }
    }

    public class vector3
    {
        /* Properties */
        public double X
        {
            get; set;
        }
        public double Y
        {
            get; set;
        }
        public double Z
        {
            get; set;
        }

        /* Constructors */
        public vector3(double _x, double _y, double _z)
        {
            X = _x;
            Y = _y;
            Z = _z;
        }
        public vector3() : this(1, 1, 1) { }
        public vector3(direction3 direction, double length)
        {
            X = length * Math.Cos(direction.Lon) * Math.Cos(direction.Lat);
            Y = length * Math.Sin(direction.Lon) * Math.Cos(direction.Lat);
            Z = length * Math.Sin(direction.Lat);
        }

        /* Operators */
        public static vector3 operator+(vector3 a, vector3 b)
        {
            return new vector3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        }
        public static vector3 operator-(vector3 a, vector3 b)
        {
            return new vector3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        }
        public static vector3 operator*(double t, vector3 a)
        {
            return new vector3(t * a.X, t * a.Y, t * a.Z);
        }
        public static vector3 operator-(vector3 a)
        {
            return -1 * a;
        }
        public static double operator*(vector3 a, vector3 b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }
        public static double operator*(vector3 a, direction3 b)
        {
            return a * (new vector3(b, 1));
        }

        public vector3 Cross(vector3 a)
        {
            return new vector3(Y * a.Z - Z * a.Y,
                               Z * a.X - X * a.Z,
                               X * a.Y - Y * a.X);
        }

        /* Methods */
        public double Length()
        {
            return Math.Sqrt(X * X + Y * Y + Z * Z);
        }
        public direction3 Normalized()
        {
            double lat = Math.Asin(Z / Length());
            double lon = Math.Atan2(Y, X);// + (Comparison.IsPositive(X) ? 0 : Math.PI);
            return new direction3(lat, lon);
        }
        public override string ToString()
        {
            return Normalized().ToString();
        }
    }

    public class direction3
    {
        private double lon, lat;

        /* Properties */
        public double Lon {
            get { return lon; }
            set
            {
                while (Comparison.IsBigger(value, Math.PI))
                    value -= 2 * Math.PI;
                while (Comparison.IsSmaller(value, -Math.PI))
                    value += 2 * Math.PI;

                lon = value;
            }
        }
        public double Lat {
            get { return lat; }
            set
            {
                bool needShiftLon = false;

                while (Comparison.IsBigger(value, Math.PI))
                {
                    value -= 2 * Math.PI;
                }
                while (Comparison.IsSmaller(value, -Math.PI))
                {
                    value += 2 * Math.PI;
                }

                if (Comparison.IsBigger(value, Math.PI / 2))
                {
                    value = Math.PI - value;
                    needShiftLon = true;
                }
                else if (Comparison.IsSmaller(value, -Math.PI / 2))
                {
                    value = -Math.PI - value;
                    needShiftLon = true;
                }
                lat = value;
                Lon = lon + (needShiftLon ? Math.PI : 0);
            }
        }

        /* Constructors */
        public direction3(double _lat, double _lon)
        {
            Lon = _lon;
            Lat = _lat;
        }
        public direction3() : this(0, 0) { }

        /* Operators */
        public static direction3 operator+(direction3 a, direction3 b)
        {
            return new direction3(a.Lat + b.Lat, a.Lon + b.Lon);
        }
        public static direction3 operator-(direction3 a, direction3 b)
        {
            return new direction3(a.Lat - b.Lat, a.Lon - b.Lon);
        }
        public static double operator*(direction3 a, direction3 b)
        {
            return (new vector3(a, 1)) * (new vector3(b, 1));
        }

        /* Methods */
        public direction3 Antipodal()
        {
            return new direction3(-lat, lon + Math.PI);
        }
        public override string ToString()
        {
            return lat * 180 / Math.PI + ", " + lon * 180 / Math.PI;
        }
    }

    public class ReferenceFrame
    {
        /* Properties */
        public vector3 OX { get; private set; }
        public vector3 OY { get; private set; }
        public vector3 OZ { get; private set; }

        /* Constructors */
        public ReferenceFrame(vector3 _ox, vector3 _oy, vector3 _oz)
        {
            OX = (1 / _ox.Length()) * _ox;
            OY = (1 / _oy.Length()) * _oy;
            OZ = (1 / _oz.Length()) * _oz;
        }
        public ReferenceFrame(vector3 _oy)
            : this(new vector3(_oy.Y, -_oy.X, 0),
                    _oy,
                    new vector3(-_oy.X * _oy.Z, -_oy.Y * _oy.Z, _oy.X * _oy.X + _oy.Y * _oy.Y))
        { }
        public ReferenceFrame()
            : this(new vector3(1,0,0), new vector3(0,1,0), new vector3(0, 0, 1))
        { }

        /* Methods */
        public vector3 ToThisFrame(vector3 a)
        {
            return new vector3(
                OX.X * a.X + OX.Y * a.Y + OX.Z * a.Z,
                OY.X * a.X + OY.Y * a.Y + OY.Z * a.Z,
                OZ.X * a.X + OZ.Y * a.Y + OZ.Z * a.Z);
        }
        public vector3 ToThisFrame(double x, double y, double z)
        {
            return new vector3(
                OX.X * x + OX.Y * y + OX.Z * z,
                OY.X * x + OY.Y * y + OY.Z * z,
                OZ.X * x + OZ.Y * y + OZ.Z * z);
        }
        public vector3 ToBaseFrame(vector3 a)
        {
            return new vector3(
                OX.X * a.X + OY.X * a.Y + OZ.X * a.Z,
                OX.Y * a.X + OY.Y * a.Y + OZ.Y * a.Z,
                OX.Z * a.X + OY.Z * a.Y + OZ.Z * a.Z);
        }
        public vector3 ToBaseFrame(double x, double y, double z)
        {
            return new vector3(
                OX.X * x + OY.X * y + OZ.X * z,
                OX.Y * x + OY.Y * y + OZ.Y * z,
                OX.Z * x + OY.Z * y + OZ.Z * z);
        }
    }

    public class Camera
    {
        private static double sightAngleStep = 1e-1;
        private static double sightDirectionStep = 5e-3;
        // Right orthonormal basis with -position as OY such that OZY contains the North pole
        private vector3 position;
        private direction3 positionDirection;
        private ReferenceFrame camPositionFrame;

        /* Properties */
        public vector3 Position
        {
            get { return position; ; }
            set
            {
                position = value;
                positionDirection = value.Normalized();

                camPositionFrame = new ReferenceFrame(
                    new vector3(-value.Y, value.X, 0),
                    -value,
                    new vector3(-value.X * value.Z, -value.Y * value.Z, value.X * value.X + value.Y * value.Y));
            }
        }
        public direction3 PositionDirection
        { get { return positionDirection; } }

        public double verticalHalfAngleOfView { get; set; }
        public double horizontalHalfAngleOfView { get; set; }

        /* Constructors */
        public Camera(vector3 _position, double _verticalHalfAngleOfView, double _horizontalHalfAngleOfView)
        {
            Position = _position;
            verticalHalfAngleOfView = _verticalHalfAngleOfView;
            horizontalHalfAngleOfView = _horizontalHalfAngleOfView;
        }
        public Camera() : this(new direction3(), 1, Math.PI / 6, Math.PI / 6) { }
        public Camera(direction3 direction, double altitude, double _verticalHalfAngleOfView, double _horizontalHalfAngleOfView)
            : this(new vector3(direction, altitude + 1), _verticalHalfAngleOfView, _horizontalHalfAngleOfView)
        { }

        /* Methods */
        public Tuple<bool, direction3, double> RegionCanBeSeen(List<vector3> pointsOfInterest)
        {
            if (pointsOfInterest.All(point => CanBeSeenFromPosition(point)))
                foreach (double sightAngle in PossibleSightAngles())
                {
                    foreach (var sightDirection in PossibleSightDirections())
                    {
                        ReferenceFrame camSightFrame = new ReferenceFrame(sightDirection.Item1);
                        if (GetPositiveNormalsToSightBoundaries(camSightFrame, sightAngle).All(normal => RegionInPositiveHalfspace(normal, pointsOfInterest)))
                            return new Tuple<bool, direction3, double>(true, sightDirection.Item2, sightAngle);
                    }
                }

            return new Tuple<bool, direction3, double>(false, new direction3(), 0);
        }

        //public List<vector3> CaptureWhatIsSeen(ReferenceFrame camSightFrame, double sightAngle, List<vector3> region)
        //{
        //    var normals = GetPositiveNormalsToSightBoundaries(camSightFrame, sightAngle);

        //}

        /* Private methods */
        private IEnumerable<double> PossibleSightAngles()
        {
            double cur = 0;
            while (Comparison.IsSmaller(cur, Math.PI))
            {
                yield return cur;
                cur += sightAngleStep;
            }
        }
        private IEnumerable<Tuple<vector3, direction3>> PossibleSightDirections()
        {
            double maxAngle = Math.Asin(1 / Position.Length());
            double horizontalMaxAngle = maxAngle; //- horizontalHalfAngleOfView;
            double verticalMaxAngle = maxAngle; //- verticalHalfAngleOfView;

            double ch, cv, sh, sv;

            double verticalAngle = 0;
            double horizontalAngle;

            do
            {
                cv = Math.Cos(verticalAngle);
                sv = Math.Sin(verticalAngle);
                horizontalAngle = 0;
                do
                {
                    ch = Math.Cos(horizontalAngle);
                    sh = Math.Sin(horizontalAngle);

                    yield return new Tuple<vector3, direction3>(camPositionFrame.ToBaseFrame(sh * cv, ch * cv, sv), new direction3(verticalAngle, horizontalAngle));
                    yield return new Tuple<vector3, direction3>(camPositionFrame.ToBaseFrame(sh * cv, ch * cv, -sv), new direction3(-verticalAngle, horizontalAngle));
                    yield return new Tuple<vector3, direction3>(camPositionFrame.ToBaseFrame(-sh * cv, ch * cv, -sv), new direction3(-verticalAngle, -horizontalAngle));
                    yield return new Tuple<vector3, direction3>(camPositionFrame.ToBaseFrame(-sh * cv, ch * cv, sv), new direction3(verticalAngle, -horizontalAngle));

                    horizontalAngle += sightDirectionStep;
                } while (Comparison.IsSmaller(horizontalAngle, horizontalMaxAngle));
                verticalAngle += sightDirectionStep;
            } while (Comparison.IsSmaller(verticalAngle, verticalMaxAngle));
        }

        private bool CanBeSeenFromPosition(vector3 a)
        {
            return Comparison.IsBigger(Position * a, a.Length());
        }
        private List<vector3> GetPositiveNormalsToSightBoundaries(ReferenceFrame camSightFrame, double sightAngle)
        {
            double cv = Math.Cos(verticalHalfAngleOfView);
            double sv = Math.Sin(verticalHalfAngleOfView);
            double ch = Math.Cos(horizontalHalfAngleOfView);
            double sh = Math.Sin(horizontalHalfAngleOfView);
            double ca = Math.Cos(sightAngle);
            double sa = Math.Sin(sightAngle);

            return new List<vector3>()
            {
                camSightFrame.ToBaseFrame(sa*cv, sv, ca*cv),
                camSightFrame.ToBaseFrame(-sa*cv, sv, -ca*cv),
                camSightFrame.ToBaseFrame(ca*ch, sh, -sa*ch),
                camSightFrame.ToBaseFrame(-ca*ch, sh, sa*ch)
            };
        }
        private bool RegionInPositiveHalfspace(vector3 normal, List<vector3> pointsOfInterest)
        {
            double threshold = Position * normal;
            double[] pointsProjection = pointsOfInterest.Select(point => point * normal).ToArray();

            // For every arc of the spherical polygon, store the extreme value of the projection onto normal.
            double[] extremeProjection = new double[pointsProjection.Length];
            for (int i = 0; i < pointsProjection.Length; ++i)
            {
                extremeProjection[i] = ExtremeValueF(
                    pointsProjection[i],
                    pointsProjection[(i + 1) % pointsProjection.Length],
                    pointsOfInterest[i] * pointsOfInterest[(i + 1) % pointsProjection.Length]);
            }

            return (pointsProjection.All(proj => Comparison.IsBigger(proj, threshold)) && extremeProjection.All(proj => Comparison.IsBigger(proj, threshold)));
        }
        // F(t) measures the projection of a unit vector w(t) onto a given direction.
        // Here, w(t) follows a great circle between two unit vectors.
        // F1 = F(0), F2 = F(1), C = cos(w(0)^w(1))
        private double F(double F1, double F2, double C, double t)
        {
            return (F1 + t * (F2 - F1)) / Math.Sqrt(1 + 2 * (1 - C) * t * (t - 1));
        }
        // F(t) has a unique stationary point on (0,1), and we evaluate F at it.
        private double ExtremeValueF(double F1, double F2, double C)
        {
            double t;
            if (Comparison.IsZero(F1 - F2))
            {
                t = 0.5;
            }
            else
            {
                double a = F1 / (F2 - F1);
                double b = 2 * (1 - C);
                t = (2 + a * b) / b / (2 * a + 1);
            }

            return F(F1, F2, C, t);
        }
    }


    public class Arc
    {
        // Additional points on the arc with their relative distances from A.
        private List<vector3_keyDouble> intermediatePoints;

        public vector3 A { get; private set; }
        public vector3 B { get; private set; }
        public vector3 Apex { get; private set; }
        public IEnumerable<vector3> IntermediatePoints
        {
            get
            {
                return intermediatePoints.Select(p => p.v);
            }
        }

        public Arc(vector3 a, vector3 b, vector3 apex)
        {
            A = a;
            B = b;
            Apex = apex;
            intermediatePoints = new List<vector3_keyDouble>();
        }

        public Arc(vector3 a, vector3 b) : this(a, b, new vector3(0, 0, 0)) { }

        public static ICollection<vector3> Intersect(Arc a, Arc b)
        {
            return SphericalGeometryRoutines.IntersectTwoSmallArcs(
                a.A, a.B, a.Apex, b.A, b.B, b.Apex).Select(p => p.v).ToList();
        }

        public static void UpdateWithIntersections(Arc a, Arc b)
        {
            var intersection = SphericalGeometryRoutines.IntersectTwoSmallArcs(
                a.A, a.B, a.Apex, b.A, b.B, b.Apex);
            foreach (var p in intersection)
            {
                a.intermediatePoints.Add(new vector3_keyDouble(p.v, p.key1));
                b.intermediatePoints.Add(new vector3_keyDouble(p.v, p.key2));
            }
        }

        public ArcChain EmplaceIntermediatePoints()
        {
            intermediatePoints.Sort();
            List<vector3> points = new List<vector3>();
            points.Add(A);
            foreach (var point in intermediatePoints)
                points.Add(point.v);
            points.Add(B);

            return new ArcChain(points, Apex);
        }

        public ArcChainWithLabels<T> EmplaceIntermediatePoints<T>(T endLabel, T intermediateLabel)
        {
            List<T> labels = new List<T> { endLabel };
            foreach (var intermediate in intermediatePoints)
                labels.Add(intermediateLabel);
            labels.Add(endLabel);

            return EmplaceIntermediatePoints().Labelize<T>(labels);
        }
    }

    public class ArcChain
    {
        // The n-th apex corresponds to the (n, n+1) arc.
        // len(vertices) - len(apexes) == 1
        protected List<vector3> apexes;
        protected List<vector3> vertices;
        // Store only unique apexes?
        //private List<int> arcApexIndex;

        public ICollection<vector3> Apexes { get { return apexes; } }
        public ICollection<vector3> Vertices { get { return vertices; } }
        public ICollection<Arc> Arcs
        {
            get
            {
                List<Arc> arcs = new List<Arc>();
                for (int i = 0; i < apexes.Count; ++i)
                    arcs.Add(new Arc(vertices[i], vertices[i+1], apexes[i]));
                return arcs;
            }
        }

        public ArcChain()
        {
            apexes = new List<vector3>();
            vertices = new List<vector3>();
        }

        public ArcChain(ICollection<vector3> vertices, ICollection<vector3> apexes) : this()
        {
            if (vertices.Count - apexes.Count != 1)
                throw new ArgumentException("Number of points and arcs is incosistent.");

            //this.apexes = new List<vector3>();
            //this.vertices = new List<vector3>();
            //arcApexIndex = new List<int>();

            foreach (var point_apex in vertices.Zip(apexes, (point, apex) => Tuple.Create(point, apex)))
            {
                this.vertices.Add(point_apex.Item1);
                this.apexes.Add(point_apex.Item2);
            }
        }

        public ArcChain(IEnumerable<vector3> vertices, vector3 apex) : this()
        {
            //this.apexes = new List<vector3>();
            //this.vertices = new List<vector3>();
            foreach (vector3 point in vertices)
            {
                this.apexes.Add(apex);
                this.vertices.Add(point);
            }
            apexes.RemoveAt(apexes.Count - 1);
        }

        public void Add(vector3 point, vector3 apex)
        {
            vertices.Add(point);
            apexes.Add(apex);
        }

        public ArcChain Connect(ArcChain tail)
        {
            if (vertices[vertices.Count - 1] != tail.vertices[0])
                throw new ArgumentException("Ends of two chains are different");

            for (int i = 0; i < apexes.Count; ++i)
                Add(tail.vertices[i + 1], tail.apexes[i]);
            return this;
        }

        public void ConnectViaJunction(ArcChain tail, vector3 junctionApex)
        {
            Add(tail.vertices[0], junctionApex);
            for (int i = 0; i < apexes.Count; ++i)
                Add(tail.vertices[i + 1], tail.apexes[i]);
        }

        public ArcChainWithLabels<T> Labelize<T>(ICollection<T> labels)
        {
            return new ArcChainWithLabels<T>(this, labels);
        }
    }

    public class ArcChainWithLabels<T> : ArcChain
    {
        private List<T> labels;

        public IEnumerable<T> Labels { get { return labels; } }

        public ArcChainWithLabels(ICollection<vector3> vertices, ICollection<vector3> apexes,
                                  ICollection<T> labels) : base(vertices, apexes)
        {
            if (labels.Count != vertices.Count)
                throw new ArgumentException("Number of points and labels is incosistent.");
            
            this.labels = new List<T>();
            foreach (T label in labels)
                this.labels.Add(label);
        }

        public ArcChainWithLabels(ICollection<vector3> vertices, vector3 apex,
                                  ICollection<T> labels) : base(vertices, apex)
        {
            if (labels.Count != vertices.Count)
                throw new ArgumentException("Number of points and labels is incosistent.");

            this.labels = new List<T>();
            foreach (T label in labels)
                this.labels.Add(label);
        }

        public ArcChainWithLabels(ArcChain chain, ICollection<T> labels)
        {
            if (chain.Vertices.Count != labels.Count)
                throw new ArgumentException("Number of points and labels is incosistent.");

            base.apexes = chain.Apexes.ToList();
            base.vertices = chain.Vertices.ToList();
            
            this.labels = new List<T>();
            foreach (T label in labels)
                this.labels.Add(label);
        }

        public ArcChainWithLabels<T> Connect(ArcChainWithLabels<T> tail)
        {
            base.Connect(tail);
            foreach (T label in tail.Labels)
                labels.Add(label);
            return this;
        }
    }

    public class Polygon
    {
        // The n-th apex corresponds to the (n, n+1) arc (including (-1, 0)).
        // len(vertices) == len(apexes)
        protected List<vector3> apexes;
        protected List<vector3> vertices;

        public IEnumerable<vector3> Apexes { get { return apexes; } }
        public IEnumerable<vector3> Vertices { get { return vertices; } }
        public IEnumerable<Arc> Arcs
        {
            get
            {
                List<Arc> arcs = new List<Arc>();
                for (int i = 0; i < vertices.Count; ++i)
                    arcs.Add(new Arc(vertices[i], vertices[(i + 1) % vertices.Count], apexes[i]));
                return arcs;
            }
        }

        public Polygon(ICollection<vector3> vertices, ICollection<vector3> apexes)
        {
            if (vertices.Count != apexes.Count)
                throw new ArgumentException("Number of points and arcs is incosistent.");

            this.apexes = new List<vector3>();
            this.vertices = new List<vector3>();

            foreach (var point_apex in vertices.Zip(apexes, (point, apex) => Tuple.Create(point, apex)))
            {
                this.vertices.Add(point_apex.Item1);
                this.apexes.Add(point_apex.Item2);
            }
        }

        public Polygon(IEnumerable<vector3> vertices, vector3 apex)
        {
            this.apexes = new List<vector3>();
            this.vertices = new List<vector3>();
            foreach (vector3 point in vertices)
            {
                this.apexes.Add(apex);
                this.vertices.Add(point);
            }
        }

        public bool Contains(vector3 point)
        {
            // Ray casting algorithm. Shoot a great semiarc and count intersections.

            // Shoot an arc
            Random rand = new Random();
            vector3 displacement = new vector3((rand.NextDouble() - 0.5) * 1e-1,
                                               (rand.NextDouble() - 0.5) * 1e-1,
                                               (rand.NextDouble() - 0.5) * 1e-1);
            Arc halfGreat = new Arc(point, new vector3((displacement - point).Normalized(), 1));

            int numOfIntersections = 0;
            foreach (Arc arc in Arcs)
                numOfIntersections += Arc.Intersect(arc, halfGreat).Count;
            return (numOfIntersections % 2) == 1;
        }

        private enum pointType { vertex, enter, exit };
        public static Polygon Intersect(Polygon p, Polygon q)
        {
            var pArcs = p.Arcs;
            var qArcs = q.Arcs;

            foreach (var pArc in pArcs)
                foreach (var qArc in qArcs)
                    Arc.UpdateWithIntersections(pArc, qArc);

            ArcChainWithLabels<pointType> pChain = pArcs.
                Select(arc => arc.EmplaceIntermediatePoints<pointType>(pointType.vertex, pointType.enter)).
                Aggregate((head, tail) => head.Connect(tail));
            ArcChain qChain = qArcs.
                Select(arc => arc.EmplaceIntermediatePoints<pointType>(pointType.vertex, pointType.enter)).
                Aggregate((head, tail) => head.Connect(tail));

            return null;
        }
    }

    public struct vector3_keyInt
    {
        public vector3 v;
        public int key;
    }

    public class vector3_keyDouble : IComparable
    {
        public vector3 v;
        public double key;

        public vector3_keyDouble(vector3 vec, double val)
        {
            v = vec;
            key = val;
        }

        public int CompareTo(object obj)
        {
            if (obj == null)
                return 1;
            var v = obj as vector3_keyDouble;
            return key.CompareTo(v.key);
        }
    }

    public struct vector3_keyDoubleDouble
    {
        public vector3 v;
        public double key1;
        public double key2;

        public vector3_keyDoubleDouble(vector3 vec, double val1, double val2)
        {
            v = vec;
            key1 = val1;
            key2 = val2;
        }
    }

    public class vector3_GreinerHormann : IComparable
    {
        public vector3 v;
        public double relativePositionOnEdge;
        public bool wasProcessed;

        public vector3_GreinerHormann(vector3 _v, double _relativePositionOnEdge, bool _wasProcessed)
        {
            v = _v;
            relativePositionOnEdge = _relativePositionOnEdge;
            wasProcessed = _wasProcessed;
        }

        public int CompareTo(object obj)
        {
            if (obj == null)
                return 1;
            var v = obj as vector3_GreinerHormann;
            return relativePositionOnEdge.CompareTo(v.relativePositionOnEdge);
        }
    }

    public class CapturedRegion
    {
        private vector3 apex;
        private List<vector3_keyInt> vertices;
        // Label == 0 means a polygon vertex; label == 1 means an edge vertex

        public CapturedRegion(vector3 _apex, List<vector3_keyInt> _vertices)
        {
            apex = _apex;
            vertices = new List<vector3_keyInt>();
            foreach (var v in _vertices)
                vertices.Add(v);
        }
    }

    public static class SphericalGeometryRoutines
    {
        public static CapturedRegion IntersectConeProjectionWithPolygon(
            vector3 apex, List<vector3> normals, List<vector3> polygon)
        {
            List<vector3> coneBase = ProjectConeOntoSphere(apex, normals);
            
            // Compute pairwise intersections
            List<vector3_keyDoubleDouble>[,] intersections = new List<vector3_keyDoubleDouble>[coneBase.Count, polygon.Count];
            for (int i = 0; i < coneBase.Count; ++i)
                for (int j = 0; j < polygon.Count; ++j)
                    intersections[i, j] = IntersectSmallArcGreatArc(coneBase[i], coneBase[(i + 1) % coneBase.Count], apex,
                                                                    polygon[j], polygon[(j + 1) % polygon.Count]).ToList();
            
            // Collect intersections on the corresponding edges and sort according to distance
            List<vector3_GreinerHormann>[] coneBaseEdges = new List<vector3_GreinerHormann>[coneBase.Count];
            List<vector3_GreinerHormann>[] polygonEdges = new List<vector3_GreinerHormann>[polygon.Count];
            for (int i = 0; i < coneBase.Count; ++i)
                for (int j = 0; j < polygon.Count; ++j)
                {
                    coneBaseEdges[i].Add(new vector3_GreinerHormann(coneBase[i], 0, false));
                    coneBaseEdges[i].Concat(intersections[i, j].Select(v_dd => new vector3_GreinerHormann(v_dd.v, v_dd.key1, false)));
                    polygonEdges[j].Add(new vector3_GreinerHormann(polygon[j], 0, false));
                    polygonEdges[j].Concat(intersections[i, j].Select(v_dd => new vector3_GreinerHormann(v_dd.v, v_dd.key2, false)));
                }
            foreach (var edge in coneBaseEdges)
                edge.Sort();
            foreach (var edge in polygonEdges)
                edge.Sort();

            List<vector3_GreinerHormann> coneBasePoints = coneBaseEdges.Aggregate((x, y) => x.Concat(y).ToList());
            List<vector3_GreinerHormann> polygonPoints = polygonEdges.Aggregate((x, y) => x.Concat(y).ToList());

            // Mark points as enter/exit


            return null;
        }


        public static List<vector3> ProjectConeOntoSphere(vector3 apex, List<vector3> normals)
        {
            List<vector3> coneEdgeDirections = new List<vector3>();
            coneEdgeDirections.Add(normals[normals.Count - 1].Cross(normals[0]));
            for (int i = 1; i < normals.Count; ++i)
                coneEdgeDirections.Add(normals[i].Cross(normals[i - 1]));

            List<vector3> result = new List<vector3>();
            foreach (vector3 direction in coneEdgeDirections)
            {
                var intersection = IntersectLineUnitSphere(apex, direction);
                // Assume a good cone: all its edges intersect the unit sphere
                result.Add(intersection.Where(v => Comparison.IsPositive(apex * v - 1)).ElementAt(0));
            }
            return result;
        }

        // Returns intersection points with their relative positions from a1 and a2
        public static List<vector3_keyDoubleDouble> IntersectTwoSmallArcs(
            vector3 a1, vector3 b1, vector3 apex1,
            vector3 a2, vector3 b2, vector3 apex2)
        {

            vector3 normal1 = (a1 - apex1).Cross(b1 - apex1);
            vector3 center1 = (a1 * normal1) / (normal1 * normal1) * normal1;
            vector3 normal2 = (a2 - apex2).Cross(b2 - apex2);
            vector3 center2 = (a2 * normal2) / (normal2 * normal2) * normal2;

            // Intersect two planes
            vector3 directionVector = normal1.Cross(normal2);
            vector3 pointOnIntersection = SolveSLE2x3(normal1,
                                                      normal2,
                                                      a1 * normal1,
                                                      a2 * normal2);
            List<vector3> intersectPlanesOnSphere = 
                IntersectLineUnitSphere(pointOnIntersection, directionVector);
            
            // Check if these points lie on the given arcs, i.e. positive dot product with inner normals
            List<vector3> innerNormals = new List<vector3> {normal1.Cross(a1 - center1),     
                                                            (b1 - center1).Cross(normal1),          
                                                            normal2.Cross(a2 - center2),       
                                                            (b2 - center2).Cross(normal2)};
            
            return intersectPlanesOnSphere.
                   Where(point => 
                         innerNormals.TrueForAll(normal => 
                                                 !Comparison.IsNegative(normal * point))).
                   Select(point => 
                          new vector3_keyDoubleDouble(point, 1 - point*(a1 - center1), 1 - point*(a2 - center2))).
                   ToList();
        }

        public static List<vector3_keyDoubleDouble> IntersectSmallArcGreatArc(
            vector3 smallA, vector3 smallB, vector3 smallApex,
            vector3 greatA, vector3 greatB)
        {
            return IntersectTwoSmallArcs(
                smallA, smallB, smallApex,
                greatA, greatB, new vector3(0, 0, 0));
        }

        public static vector3 SolveSLE2x3(vector3 a1, vector3 a2, double b1, double b2)
        {
            double x, y, z;
            double detXY = a1.X * a2.Y - a1.Y * a2.X;
            double detXZ = a1.X * a2.Z - a1.Z * a2.X;
            double detYZ = a1.Y * a2.Z - a1.Z * a2.Y;

            if (!Comparison.IsZero(detXY))
            {
                z = 0;
                x = (b1 * a2.Y - b2 * a1.Y) / detXY;
                y = (b2 * a1.X - b1 * a2.X) / detXY;
            }
            else if (!Comparison.IsZero(detXZ))
            {
                y = 0;
                x = (b1 * a2.Z - b2 * a1.Z) / detXZ;
                z = (b2 * a1.X - b1 * a2.X) / detXZ;
            }
            else if (!Comparison.IsZero(detYZ))
            {
                x = 0;
                y = (b1 * a2.Z - b2 * a1.Z) / detXZ;
                z = (b2 * a1.Y - b1 * a2.Y) / detXZ;
            }
            else
                throw new System.ArgumentException("System is rank deficient.");

            return new vector3(x, y, z);
        }

        public static List<double> SolveQuadraticEquation(double a, double b, double c)
        {
            List<double> res = new List<double>();
            double D = b * b - 4 * a * c;
            if (Comparison.IsPositive(D))
            {
                double sqrt = Math.Sqrt(D);
                res.Add((-b - sqrt) / (2 * a));
                res.Add((-b + sqrt) / (2 * a));
            }
            else if (Comparison.IsZero(D))
            {
                res.Add(-b / (2 * a));
            }
            return res;
        }

        public static List<vector3> IntersectLineUnitSphere(vector3 a, vector3 dir)
        {
            List<double> parameters = SolveQuadraticEquation(dir * dir, 2 * a * dir, a * a - 1);
            return parameters.Select(t => a + t * dir).ToList();
        }
    }
}
