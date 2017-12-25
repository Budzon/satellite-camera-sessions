using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

using Common;
using Astronomy;

namespace SphericalGeom
{
    public class Arc
    {
        // Additional points on the arc with their relative distances from A.
        private Dictionary<double, Vector3D> intermediatePoints;

        #region Arc properties
        /// <summary>
        /// Arc's first endpoint.
        /// </summary>
        public Vector3D A { get; private set; }
        /// <summary>
        /// Arc's second endpoint.
        /// </summary>
        public Vector3D B { get; private set; }
        /// <summary>
        /// Unit normal to the plane to which the arc belongs.
        /// </summary>
        /// <remarks>Axis = CrossProduct(A, B)</remarks>
        public Vector3D Axis { get; private set; }
        /// <summary>
        /// Center of the small circle to which the arc belongs.
        /// </summary>
        public Vector3D Center { get; private set; }
        /// <summary>
        /// Unit tangent vector to the arc at the first endpoint in the traverse direction.
        /// </summary>
        public Vector3D TangentA { get; private set; }
        /// <summary>
        /// Unit tangent vector to the arc at the last endpoint in the traverse direction.
        /// </summary>
        public Vector3D TangentB { get; private set; }
        /// <summary>
        /// Angle in radians that the arc subtends with the small circle center.
        /// </summary>
        public double CentralAngle { get; private set; }
        /// <summary>
        /// Angle in radians that the arc subtends with the small circle center.
        /// Unlike CentralAngle, it respects the traversion direction.
        /// Is undefined for great circles.
        /// </summary>
        public double CentralAngleWithOrientation { get; private set; }
        /// <summary>
        /// Radius of curvature.
        /// </summary>
        public double Radius { get; private set; }
        public double GeodesicCurvature { get; private set; }
        /// <summary>
        /// True if the arc corresponds to a counterclockwise rotation about Center
        /// </summary>
        public bool Counterclockwise { get; private set; }
        public IEnumerable<Vector3D> IntermediatePoints
        {
            get { return intermediatePoints.Select(p => p.Value); }
        }
        #endregion

        #region Arc constructors
        /// <summary>
        /// Creates a small circle minor arc with endpoints <paramref name="a"/> and <paramref name="b"/>
        /// that lies in one plane with <paramref name="apex"/>
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="apex"></param>
        public Arc(Vector3D a, Vector3D b, Vector3D apex)
        {
            A = a;
            A.Normalize();
            B = b;
            B.Normalize();

            var normal = Vector3D.CrossProduct(Comparison.IsSmaller(apex.Length, 1) ? A - apex : apex - A,
                                                    B - apex);
            normal.Normalize();
            Axis = normal;
            Center = Vector3D.DotProduct(A, normal) * normal;

            CentralAngle = AstronomyMath.ToRad(Vector3D.AngleBetween(A - Center, B - Center));
            double sin = Vector3D.DotProduct(
                Vector3D.CrossProduct(A - Center, B - Center),
                Center) / Center.Length;
            double cos = Vector3D.DotProduct(A - Center, B - Center);
            CentralAngleWithOrientation = Math.Atan2(sin, cos);
            if (CentralAngleWithOrientation < 0)
                CentralAngleWithOrientation += 2 * Math.PI;
           
            Radius = (A - Center).Length;
            GeodesicCurvature = Math.Sqrt(1 - Radius * Radius) / Radius;

            Vector3D tangentA = Vector3D.CrossProduct(A - Center, Vector3D.CrossProduct(B - A, A - Center));
            tangentA.Normalize();
            TangentA = tangentA;

            Vector3D tangentB = Vector3D.CrossProduct(B - Center, Vector3D.CrossProduct(B - A, B - Center));
            tangentB.Normalize();
            TangentB = tangentB;
            Vector3D temp = Vector3D.CrossProduct(tangentA, A);
            double tt = Vector3D.DotProduct(A - Center, temp);
            Counterclockwise = Comparison.IsPositive(tt);

            intermediatePoints = new Dictionary<double, Vector3D>();
        }

        /// <summary>
        /// Creates a great circle minor arc with endpoints <paramref name="a"/> and <paramref name="b"/>.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        public Arc(Vector3D a, Vector3D b) : this(a, b, new Vector3D(0, 0, 0)) { }
        #endregion

        public double MaxLatitudeDeg()
        {
            return AstronomyMath.ToDegrees(Routines.FindMax(ParametrizedLatitudeRad, 0, CentralAngle));
        }

        public double MinLatitudeDeg()
        {
            return AstronomyMath.ToDegrees(Routines.FindMin(ParametrizedLatitudeRad, 0, CentralAngle));
        }

        public double MaxLongitudeDeg()
        {
            return AstronomyMath.ToDegrees(Routines.FindMax(ParametrizedLongitudeRad, 0, CentralAngle));
        }

        public double MinLongitudeDeg()
        {
            return AstronomyMath.ToDegrees(Routines.FindMin(ParametrizedLongitudeRad, 0, CentralAngle));
        }

        /// <summary>
        /// Intersect two arcs.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns>Intersection points.</returns>
        public static ICollection<Vector3D> Intersect(Arc a, Arc b)
        {
            return IntersectWithDistances(a, b).Values;
        }

        #region Arc internal methods
        /// <summary>
        /// Intersects two arcs and stores the intersection points inside <paramref name="a"/> and <paramref name="b"/>/
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        internal static void UpdateWithIntersections(Arc a, Arc b)
        {
            var intersection = IntersectWithDistances(a, b);
            foreach (var p in intersection)
            {
                a.intermediatePoints.Add(p.Key.Item1, p.Value);
                b.intermediatePoints.Add(p.Key.Item2, p.Value);
            }
        }

        internal ArcChain EmplaceIntermediatePoints()
        {
            var points = new List<Vector3D> { A };
            foreach (var point in intermediatePoints.OrderBy(p => p.Key))
                points.Add(point.Value);
            points.Add(B);
            intermediatePoints = new Dictionary<double, Vector3D>();
            return new ArcChain(points, Center);
        }

        internal ArcChainWithLabels<T> EmplaceIntermediatePoints<T>(T endLabel, T intermediateLabel)
        {
            List<T> labels = new List<T> { endLabel };
            foreach (var intermediate in intermediatePoints)
                labels.Add(intermediateLabel);
            labels.Add(endLabel);
            return EmplaceIntermediatePoints().Labelize<T>(labels);
        }
        #endregion

        #region Arc private methods
        private double ParametrizedLongitudeRad(double angleFromAinRad)
        {
            double C = Math.Cos(angleFromAinRad), S = Math.Sin(angleFromAinRad), t = 1 - C;
            return Math.Atan2(
                (t * Axis.X * Axis.Y + S * Axis.Z) * A.X 
                + (t * Axis.Y * Axis.Y + C) * A.Y 
                + (t * Axis.Y * Axis.Z - S * Axis.X) * A.Z,
                /**/
                (t * Axis.X * Axis.X + C) * A.X 
                + (t * Axis.X * Axis.Y - S * Axis.Z) * A.Y 
                + (t * Axis.X * Axis.Z + S * Axis.Y) * A.Z);
        }

        private double ParametrizedLatitudeRad(double angleFromAinRad)
        {
            double C = Math.Cos(angleFromAinRad), S = Math.Sin(angleFromAinRad), t = 1 - C;
            return Math.Asin(
                (t * Axis.X * Axis.Z - S * Axis.Y) * A.X
                + (t * Axis.Y * Axis.Z + S * Axis.X) * A.Y
                + (t * Axis.Z * Axis.Z + C) * A.Z);
        }

        /// <summary>
        /// Intersects two arcs and computes relative positions of the intersection points
        /// on the arcs with respect to their first endpoints.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns>Intersection points, their relative positions on <paramref name="a"/> and <paramref name="b"/></returns>
        private static Dictionary<Tuple<double, double>, Vector3D> IntersectWithDistances(Arc a, Arc b)
        {
            // Intersect two planes
            var directionVector = Vector3D.CrossProduct(a.Axis, b.Axis);
            var pointOnIntersection = Routines.SolveSLE2x3(a.Axis, b.Axis, a.Center.Length, b.Center.Length);
            List<Vector3D> intersectPlanesOnSphere =
                Routines.IntersectLineUnitSphere(pointOnIntersection, directionVector);

            // Check if these points lie on the given arcs, i.e. positive dot product with inner normals
            List<Vector3D> innerNormals = new List<Vector3D> {
                Vector3D.CrossProduct(a.Axis, a.A - a.Center),
                Vector3D.CrossProduct(a.B - a.Center, a.Axis),
                Vector3D.CrossProduct(b.Axis, b.A - b.Center),
                Vector3D.CrossProduct(b.B - b.Center, b.Axis)
            };

            return intersectPlanesOnSphere.Where(point =>
                         innerNormals.TrueForAll(normal => !Comparison.IsNegative(Vector3D.DotProduct(normal, point)))).
                   ToDictionary(point => Tuple.Create(1 - Vector3D.DotProduct(point, a.A - a.Center),
                                                      1 - Vector3D.DotProduct(point, b.A - b.Center)));
        }
        #endregion
    }
}
