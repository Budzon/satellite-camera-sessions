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
        private List<VectorWithDistance> intermediatePoints;

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
            get { return intermediatePoints.Select(vwtd => vwtd.Vector); }
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
           
            Radius = (A - Center).Length;
            GeodesicCurvature = Math.Sqrt(1 - Radius * Radius) / Radius;

            Vector3D tangentA = Vector3D.CrossProduct(A - Center, Vector3D.CrossProduct(B - A, A - Center));
            tangentA.Normalize();
            TangentA = tangentA;

            Vector3D tangentB = Vector3D.CrossProduct(B - Center, Vector3D.CrossProduct(B - A, B - Center));
            tangentB.Normalize();
            TangentB = tangentB;
 
            Counterclockwise = !Comparison.IsNegative(Vector3D.DotProduct(A - Center,Vector3D.CrossProduct(tangentA, A)));
            CentralAngleWithOrientation = Counterclockwise ? CentralAngle : 2 * Math.PI - CentralAngle;

            intermediatePoints = new List<VectorWithDistance>();
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
        public static IEnumerable<Vector3D> Intersect(Arc a, Arc b)
        {
            return IntersectWithDistances(a, b).Select(vwtd => vwtd.Vector);
        }

        #region Arc internal methods
        /// <summary>
        /// Intersects two arcs and stores the intersection points inside <paramref name="a"/> and <paramref name="b"/>/
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        internal static void UpdateWithIntersections(Arc a, Arc b)
        {
            IEnumerable<VectorWithTwoDistances> intersection = IntersectWithDistances(a, b);
            foreach (VectorWithTwoDistances vwtd in intersection)
            {
                a.intermediatePoints.Add(new VectorWithDistance { Vector = vwtd.Vector, Distance = vwtd.DistanceOnFirstArc });
                b.intermediatePoints.Add(new VectorWithDistance { Vector = vwtd.Vector, Distance = vwtd.DistanceOnSecondArc });
            }
        }

        internal ArcChain EmplaceIntermediatePoints()
        {
            intermediatePoints.Sort();

            List<Vector3D> points = new List<Vector3D> { A };
            foreach (VectorWithDistance vwd in intermediatePoints)
                points.Add(vwd.Vector);
            points.Add(B);

            intermediatePoints = new List<VectorWithDistance>();
            return new ArcChain(points, Center);
        }

        internal ArcChainWithLabels<T> EmplaceIntermediatePoints<T>(T endLabel, T intermediateLabel)
        {
            List<T> labels = new List<T> { endLabel };
            for (int i = 0; i < intermediatePoints.Count; ++i)
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
        private static IEnumerable<VectorWithTwoDistances> IntersectWithDistances(Arc a, Arc b)
        {
            // Intersect two planes
            var directionVector = Vector3D.CrossProduct(a.Axis, b.Axis);
            var pointOnIntersection = Routines.SolveSLE2x3(a.Axis, b.Axis, a.Center.Length, b.Center.Length);
            IEnumerable<Vector3D> intersectPlanesOnSphere =
                Routines.IntersectLineUnitSphere(pointOnIntersection, directionVector);

            // Check if these points lie on the given arcs, i.e. positive dot product with inner normals
            List<Vector3D> innerNormals = new List<Vector3D> {
                Vector3D.CrossProduct(a.Axis, a.A - a.Center),
                Vector3D.CrossProduct(a.B - a.Center, a.Axis),
                Vector3D.CrossProduct(b.Axis, b.A - b.Center),
                Vector3D.CrossProduct(b.B - b.Center, b.Axis)
            };

            return intersectPlanesOnSphere
                   .Where(point =>
                         innerNormals
                         .TrueForAll(normal => 
                                    !Comparison.IsNegative(Vector3D.DotProduct(normal, point))))
                                    .Select(point => new VectorWithTwoDistances
                                    {
                                        Vector = point,
                                        DistanceOnFirstArc = 1 - Vector3D.DotProduct(point, a.A - a.Center),
                                        DistanceOnSecondArc = 1 - Vector3D.DotProduct(point, b.A - b.Center)
                                    });
        }
        #endregion
    }

    internal class VectorWithDistance : IComparable<VectorWithDistance>
    {
        public Vector3D Vector { get; set; }
        public double Distance { get; set; }

        public int CompareTo(VectorWithDistance vwd)
        {
            if (vwd == null)
                return 1;
            return Distance.CompareTo(vwd.Distance);
        }
    }

    internal struct VectorWithTwoDistances
    {
        public Vector3D Vector { get; set; }
        public double DistanceOnFirstArc { get; set; }
        public double DistanceOnSecondArc { get; set; }
    }
}
