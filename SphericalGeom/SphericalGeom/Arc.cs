using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

using Common;

namespace SphericalGeom
{
    public class Arc
    {
        // Additional points on the arc with their relative distances from A.
        //private List<vector3_keyDouble> intermediatePoints;
        private Dictionary<double, Vector3D> intermediatePoints;

        public Vector3D A { get; private set; }
        public Vector3D B { get; private set; }
        public Vector3D Apex { get; private set; }
        public Vector3D Axis { get; private set; }
        public Vector3D Center { get; private set; }
        public IEnumerable<Vector3D> IntermediatePoints
        {
            get { return intermediatePoints.Select(p => p.Value); }
        }

        public Arc(Vector3D a, Vector3D b, Vector3D apex)
        {
            A = a;
            A.Normalize();
            B = b;
            B.Normalize();
            Apex = apex;

            var normal = Vector3D.CrossProduct(Comparison.IsSmaller(Apex.Length, 1) ? A - Apex : Apex - A,
                                                    B - Apex);
            normal.Normalize();
            Axis = normal;
            Center = Vector3D.DotProduct(A, normal) * normal;

            intermediatePoints = new Dictionary<double, Vector3D>();
        }

        public Arc(Vector3D a, Vector3D b) : this(a, b, new Vector3D(0, 0, 0)) { }

        public double MaxLatitudeDeg()
        {
            double maxRotation = Vector3D.AngleBetween(A, B) * System.Math.PI / 180.0;
            return Routines.FindMax(ParametrizedLatitudeRad, 0, maxRotation) * 180.0 / Math.PI;
        }

        public double MinLatitudeDeg()
        {
            double maxRotation = Vector3D.AngleBetween(A, B) * System.Math.PI / 180.0;
            return Routines.FindMin(ParametrizedLatitudeRad, 0, maxRotation) * 180.0 / Math.PI;
        }

        public double MaxLongitudeDeg()
        {
            double maxRotation = Vector3D.AngleBetween(A, B) * System.Math.PI / 180.0;
            return Routines.FindMax(ParametrizedLongitudeRad, 0, maxRotation) * 180.0 / Math.PI;
        }

        public double MinLongitudeDeg()
        {
            double maxRotation = Vector3D.AngleBetween(A, B) * System.Math.PI / 180.0;
            return Routines.FindMin(ParametrizedLongitudeRad, 0, maxRotation) * 180.0 / Math.PI;
        }

        public static ICollection<Vector3D> Intersect(Arc a, Arc b)
        {
            return Routines.IntersectTwoSmallArcs(
                a.A, a.B, a.Apex, b.A, b.B, b.Apex).Values;
        }

        public static void UpdateWithIntersections(Arc a, Arc b)
        {
            var intersection = Routines.IntersectTwoSmallArcs(
                a.A, a.B, a.Apex, b.A, b.B, b.Apex);
            foreach (var p in intersection)
            {
                a.intermediatePoints.Add(p.Key.Item1, p.Value);
                b.intermediatePoints.Add(p.Key.Item2, p.Value);
            }
        }

        public ArcChain EmplaceIntermediatePoints()
        {
            var points = new List<Vector3D> { A };
            foreach (var point in intermediatePoints.OrderBy(p => p.Key))
                points.Add(point.Value);
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
    }
}
