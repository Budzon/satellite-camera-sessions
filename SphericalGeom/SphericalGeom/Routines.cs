using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace SphericalGeom
{
    public static class Routines
    {
        public static Polygon ProjectConeOntoSphere(Vector3D apex, IList<Vector3D> normals)
        {
            var coneEdgeDirections = new List<Vector3D>();
            coneEdgeDirections.Add(Vector3D.CrossProduct(normals[normals.Count - 1], normals[0]));
            for (int i = 1; i < normals.Count; ++i)
                coneEdgeDirections.Add(Vector3D.CrossProduct(normals[i-1], normals[i]));

            var result = new List<Vector3D>();
            foreach (var direction in coneEdgeDirections)
            {
                var intersection = IntersectLineUnitSphere(apex, direction);
                // Assume a good cone: all its edges intersect the unit sphere
                result.Add(intersection.Where(v => Comparison.IsPositive(Vector3D.DotProduct(apex, v) - 1)).ElementAt(0));
            }
            return new Polygon(result, apex);
        }

        // Returns intersection points with their relative positions from a1 and a2
        public static Dictionary<Tuple<double, double>, Vector3D> IntersectTwoSmallArcs(
            Vector3D a1, Vector3D b1, Vector3D apex1,
            Vector3D a2, Vector3D b2, Vector3D apex2)
        {

            var normal1 = Vector3D.CrossProduct(Comparison.IsSmaller(apex1.Length, 1) ? a1 - apex1 : apex1 - a1, 
                                                b1 - apex1);
            normal1.Normalize();
            var center1 = Vector3D.DotProduct(a1, normal1) * normal1;
            var normal2 = Vector3D.CrossProduct(Comparison.IsSmaller(apex2.Length, 1) ? a2 - apex2 : apex2 - a2, 
                                                b2 - apex2);
            normal2.Normalize();
            var center2 = Vector3D.DotProduct(a2, normal2) * normal2;

            // Intersect two planes
            var directionVector = Vector3D.CrossProduct(normal1, normal2);
            var pointOnIntersection = SolveSLE2x3(normal1,
                                                      normal2,
                                                      Vector3D.DotProduct(a1, normal1),
                                                      Vector3D.DotProduct(a2, normal2));
            List<Vector3D> intersectPlanesOnSphere = 
                IntersectLineUnitSphere(pointOnIntersection, directionVector);
            
            // Check if these points lie on the given arcs, i.e. positive dot product with inner normals
            List<Vector3D> innerNormals = new List<Vector3D> {
                Vector3D.CrossProduct(normal1, a1 - center1),
                Vector3D.CrossProduct(b1 - center1, normal1),
                Vector3D.CrossProduct(normal2, a2 - center2),
                Vector3D.CrossProduct(b2 - center2, normal2)
            };
            
            return intersectPlanesOnSphere.
                   Where(point => 
                         innerNormals.TrueForAll(normal => 
                                                 !Comparison.IsNegative(Vector3D.DotProduct(normal, point)))).
                   ToDictionary(point => 
                                Tuple.Create(1 - Vector3D.DotProduct(point, a1 - center1),
                                             1 - Vector3D.DotProduct(point, a2 - center2)));
        }

        public static Dictionary<Tuple<double, double>, Vector3D> IntersectSmallArcGreatArc(
            Vector3D smallA, Vector3D smallB, Vector3D smallApex,
            Vector3D greatA, Vector3D greatB)
        {
            return IntersectTwoSmallArcs(
                smallA, smallB, smallApex,
                greatA, greatB, new Vector3D(0, 0, 0));
        }

        public static Vector3D SolveSLE2x3(Vector3D a1, Vector3D a2, double b1, double b2)
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

            return new Vector3D(x, y, z);
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

        public static List<Vector3D> IntersectLineUnitSphere(Vector3D a, Vector3D dir)
        {
            List<double> parameters = SolveQuadraticEquation(dir.LengthSquared, 
                                                             2 * Vector3D.DotProduct(a, dir), 
                                                             a.LengthSquared - 1);
            return parameters.Select(t => a + t * dir).ToList();
        }
    }
}
