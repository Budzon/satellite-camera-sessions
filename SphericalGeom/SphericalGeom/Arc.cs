using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

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
        public IEnumerable<Vector3D> IntermediatePoints
        {
            get { return intermediatePoints.Select(p => p.Value); }
        }

        public Arc(Vector3D a, Vector3D b, Vector3D apex)
        {
            A = a;
            B = b;
            Apex = apex;
            intermediatePoints = new Dictionary<double, Vector3D>();
        }

        public Arc(Vector3D a, Vector3D b) : this(a, b, new Vector3D(0, 0, 0)) { }

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
    }
}
