using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace SphericalGeom
{
    public class ArcChain
    {
        // The n-th apex corresponds to the (n, n+1) arc.
        // len(vertices) - len(apexes) == 1
        protected List<Vector3D> apexes;
        protected List<Vector3D> vertices;

        public IList<Vector3D> Apexes { get { return apexes; } }
        public IList<Vector3D> Vertices { get { return vertices; } }
        public ICollection<Arc> Arcs
        {
            get
            {
                var arcs = new List<Arc>();
                for (int i = 0; i < apexes.Count; ++i)
                    arcs.Add(new Arc(vertices[i], vertices[i + 1], apexes[i]));
                return arcs;
            }
        }

        public ArcChain()
        {
            apexes = new List<Vector3D>();
            vertices = new List<Vector3D>();
        }
        public ArcChain(ICollection<Vector3D> vertices, ICollection<Vector3D> apexes) : this()
        {
            if (vertices.Count - apexes.Count != 1)
                throw new ArgumentException("Number of points and arcs is incosistent.");

            foreach (var point_apex in vertices.Zip(apexes, (point, apex) => Tuple.Create(point, apex)))
            {
                this.vertices.Add(point_apex.Item1);
                this.apexes.Add(point_apex.Item2);
            }
        }
        public ArcChain(IEnumerable<Vector3D> vertices, Vector3D apex) : this()
        {
            foreach (var point in vertices)
            {
                this.apexes.Add(apex);
                this.vertices.Add(point);
            }
            apexes.RemoveAt(apexes.Count - 1);
        }

        public void Add(Vector3D point, Vector3D apex)
        {
            vertices.Add(point);
            apexes.Add(apex);
        }
        public ArcChain Connect(ArcChain tail)
        {
            if (vertices[vertices.Count - 1] != tail.vertices[0])
                throw new ArgumentException("Ends of two chains are different");

            for (int i = 1; i < tail.vertices.Count; ++i)
                Add(tail.vertices[i], tail.apexes[i - 1]);
            return this;
        }
        public void ConnectViaJunction(ArcChain tail, Vector3D junctionApex)
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

        public IList<T> Labels { get { return labels; } }

        public ArcChainWithLabels(ICollection<Vector3D> vertices, ICollection<Vector3D> apexes,
                                  ICollection<T> labels) : base(vertices, apexes)
        {
            if (labels.Count != vertices.Count)
                throw new ArgumentException("Number of points and labels is incosistent.");

            this.labels = new List<T>();
            foreach (T label in labels)
                this.labels.Add(label);
        }
        public ArcChainWithLabels(ICollection<Vector3D> vertices, Vector3D apex,
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
            for (int i = 1; i < tail.labels.Count; ++i)
                labels.Add(tail.labels[i]);
            return this;
        }
    }
}
