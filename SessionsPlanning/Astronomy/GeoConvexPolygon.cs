using System;
using System.Collections.Generic;
using System.Text;

namespace Common
{
    public class GeoConvexPolygon
    {
        private GeoPoint[] points;

        public GeoConvexPolygon(GeoPoint[] points)
        {
            this.points = points;
        }

        public GeoConvexPolygon(IList<GeoPoint> points)
        {
            this.points = new GeoPoint[points.Count];
            for (int i = 0; i < points.Count; i++)
            {
                this.points[i] = points[i];
            }
        }

        public int Count { get { return points.Length; } }

        public GeoPoint this[int index] { get { return points[index]; } }

        public GeoPoint[] Points { get { return points; } }

        public bool IsEmpty { get { return points.Length == 0; } }
    }
}
