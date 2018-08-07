using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using Microsoft.SqlServer.Types;
using System.Data.SqlTypes;
using Common;
using System.Runtime.InteropServices;

using MNCbicSplne = MathNet.Numerics.Interpolation.CubicSpline; // пересечения с Astronomy CubicSpline

namespace SphericalGeom
{
    public class Polygon
    {
        private SqlGeography _geography;

        public Polygon(SqlGeography geography)
        {
            _geography = geography;
        }

        public Polygon(string wkt)
        {
            _geography = SqlGeography.STGeomFromText(new SqlChars(wkt), 4326);
        }

        public Polygon(List<Vector3D> points)
        {
           // _geography.
            //GeoPoint gpoint = GeoPoint.FromCartesian(ver);
            SqlGeographyBuilder builder = new SqlGeographyBuilder();
            builder.SetSrid(4326);
            builder.BeginGeography(OpenGisGeographyType.Polygon);
            GeoPoint firstgeop = GeoPoint.FromCartesian(points.First());
            builder.BeginFigure(firstgeop.Latitude, firstgeop.Longitude);
            foreach (var p in points.Skip(1))
            {
                GeoPoint geop = GeoPoint.FromCartesian(p);
                builder.AddLine(geop.Latitude, geop.Longitude);
            }
            builder.AddLine(firstgeop.Latitude, firstgeop.Longitude);
            builder.EndFigure();
            builder.EndGeography();
            _geography = builder.ConstructedGeography; 
        }


        public double Area
        {
            get
            {
                return (double)_geography.STArea();
            }
        }
        public string ToWtk()
        {
            return _geography.ToString();
        }
        public List<GeoPoint> getCenterLine()
        {             
            List<GeoPoint> points = new List<GeoPoint>(); 
            return points;
        }

        public List<Vector3D> Vertices
        {
            get
            {
                List<Vector3D> vertices = new List<Vector3D>();
                for (int i = 1; i <= _geography.STNumPoints() ; i++)
                {
                    GeoPoint gpoint = new GeoPoint(
                        (double)_geography.STPointN(i).Lat,
                        (double)_geography.STPointN(i).Long);
                    vertices.Add(GeoPoint.ToCartesian(gpoint, 1));
                }
                return vertices;
            }
        }

        public static IList<Polygon> Intersect(Polygon p, Polygon q)
        {
            IList<Polygon> intersects = new List<Polygon>();

            SqlGeography intersection = p._geography.STIntersection(q._geography);

            for (int i = 1; i <= intersection.STNumGeometries(); i++)
            {
                intersects.Add( new Polygon(intersection.STGeometryN(i)) );
            }
            return intersects;
        }


        public static Tuple<List<Polygon>, List<Polygon>> IntersectAndSubtract(Polygon p, Polygon q)
        {
            List<Polygon> intersects = new List<Polygon>();
            List<Polygon> differences = new List<Polygon>();
            SqlGeography intersection = p._geography.STIntersection(q._geography);
            SqlGeography difference = p._geography.STDifference(q._geography);
            
            for (int i = 1; i <= intersection.STNumGeometries(); i++)
            {
                intersects.Add( new Polygon(intersection.STGeometryN(i)) );
            }

            for (int i = 1; i <= difference.STNumGeometries(); i++)
            {
                differences.Add(new Polygon(difference.STGeometryN(i)));
            }

            return Tuple.Create(intersects, differences);
        }

        public static Tuple<List<Polygon>, List<Polygon>> IntersectAndSubtract(Polygon p, IList<Polygon> qs)
        {
            List<Polygon> intersection = new List<Polygon>();
            List<Polygon> diff = new List<Polygon>() { p };
            foreach (Polygon q in qs)
            {
                List<Polygon> newDiff = new List<Polygon>();
                foreach (Polygon d in diff)
                {
                    var intSub = IntersectAndSubtract(d, q);
                    newDiff.AddRange(intSub.Item2);
                    intersection.AddRange(intSub.Item1);
                }
                diff.Clear();
                diff = newDiff;
            }

            return Tuple.Create(intersection, diff);
        }
         


        public static string getMultipolFromWkts(List<string> wkts)
        {
            string res = "";
            /*
            if (wkts.Count == 0)
                return res;

            foreach (var wkt in wkts)
            {
                res = res + wkt + "\n";
                if (wkt != wkts.Last())
                    res += ",";
            }
            res = "GEOMETRYCOLLECTION (" + res + ")";
             */
            return res;
        }

        public static string getMultipolFromPolygons(List<Polygon> polygons)
        {
            string res = "";

            if (polygons.Count == 0)
                return res;

            for (int i = 0; i < polygons.Count; i++)
            {
                res = res + polygons[i].ToWtk() + "\n";
                if (i != polygons.Count - 1)
                    res += ",";
            }
            res = "GEOMETRYCOLLECTION (" + res + ")";
            return res;
        }
    }
}

