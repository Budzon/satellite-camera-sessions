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

        public bool Contains(GeoPoint testP)
        {
            SqlGeographyBuilder builder = new SqlGeographyBuilder();
            builder.SetSrid(4326);
            builder.BeginGeography(OpenGisGeographyType.Point);
            builder.BeginFigure(testP.Latitude, testP.Longitude);
            builder.EndFigure();
            builder.EndGeography();
            SqlGeography pointGeography = builder.ConstructedGeography;
            return (bool)_geography.STContains(pointGeography);
        }

        public Polygon(List<GeoPoint> points)
        { 
            SqlGeographyBuilder builder = new SqlGeographyBuilder();
            builder.SetSrid(4326);
            builder.BeginGeography(OpenGisGeographyType.Polygon);
            GeoPoint firstgeop = points.First();
            builder.BeginFigure(firstgeop.Latitude, firstgeop.Longitude);
            foreach (var geop in points.Skip(1))
            {
                builder.AddLine(geop.Latitude, geop.Longitude);
            }
            builder.AddLine(firstgeop.Latitude, firstgeop.Longitude);
            builder.EndFigure();
            builder.EndGeography();
            _geography = builder.ConstructedGeography; 
        }

        public Polygon(List<Vector3D> points)
            : this(points.Select(p => GeoPoint.FromCartesian(p)).ToList())
        {}
        
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

        [DllImport("CGALWrapper", EntryPoint = "getPolygonSpine", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr getPolygonSpine(string wktPolygon);
        public List<GeoPoint> getCenterLine()
        {
            IntPtr pstr = getPolygonSpine(this.ToWtk());
            string wktSkeleton = Marshal.PtrToStringAnsi(pstr);

            SqlGeography geom = SqlGeography.STGeomFromText(new SqlChars(wktSkeleton), 4326);

            List<GeoPoint> points = new List<GeoPoint>();

            for (int i = 2; i < geom.STNumPoints(); i++)
            {
                double lat = (double)geom.STPointN(i).Lat;
                double lon = (double)geom.STPointN(i).Long;
                points.Add(new GeoPoint(lat, lon));
            }
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

