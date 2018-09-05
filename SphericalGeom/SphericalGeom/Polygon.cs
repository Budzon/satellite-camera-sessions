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
        private const double maxArea = 2.5505e+14; // максимально возможная площадь (половина земной)
        private bool knownVertices = false;
        private Vector3D[] vertices;

        public double Area { get; private set; }

        public Polygon(SqlGeography geography)
        {
            _geography = geography;
            calcArea();       
        }

        public Polygon(string wkt)
            :this(SqlGeography.STGeomFromText(new SqlChars(wkt), 4326))
        {}

        public bool IsValid()
        {
            return (bool)_geography.STIsValid();
        }

        public Polygon(IEnumerable<GeoPoint> points)
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
            calcArea();
        }
       
        public Polygon(IEnumerable<Vector3D> points)
            : this(points.Select(p => GeoPoint.FromCartesian(p)).ToList())
        {
            vertices = points.ToArray();
            knownVertices = true;
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

        public bool Contains(Vector3D testP)
        {
            return Contains(GeoPoint.FromCartesian(testP));
        }

        public void Add(Polygon other)
        {           
            _geography = _geography.STUnion(other._geography);
        }
        
        public string ToWtk()
        {
            return _geography.ToString();
        }

        public override string ToString()
        {
            return ToWtk();
        }

        public string ToWebglearthString()
        {
            string res = "WE.polygon([";
            for (int i = 1; i <= _geography.STNumPoints(); i++)
            {
                GeoPoint gpoint = new GeoPoint(
                    (double)_geography.STPointN(i).Lat,
                    (double)_geography.STPointN(i).Long);
                res += "[" + ((string)_geography.STPointN(i).Lat.ToSqlString()).Replace(',', '.') + ", " + ((string)_geography.STPointN(i).Long.ToSqlString()).Replace(',', '.') + "]";
                if (i != _geography.STNumPoints())
                    res += ",";
            }
            res += "]  );";
            return res;
        }

        public List<Polygon> BreakIntoLobes()
        {
            List<List<GeoPoint>> Lobes = new List<List<GeoPoint>>();
            Lobes.Add(new List<GeoPoint>
                {
                    new GeoPoint(0, 0),
                    new GeoPoint(90, 0),
                    new GeoPoint(0, -179.9),
                    new GeoPoint(-90, 0)
                });
            Lobes.Add(new List<GeoPoint>
                {
                    new GeoPoint(0, 179.9),
                    new GeoPoint(90, 0),
                    new GeoPoint(0, 0),
                    new GeoPoint(-90, 0)
                });
            List<Polygon> output = new List<Polygon>();
            foreach (var lobe in Lobes)
                output.AddRange(Intersect(this, new Polygon(lobe)));
            return output;
        }

        [DllImport("CGALWrapper", EntryPoint = "getPolygonSpine", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr getPolygonSpine(string wktPolygon);
        public List<GeoPoint> getCenterLine()
        {
            IntPtr pstr = getPolygonSpine(this.ToWtk());
            string wktSkeleton = Marshal.PtrToStringAnsi(pstr);
            List<GeoPoint> points = new List<GeoPoint>();

            try
            {
                SqlGeography geom = SqlGeography.STGeomFromText(new SqlChars(wktSkeleton), 4326);
                for (int i = 2; i < geom.STNumPoints(); i++)
                {
                    double lat = (double)geom.STPointN(i).Lat;
                    double lon = (double)geom.STPointN(i).Long;
                    points.Add(new GeoPoint(lat, lon));
                }
                return points;
            }
            catch (Exception e)
            {
                return points;
            }
        }

        public Vector3D[] Vertices
        {
            get
            {
                if (!knownVertices)
                    calcVertices();
                return vertices;
            }
        }

        public static Polygon Hemisphere(Vector3D capCenter)
        {
            var basic = new Polygon(new List<GeoPoint>{
                new GeoPoint(90, 0),
                new GeoPoint(0, -90),
                new GeoPoint(-90, 0),
                new GeoPoint(0, 90)
            });
            Vector3D basisVec = Vector3D.CrossProduct(new Vector3D(0, 0, 1), capCenter); //Vector3D.CrossProduct(capCenter,  new Vector3D(rand.NextDouble(), rand.NextDouble(), rand.NextDouble()));
            ReferenceFrame capFrame = new ReferenceFrame(
                basisVec,
                Vector3D.CrossProduct(basisVec, capCenter),
                capCenter);
            return basic.ToNewFrame(capFrame, true);
        }

        public Polygon ToNewFrame(ReferenceFrame frame, bool transpose)
        {
            return new Polygon(Vertices.Select(v => transpose ? frame.ToBaseFrame(v) : frame.ToThisFrame(v)));
        }

        private void calcVertices()
        {            
            int size = (int)_geography.STNumPoints();
            vertices = new Vector3D[size];
            for (int i = 1; i <= size; i++)
            {
                GeoPoint gpoint = new GeoPoint(
                    (double)_geography.STPointN(i).Lat,
                    (double)_geography.STPointN(i).Long);
                vertices[i-1] = GeoPoint.ToCartesian(gpoint, 1);
            }
            knownVertices = true;
        }

        public static List<Polygon> Intersect(Polygon p, Polygon q)
        {
            List<Polygon> intersects = new List<Polygon>();

            SqlGeography intersection = p._geography.STIntersection(q._geography);

            for (int i = 1; i <= intersection.STNumGeometries(); i++)
            {
                intersects.Add( new Polygon(intersection.STGeometryN(i)) );
            }
            return intersects;
        }

         
        public static List<Polygon> Subtract(Polygon p, Polygon q)
        {
            List<Polygon> res = new List<Polygon>();
            SqlGeography diff = p._geography.STDifference(q._geography);
            for (int i = 1; i <= diff.STNumGeometries(); i++)
            {
                res.Add(new Polygon(diff.STGeometryN(i)));
            }
            return res;
        }


		public static List<Polygon> Subtract(Polygon p, IEnumerable<Polygon> qs)
        {
            List<Polygon> diff = new List<Polygon>() { p };
            foreach (var q in qs)
            {
                List<Polygon> newDiff = new List<Polygon>();
                foreach (var d in diff)
                {
                    newDiff.AddRange(Subtract(d, q));
                }
                diff.Clear();
                diff = newDiff;
            }
            return diff;
        }
        

        public static Tuple<List<Polygon>, List<Polygon>> IntersectAndSubtract(Polygon p, Polygon q)
        {
            return Tuple.Create(Intersect(p, q), Subtract(p, q));
        }

        public static string getMultipolFromWkts(List<string> wkts)
        {
            string res = "";
            
            if (wkts.Count == 0)
                return res;

            foreach (var wkt in wkts)
            {
                res = res + wkt + "\n";
                if (wkt != wkts.Last())
                    res += ",";
            }
            res = "GEOMETRYCOLLECTION (" + res + ")";
            
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

        private void calcArea()
        { 
            Area = (double)_geography.STArea();
            if (Area >= maxArea)
                throw new ArgumentException("Wrong points order");
        }
    }
}

