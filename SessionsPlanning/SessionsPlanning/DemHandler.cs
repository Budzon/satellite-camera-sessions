using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

using SphericalGeom;
using Common;
using Gdal = OSGeo.GDAL.Gdal;

namespace SessionsPlanning
{   
    //public class DMS
    //{
    //    public uint Degrees { get; private set; }
    //    public uint Minutes { get; private set; }
    //    public double Seconds { get; private set; }
    //    public bool Positive { get; private set; }

    //    public string LatLetter { get { return Positive ? "N" : "S"; } }
    //    public string LonLetter { get { return Positive ? "E" : "W"; } }
    //    public double DecimalDegrees
    //    {
    //        get
    //        {
    //            double abs = Degrees + Minutes / 60.0 + Seconds / 3600.0;
    //            return Positive ? abs : -abs;
    //        }
    //    }

    //    public DMS(uint d, uint m, double s, bool positive = true)
    //    {
    //        if (Comparison.IsNegative(s))
    //            throw new ArgumentException("Seconds must be nonnegative.");
    //        Degrees = d;
    //        Minutes = m;
    //        Seconds = s;
    //        Positive = positive;
    //    }

    //    public DMS(double dd)
    //    {
    //        Positive = Comparison.IsPositive(dd);
    //        dd = Math.Abs(dd);
    //        Degrees = (uint)Math.Truncate(dd);
    //        Minutes = (uint)((dd - Degrees) * 60);
    //        Seconds = (dd - Degrees - Minutes / 60.0) * 3600;
    //    }
    //}

    public struct RasterData
    {
        public Int16[] Data { get; set; }
        public int xSize { get; set; }
        public int ySize { get; set; }
        public double[] GeoTransform { get; set; }
    }

    public class Tile
    {
        public uint Latitude { get; set; }
        public uint Longitude { get; set; }
        public bool North { get; set; }
        public bool East { get; set; }

        public string Name
        {
            get
            {
                return String.Format("{0}{1}{2}{3}.hgt", North ? "N" : "S", Latitude.ToString("D2"), East ? "E" : "W", Longitude.ToString("D3"));
            }
        }

        public Tile(GeoPoint gp)
        {
            North = Comparison.IsPositive(gp.Latitude);
            East = Comparison.IsPositive(gp.Longitude);
            double alat = Math.Abs(gp.Latitude);
            double alon = Math.Abs(gp.Longitude);
            Latitude = (uint)(North ? Math.Floor(alat) : Math.Ceiling(alat));
            Longitude = (uint)(East ? Math.Floor(alon) : Math.Ceiling(alon));
        }
    }

    public class DemHandler
    {
        public string Path { get; private set; }
        private Random rand;

        public DemHandler(string path)
        {
            rand = new Random();
            Path = path;
            GdalConfiguration.ConfigureGdal();
        }

        private RasterData GetRaster(GeoPoint gp)
        {
            return GetRaster(new Tile(gp));
        }

        private RasterData GetRaster(Tile tile)
        {
            var dataSet = Gdal.Open(Path + tile.Name, OSGeo.GDAL.Access.GA_ReadOnly);

            if (dataSet.RasterCount != 1)
                throw new Exception(String.Format("Wrong number of raster bands: {0}", dataSet.RasterCount));

            var rasterBand = dataSet.GetRasterBand(1);
            int xs = rasterBand.XSize;
            int ys = rasterBand.YSize;

            if (rasterBand.DataType != OSGeo.GDAL.DataType.GDT_Int16)
                throw new Exception(String.Format("Wrong data format: {0}", rasterBand.DataType));

            Int16[] data = new Int16[xs * ys];
            rasterBand.ReadRaster(0, 0, xs, ys, data, xs, ys, 0, 0);

            double[] geoTransform = new double[6];
            dataSet.GetGeoTransform(geoTransform);
            
            return new RasterData { Data = data, xSize = xs, ySize = ys, GeoTransform = geoTransform };
        }

        private Int16 GetHeight(RasterData rasterData, double lat, double lon)
        {
            int row = (int)Math.Ceiling((lat - rasterData.GeoTransform[3]) / rasterData.GeoTransform[5]);
            int col = (int)Math.Ceiling((lon - rasterData.GeoTransform[0]) / rasterData.GeoTransform[1]);
            int index = rasterData.ySize * (row - 1) + col;

            return rasterData.Data[index];
        }

        private Int16 GetHeight(RasterData rasterData, GeoPoint gp)
        {
            return GetHeight(rasterData, gp.Latitude, gp.Longitude);
        }

        public Int16 GetHeight(GeoPoint gp)
        {
            RasterData rasterData = GetRaster(new Tile(gp));
            return GetHeight(rasterData, gp.Latitude, gp.Longitude);
        }

        //private List<Int16> GetCoveredHeights(Polygon p)
        //{
        //    GeoPoint inner = GeoPoint.FromCartesian(p.PointInside());
        //    var data = GetRaster(inner);

        //    // if less than 3 arcseconds
        //    if (p.Diameter < 3.5 / 60 / 180 * Math.PI)
        //        return new List<Int16> { GetHeight(data, inner) };
        //    else
        //    {
                
        //    }
        //}

        public Int16 GetAverageHeight(Polygon p, uint samples = 1000)
        {
            int height = 0;
            Vector3D v;
            Dictionary<string, RasterData> data = new Dictionary<string,RasterData>();

            for (int i = 0; i < samples; ++i)
            {
                do
                {
                    v = RandomConvexCombination(p.Vertices);
                } while (!p.Contains(v));
                GeoPoint gp = GeoPoint.FromCartesian(v);
                Tile tile = new Tile(gp);
                if (!data.ContainsKey(tile.Name))
                {
                    data.Add(tile.Name, GetRaster(tile));
                }
                height += GetHeight(data[tile.Name], gp);
            }
            return (Int16) (height / samples);
        }

        private Vector3D RandomConvexCombination(IList<Vector3D> verts)
        {
            int from = rand.Next(0, verts.Count - 1);
            Vector3D v = new Vector3D(0, 0, 0);
            double curLambda, leftLambda = 1;
            for (int i = 0; i < verts.Count - 1; ++i)
            {
                curLambda = rand.NextDouble() * leftLambda;
                v += curLambda * verts[(from + i) % verts.Count];
                leftLambda -= curLambda;
            }
            return v;
        }

        //private List<GeoPoint> SampleSquare(GeoPoint lowerLeft, uint count = 1000)
        //{
        //    var output = new List<GeoPoint>();
        //    for (uint i = 0; i < count; ++i)
        //        output.Add(new GeoPoint(lowerLeft.Latitude + rand.NextDouble(), lowerLeft.Longitude + rand.NextDouble()));
        //    return output;
        //}
    }
}
