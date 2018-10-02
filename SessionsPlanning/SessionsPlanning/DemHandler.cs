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
        public int[] Data { get; set; }
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
            North = !Comparison.IsNegative(gp.Latitude);
            East = !Comparison.IsNegative(gp.Longitude);
            double alat = Math.Abs(gp.Latitude);
            double alon = Math.Abs(gp.Longitude);
            Latitude = (uint)(North ? Math.Floor(alat) : Math.Ceiling(alat));
            Longitude = (uint)(East ? Math.Floor(alon) : Math.Ceiling(alon));
        }
    }

    public class DemHandler
    {
        public string Path { get; private set; }
        public string Prefix { get; private set; }
        public bool Zip { get; private set; }
        public bool Ftp { get; private set; }
        private static Random rand = new Random();

        public static HashSet<string> nonzeroDems = null;
        private static Dictionary<string, RasterData> loadedRasters = new Dictionary<string, RasterData>();
        //private static const int maxNumOfLoadedRasters = 100;

        public DemHandler()
        {
            if (nonzeroDems == null)
            {
                nonzeroDems = new HashSet<string>(System.IO.File.ReadLines("known_dem_cut.list"));
            }

            Path = System.IO.File.ReadLines("dem_location.conf").First();
            Prefix = "";
            Zip = true;
            Ftp = true;
            Prefix = "/vsizip/vsicurl/";
            GdalConfiguration.ConfigureGdal();
        }

        public DemHandler(string path, string pathToNonzeroDemList) : this(path, true, true, pathToNonzeroDemList) { }

        public DemHandler(string path, bool zip, bool ftp, string pathToNonzeroDemList)
        {
            if (nonzeroDems == null)
            {
                nonzeroDems = new HashSet<string>(System.IO.File.ReadLines(pathToNonzeroDemList));
            }
            
            Path = path;
            Prefix = "";
            Zip = zip;
            Ftp = ftp;
            if (zip)
                Prefix += Prefix.Length == 0 ? "/vsizip/" : "vsizip/";
            if (ftp)
                Prefix += Prefix.Length == 0 ? "/vsicurl/" : "vsicurl/";
            GdalConfiguration.ConfigureGdal();
        }

        private RasterData GetRaster(GeoPoint gp)
        {
            return GetRaster(new Tile(gp));
        }

        private RasterData GetRaster(Tile tile)
        {
            if (!loadedRasters.ContainsKey(tile.Name))
            {
                var dataSet = Gdal.Open(Prefix + Path + tile.Name + (Zip ? (".zip" + (Ftp ? "/" : "\\") + tile.Name) : ""), OSGeo.GDAL.Access.GA_ReadOnly);

                if (dataSet.RasterCount != 1)
                    throw new Exception(String.Format("Wrong number of raster bands: {0}", dataSet.RasterCount));

                var rasterBand = dataSet.GetRasterBand(1);
                int xs = rasterBand.XSize;
                int ys = rasterBand.YSize;

                if (rasterBand.DataType != OSGeo.GDAL.DataType.GDT_Int16)
                    throw new Exception(String.Format("Wrong data format: {0}", rasterBand.DataType));

                int[] data = new int[xs * ys];
                rasterBand.ReadRaster(0, 0, xs, ys, data, xs, ys, 0, 0);

                double[] geoTransform = new double[6];
                dataSet.GetGeoTransform(geoTransform);

                RasterData loaded = new RasterData { Data = data, xSize = xs, ySize = ys, GeoTransform = geoTransform };
                loadedRasters.Add(tile.Name, loaded);
            }

            return loadedRasters[tile.Name];
        }

        private int GetHeight(RasterData rasterData, double lat, double lon)
        {
            int row = (int)Math.Ceiling((lat - rasterData.GeoTransform[3]) / rasterData.GeoTransform[5]);
            int col = (int)Math.Ceiling((lon - rasterData.GeoTransform[0]) / rasterData.GeoTransform[1]);
            int index = rasterData.ySize * (row - 1) + col;

            return rasterData.Data[index];
        }

        private int GetHeight(RasterData rasterData, GeoPoint gp)
        {
            return GetHeight(rasterData, gp.Latitude, gp.Longitude);
        }

        public OSGeo.GDAL.Dataset Open(string path)
        {
            return Gdal.Open(path, OSGeo.GDAL.Access.GA_ReadOnly);
        }

        public int GetHeight(GeoPoint gp)
        {
            Tile tile = new Tile(gp);
            if (nonzeroDems.Contains(tile.Name))
                return GetHeight(GetRaster(new Tile(gp)), gp);
            else
                return 0;
        }

        public int GetAverageHeight(Polygon p, uint samples = 1000)
        {
            int height = 0;
            Vector3D v;

            for (int i = 0; i < samples; ++i)
            {
                do
                {
                    v = RandomConvexCombination(p.Vertices);
                } while (!p.Contains(v));
                GeoPoint gp = GeoPoint.FromCartesian(v);
                height += GetHeight(gp);
            }
            return (int)(height / samples);
        }

        private Vector3D RandomConvexCombination(IList<Vector3D> verts)
        {
            int from = rand.Next(0, verts.Count - 1);
            Vector3D v = new Vector3D(0, 0, 0);
            double curLambda, remainingLambda = 1;
            for (int i = 0; i < verts.Count - 1; ++i)
            {
                curLambda = rand.NextDouble() * remainingLambda;
                v += curLambda * verts[(from + i) % verts.Count];
                remainingLambda -= curLambda;
            }
            return v;
        }
    }
}
