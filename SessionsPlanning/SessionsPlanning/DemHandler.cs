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
        
        public string Name
        {
            get
            {
                return String.Format("srtm_{0:00}_{1:00}", Longitude, Latitude);
            }
        }

        public Tile(GeoPoint gp)
        {
            Latitude = 1 + ((60 - (uint)Math.Ceiling(gp.Latitude)) / 5) % 24;
            Longitude = 1 + ((180 + (uint)Math.Floor(gp.Longitude)) / 5) % 72;
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
                string pathTail = "";
                if (Zip)
                {
                    pathTail = tile.Name + ".zip" + (Ftp ? "/" : "\\");
                }
                var dataSet = Gdal.Open(Prefix + Path + pathTail + tile.Name + ".asc", OSGeo.GDAL.Access.GA_ReadOnly);
                
                if (dataSet.RasterCount != 1)
                    throw new Exception(String.Format("Wrong number of raster bands: {0}", dataSet.RasterCount));

                var rasterBand = dataSet.GetRasterBand(1);
                int xs = rasterBand.XSize;
                int ys = rasterBand.YSize;

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
            int h = 0;
            using (var sw = System.IO.File.AppendText("srtm.log"))
            {
                sw.WriteLine("Fetching height at lat={0:0.####}, lon={1:0.####}...", gp.Latitude, gp.Longitude);
                if (Math.Abs(gp.Latitude) > 60 - 1e-12)
                {
                    sw.WriteLine("Latitude exceeds 60 degrees, no data there.");
                    h = 0;
                }
                else
                {
                    Tile tile = new Tile(gp);
                    sw.WriteLine("Looking for tile {0}", tile.Name);
                    if (nonzeroDems.Contains(tile.Name))
                    {
                        sw.WriteLine("Tile found in the list.");
                        h = GetHeight(GetRaster(tile), gp);
                    }
                    else
                    {
                        sw.WriteLine("Tile not found in the list.");
                        h = 0;
                    }
                }
                sw.WriteLine("Returning {0}", h);
                sw.WriteLine("\n\nKnown tiles as read from known_dem_cut.list (total {0})", nonzeroDems.Count);
                foreach (string s in nonzeroDems)
                    sw.WriteLine(s);
                sw.WriteLine("---------------------------------------------------------------");
                sw.WriteLine();
            }
            return h;
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
