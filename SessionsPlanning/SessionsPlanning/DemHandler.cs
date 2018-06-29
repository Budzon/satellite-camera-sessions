using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using SphericalGeom;
using Common;
using Gdal = OSGeo.GDAL.Gdal;

namespace SessionsPlanning
{   
    public class DMS
    {
        public uint Degrees { get; private set; }
        public uint Minutes { get; private set; }
        public double Seconds { get; private set; }
        public bool Positive { get; private set; }

        public string LatLetter { get { return Positive ? "N" : "S"; } }
        public string LonLetter { get { return Positive ? "E" : "W"; } }
        public double DecimalDegrees
        {
            get
            {
                double abs = Degrees + Minutes / 60.0 + Seconds / 3600.0;
                return Positive ? abs : -abs;
            }
        }

        public DMS(uint d, uint m, double s, bool positive = true)
        {
            if (Comparison.IsNegative(s))
                throw new ArgumentException("Seconds must be nonnegative.");
            Degrees = d;
            Minutes = m;
            Seconds = s;
            Positive = positive;
        }

        public DMS(double dd)
        {
            Positive = Comparison.IsPositive(dd);
            dd = Math.Abs(dd);
            Degrees = (uint)Math.Truncate(dd);
            Minutes = (uint)((dd - Degrees) * 60);
            Seconds = (dd - Degrees - Minutes / 60.0) * 3600;
        }
    }

    public struct RasterData
    {
        public Int16[] Data { get; set; }
        public int xSize { get; set; }
        public int ySize { get; set; }
        public double[] GeoTransform { get; set; }
    }

    public class DemHandler
    {
        public string Path { get; private set; }

        public DemHandler(string path)
        {
            Path = path;
            GdalConfiguration.ConfigureGdal();
        }

        private static string SrtmName(DMS lat, DMS lon)
        {
            return String.Format("{0}{1}{2}{3}.hgt", lat.LatLetter, lat.Degrees.ToString("D2"), lon.LonLetter, lon.Degrees.ToString("D3"));
        }

        private RasterData GetRaster(DMS lat, DMS lon)
        {
            var dataSet = Gdal.Open(Path + SrtmName(lat, lon), OSGeo.GDAL.Access.GA_ReadOnly);

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

        public Int16 GetHeight(GeoPoint gp)
        {
            DMS lat = new DMS(gp.Latitude);
            DMS lon = new DMS(gp.Longitude);

            RasterData rasterData = GetRaster(lat, lon);
            int row = (int)Math.Ceiling((lat.DecimalDegrees - rasterData.GeoTransform[3]) / rasterData.GeoTransform[5]);
            int col = (int)Math.Ceiling((lon.DecimalDegrees - rasterData.GeoTransform[0]) / rasterData.GeoTransform[1]);
            int index = rasterData.ySize * (row - 1) + col;

            return rasterData.Data[index];
        }
    }
}
