using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace SatelliteRequests
{
    public class Request : ObservableCollection<SurfacePoint>
    {
        public DateTime ExpectedFromTime { get; set; }
        public DateTime ExpectedUntilTime { get; set; }

        public Request(DateTime expectedFromTime,
                       DateTime expectedUntilTime) : base()
        {
            ExpectedFromTime = expectedFromTime;
            ExpectedUntilTime = expectedUntilTime;
        }

        public Request()
            : this(DateTime.UtcNow, DateTime.UtcNow)
        {}
    }

    public struct SurfacePoint
    {
        public double Lat, Lon;

        public SurfacePoint(double lat, double lon)
        {
            Lat = lat;
            Lon = lon;
        }

        public override string ToString()
        {
            return Lat * 180 / Math.PI + ", " + Lon * 180 / Math.PI;
        }
    }
}
