using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace SatelliteRequests
{
    public class Request
    {
        public int Id { get; set; }
        public DateTime ExpectedFromTime { get; set; }
        public DateTime ExpectedUntilTime { get; set; }
        public ObservableCollection<SurfacePoint> Polygon { get; set; }

        public Request(int id,
                       DateTime expectedFromTime,
                       DateTime expectedUntilTime)
        {
            Id = id;
            ExpectedFromTime = expectedFromTime;
            ExpectedUntilTime = expectedUntilTime;          
            Polygon = new ObservableCollection<SurfacePoint>();
        }

        public Request(int id) : this(id, DateTime.UtcNow, DateTime.UtcNow)
        {}

        public Request()
            : this(0, DateTime.UtcNow, DateTime.UtcNow)
        {}
    }

    public class SurfacePoint
    {
        public double Lat { get; set; }
        public double Lon { get; set; }

        public SurfacePoint(double lat, double lon)
        {
            Lat = lat;
            Lon = lon;
        }
    }
}
