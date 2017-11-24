using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.Serialization.Formatters.Binary;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;

using SphericalGeom;
using SatelliteRequests;

namespace ViewModel
{
    public class Requests : ObservableCollection<Request>
    {
        public Requests() : base()
        { }
    }

    public class EarthSatelliteViewModel : INotifyPropertyChanged
    {
        private Camera camera;
        private Polygon coneBase;
        private Polygon curRequest;
        private Polygon curIntersection;

        private readonly DelegateCommand addRequestCmd;
        private readonly DelegateCommand removeRequestCmd;
        private readonly DelegateCommand addPointCmd;
        private readonly DelegateCommand removePointCmd;
        private readonly DelegateCommand verifyIfRegionCanBeSeenCmd;

        public event PropertyChangedEventHandler PropertyChanged;

        public EarthSatelliteViewModel()
        {
            camera = new Camera();
            coneBase = SphericalGeometryRoutines.ProjectConeOntoSphere(camera.Position, camera.NormalsToCone);

            Requests = new Requests();
            //SelectedPoint = -1;
            //SelectedRequest = -1;

            addRequestCmd = new DelegateCommand(_ =>
            {
                Requests.Add(new Request(RequestId));
                RequestId = 0;
                RaisePropertyChanged("RequestId");
            });

            removeRequestCmd = new DelegateCommand(_ =>
            {
                Requests.RemoveAt(SelectedRequest);
            }, _ => { return SelectedRequest != -1; });

            addPointCmd = new DelegateCommand(_ =>
            {
                Requests.ElementAt(SelectedRequest).Polygon.Add(new SurfacePoint(NewPointLat, NewPointLon));
                NewPointLon = 0;
                NewPointLat = 0;
                RaisePropertyChanged("NewPointLon");
                RaisePropertyChanged("NewPointLat");
            }, _ => { return SelectedRequest != -1; });

            removePointCmd = new DelegateCommand(_ =>
            {
                Requests.ElementAt(SelectedRequest).Polygon.RemoveAt(SelectedPoint);
            }, _ => { return (SelectedRequest != -1 && SelectedPoint != -1); });

            verifyIfRegionCanBeSeenCmd = new DelegateCommand(_ =>
            {
                var result = camera.RegionCanBeSeen(Requests.ElementAt(SelectedRequest).Polygon.Select(point => new vector3(new direction3(point.Lat, point.Lon), 1)).ToList());

                RegionCanBeCaptured = result.Item1;
                SatellitePitch = result.Item2.Lat;
                SatelliteYaw = result.Item2.Lon;
                SatelliteRoll = result.Item3;
                RaisePropertyChanged("RegionCanBeCaptured");
                RaisePropertyChanged("SatelliteRoll");
                RaisePropertyChanged("SatellitePitch");
                RaisePropertyChanged("SatelliteYaw");
            }, _ => { return SelectedRequest != -1; });
        }

        public void RaisePropertyChanged(string s)
        {
            if (PropertyChanged != null)
                PropertyChanged(this, new PropertyChangedEventArgs(s));
        }

        public Requests Requests { get; set; }

        public double SatelliteLat { get; set; }
        public double SatelliteLon { get; set; }
        public double SatelliteAltitude
        {
            get { return camera.Position.Length() - 1; }
            set {
                camera.Position = new vector3(new direction3(SatelliteLat, SatelliteLon), value + 1); 
                coneBase = SphericalGeometryRoutines.ProjectConeOntoSphere(camera.Position, camera.NormalsToCone);
                curIntersection = null;
            }
        }
        public double SatelliteVerticalAngleOfView
        {
            get { return camera.verticalHalfAngleOfView * 2; }
            set { 
                camera.verticalHalfAngleOfView = value / 2; 
                coneBase = SphericalGeometryRoutines.ProjectConeOntoSphere(camera.Position, camera.NormalsToCone);
                curIntersection = null;
            }
        }
        public double SatelliteHorizontalAngleOfView
        {
            get { return camera.horizontalHalfAngleOfView * 2; }
            set { 
                camera.horizontalHalfAngleOfView = value / 2;
                coneBase = SphericalGeometryRoutines.ProjectConeOntoSphere(camera.Position, camera.NormalsToCone);
                curIntersection = null;
            }
        }

        public int RequestId { get; set; }
        private int selectedRequest;
        public int SelectedRequest
        {
            get
            {
                return selectedRequest;
            }
            set
            {
                selectedRequest = value;
                curRequest = new SphericalGeom.Polygon(
                    Requests[value].Polygon.Select(sp => new vector3(new direction3(sp.Lat, sp.Lon), 1)),
                    new vector3(0, 0, 0));
                curIntersection = null;
            }
        }

        public double NewPointLon { get; set; }
        public double NewPointLat { get; set; }
        public int SelectedPoint { get; set; }

        public bool PointInPolygon(double x, double y, double z)
        {
            return curRequest.Contains(new vector3(x, y, z));
        }

        public bool PointInCamera(double x, double y, double z)
        {
            return coneBase.Contains(new vector3(x, y, z));
        }

        public bool PointInIntersection(double x, double y, double z)
        {
            if (curIntersection == null)
                curIntersection = Polygon.Intersect(curRequest, coneBase);

            return curIntersection.Contains(new vector3(x, y, z));
        }
        
        public bool RegionCanBeCaptured { get; private set; }
        public double SatelliteRoll { get; private set; }
        public double SatellitePitch { get; private set; }
        public double SatelliteYaw { get; private set; }

        public ICommand AddRequestCmd { get { return addRequestCmd; } }
        public ICommand RemoveRequestCmd { get { return removeRequestCmd; } }
        public ICommand AddPointCmd { get { return addPointCmd; } }
        public ICommand RemovePointCmd { get { return removePointCmd; } }
        public ICommand VerifyIfRegionCanBeSeenCmd { get { return verifyIfRegionCanBeSeenCmd; } }
    }

    public class DegreeToRadianConverter : IValueConverter
    {
        public object ConvertBack(object value, 
                              Type targetType, 
                              object parameter,
                              System.Globalization.CultureInfo culture)
        {
            return Double.Parse(value.ToString()) / 180 * Math.PI;
        }

        public object Convert(object value,
                                  Type targetType,
                                  object parameter,
                                  System.Globalization.CultureInfo culture)
        {
            return Double.Parse(value.ToString()) * 180 / Math.PI;
        }
    }

    public class DoubleInRange : ValidationRule
    {
        private double _min;
        private double _max;

        public DoubleInRange()
        {
        }

        public double Min
        {
            get { return _min; }
            set { _min = value; }
        }

        public double Max
        {
            get { return _max; }
            set { _max = value; }
        }

        public override ValidationResult Validate(object value,
            System.Globalization.CultureInfo cultureInfo)
        {
            double val = 0;

            try
            {
                if (((string)value).Length > 0)
                    val = Double.Parse((String)value);
            }
            catch (Exception e)
            {
                return new ValidationResult(false, "Illegal characters or " + e.Message);
            }

            if ((Min < Max) && ((val < Min) || (val > Max)) || (val < Min) && (Min > Max))
            {
                return new ValidationResult(false,
                  "Must be in range: " + Min + " - " + Max + ".");
            }
            else
            {
                return new ValidationResult(true, null);
            }
        }
    }

    public class CanComputeToTextConverter : IValueConverter
    {
        public object ConvertBack(object value,
                              Type targetType,
                              object parameter,
                              System.Globalization.CultureInfo culture)
        {
            return true;
        }

        public object Convert(object value,
                                  Type targetType,
                                  object parameter,
                                  System.Globalization.CultureInfo culture)
        {
            if (bool.Parse(value.ToString()))
            {
                return "Region can be captured?";
            }
            else
            {
                return "Computing...";
            } 
        }
    }

    public class DelegateCommand : ICommand
    {
        private readonly Predicate<object> _canExecute;
        private readonly Action<object> _execute;

        public event EventHandler CanExecuteChanged;

        public DelegateCommand(Action<object> execute)
                       : this(execute, null)
        {
        }

        public DelegateCommand(Action<object> execute,
                       Predicate<object> canExecute)
        {
            _execute = execute;
            _canExecute = canExecute;
        }

        public bool CanExecute(object parameter)
        {
            if (_canExecute == null)
            {
                return true;
            }

            return _canExecute(parameter);
        }

        public void Execute(object parameter)
        {
            _execute(parameter);
        }

        public void RaiseCanExecuteChanged()
        {
            if (CanExecuteChanged != null)
            {
                CanExecuteChanged(this, EventArgs.Empty);
            }
        }
    }
}
