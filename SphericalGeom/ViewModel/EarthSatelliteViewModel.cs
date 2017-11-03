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
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;

using SphericalGeom;
using SatelliteRequests;

namespace ViewModel
{
    public class EarthSatelliteViewModel : System.Windows.Data.ListCollectionView, INotifyPropertyChanged
    {
        private Camera camera = new Camera();
        private SurfacePoint newPoint = new SurfacePoint(0, 0);

        private readonly DelegateCommand addPointCmd;
        private readonly DelegateCommand removePointCmd;
        private readonly DelegateCommand verifyIfRegionCanBeSeenCmd;

        public event PropertyChangedEventHandler PropertyChanged;

        public EarthSatelliteViewModel() : base(new Request())
        {
            //SelectedIndex = -1;
            CanPerformCalculation = true;
            addPointCmd = new DelegateCommand(_ =>
            {
                var region = SourceCollection as Request;
                region.Add(newPoint);
                newPoint = new SurfacePoint();
                NewPointLon = newPoint.Lon * 180 / Math.PI;
                NewPointLat = newPoint.Lat * 180 / Math.PI;
            });

            removePointCmd = new DelegateCommand(_ =>
            {
                var region = SourceCollection as Request;
                region.RemoveAt(SelectedIndex);
            }, _ => { return SelectedIndex != -1; });

            verifyIfRegionCanBeSeenCmd = new DelegateCommand(_ =>
            {
                CanPerformCalculation = false;
                RaisePropertyChanged("CanPerformCalculation");

                var region = (SourceCollection as Request).ToList();
                var result = camera.RegionCanBeSeen(region.Select(point => new vector3(new direction3(point.Lat, point.Lon), 1)).ToList());

                RegionCanBeCaptured = result.Item1;
                SatelliteSightDirectionAngleShifts = result.Item2;
                SatelliteAngleAboutSightDirection = result.Item3 * 180 / Math.PI;
                RaisePropertyChanged("RegionCanBeCaptured");
                RaisePropertyChanged("SatelliteSightDirectionAngleShifts");
                RaisePropertyChanged("SatelliteAngleAboutSightDirection");

                CanPerformCalculation = true;
                RaisePropertyChanged("CanPerformCalculation");
            });
        }

        public void RaisePropertyChanged(string s)
        {
            if (PropertyChanged != null)
                PropertyChanged(this, new PropertyChangedEventArgs(s));
        }

        public bool CanPerformCalculation { get; set; }

        public double NewPointLat
        {
            get { return newPoint.Lat * 180 / Math.PI; }
            set { newPoint.Lat = value * Math.PI / 180; RaisePropertyChanged("NewPointLat"); RaisePropertyChanged("NewPointLon"); }
        }
        public double NewPointLon
        {
            get { return newPoint.Lon * 180 / Math.PI; }
            set { newPoint.Lon = value * Math.PI / 180; RaisePropertyChanged("NewPointLongtitude"); }
        }

        public double SatelliteLat
        {
            get { return camera.PositionDirection.Lat * 180 / Math.PI; }
            set
            {
                camera.Position = new vector3(new direction3(value * Math.PI / 180, SatelliteLon * Math.PI / 180), SatelliteAltitude + 1);
                RaisePropertyChanged("SatelliteLat");
                RaisePropertyChanged("SatelliteLon");
            }
        }
        public double SatelliteLon
        {
            get { return camera.PositionDirection.Lon * 180 / Math.PI; }
            set
            {
                camera.Position = new vector3(new direction3(SatelliteLat * Math.PI / 180, value * Math.PI / 180), SatelliteAltitude + 1);
                RaisePropertyChanged("SatelliteLon");
            }
        }
        public double SatelliteAltitude
        {
            get { return camera.Position.Length() - 1; }
            set
            {
                camera.Position = new vector3(new direction3(SatelliteLat, SatelliteLon), value + 1);
                RaisePropertyChanged("SatelliteAltitude");
            }
        }
        public double SatelliteVerticalHalfAngleOfView
        {
            get { return camera.verticalHalfAngleOfView * 180 / Math.PI; }
            set { camera.verticalHalfAngleOfView = value * Math.PI / 180; RaisePropertyChanged("SatelliteVerticalHalfAngleOfView"); }
        }
        public double SatelliteHorizontalHalfAngleOfView
        {
            get { return camera.horizontalHalfAngleOfView * 180 / Math.PI; }
            set { camera.horizontalHalfAngleOfView = value * Math.PI / 180;  RaisePropertyChanged("SatelliteHorizontalHalfAngleOfView"); }
        }

        public int SelectedIndex { get; set; }

        public bool RegionCanBeCaptured { get; private set; }
        public direction3 SatelliteSightDirectionAngleShifts { get; private set; }
        public double SatelliteAngleAboutSightDirection { get; private set; }

        public ICommand AddPointCmd { get { return addPointCmd; } }
        public ICommand RemovePointCmd { get { return removePointCmd; } }
        public ICommand VerifyIfRegionCanBeSeenCmd { get { return verifyIfRegionCanBeSeenCmd; } }
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
