using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.ComponentModel;

namespace Astronomy
{
    public class TrajectorySource : INotifyPropertyChanged
    {
        private string name;
        private TrajectoryType type;
        private bool isActive;
        private bool isVisible;
        private string path;

        public TrajectorySource(string name, TrajectoryType type, string path)
        {
            this.name = name;
            this.type = type;
            this.isActive = false;
            this.isVisible = true;
            this.path = path;
        }

        public string Name
        {
            get { return name; }
            set
            {
                if (name == value) return;
                name = value;
                RaisePropertyChanged("Name");
            }
        }

        public TrajectoryType Type
        {
            get { return type; }
            set
            {
                if (type == value) return;
                type = value;
                RaisePropertyChanged("Type");
            }
        }

        public bool IsActive
        {
            get { return isActive; }
            set
            {
                if (isActive == value) return;
                isActive = value;
                RaisePropertyChanged("IsActive"); 
            }
        }

        public bool IsVisible
        {
            get { return isVisible; }
            set
            {
                if (isVisible == value) return;
                isVisible = value;               
                RaisePropertyChanged("IsVisible");
            }
        }

        public string Path
        {
            get { return path; }
            set
            {
                if (path == value) return;
                path = value;
                RaisePropertyChanged("Path");
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        private void RaisePropertyChanged(string name)
        {
            if (PropertyChanged != null)
                PropertyChanged(this, new PropertyChangedEventArgs(name));
        }
    }

    public enum TrajectoryType
    {
        KA, KM
    }
}
