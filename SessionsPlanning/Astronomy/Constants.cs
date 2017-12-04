using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;

namespace Astronomy
{
    public static class Constants
    {
        /// <summary>
        /// Radius of the Earth measured in kilometers.
        /// </summary>
        [Obsolete("Use SpaceConstants instead")]
        public const double EarthRadius = 6371.3;

        public static readonly Point3D NorthPole = new Point3D(0, 0, EarthRadius);

        public const double ToRad = Math.PI / 180.0;
        public const double ToDeg = 180.0 / Math.PI;

    }
}
