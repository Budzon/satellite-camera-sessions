using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;


namespace SphericalGeom
{
    public static class Comparison
    {
        private static double precision = 1e-12;

        public static bool IsPositive(double a)
        {
            return a > precision;
        }

        public static bool IsNegative(double a)
        {
            return a < -precision;
        }

        public static bool IsZero(double a)
        {
            return !IsPositive(a) && !IsNegative(a);
        }

        public static bool IsBigger(double a, double b)
        {
            return IsPositive(a - b);
        }

        public static bool IsSmaller(double a, double b)
        {
            return IsNegative(a - b);
        }

        public static bool IsEqual(double a, double b)
        {
            return IsZero(a - b);
        }

        public static bool IsZero(Vector3D v)
        {
            return IsZero(v.Length);
        }

        public static bool IsEqual(Vector3D v, Vector3D u)
        {
            return IsZero(v - u);
        }
    }
}
