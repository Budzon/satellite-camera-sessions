using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SphericalGeom
{
    public static class Comparison
    {
        private static double precision = 1e-7;

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
    }

    public class vector3
    {
        /* Properties */
        public double X
        {
            get; set;
        }
        public double Y
        {
            get; set;
        }
        public double Z
        {
            get; set;
        }

        /* Constructors */
        public vector3(double _x, double _y, double _z)
        {
            X = _x;
            Y = _y;
            Z = _z;
        }
        public vector3() : this(1, 1, 1) { }
        public vector3(direction3 direction, double length)
        {
            X = length * Math.Cos(direction.Lon) * Math.Cos(direction.Lat);
            Y = length * Math.Sin(direction.Lon) * Math.Cos(direction.Lat);
            Z = length * Math.Sin(direction.Lat);
        }

        /* Operators */
        public static vector3 operator+(vector3 a, vector3 b)
        {
            return new vector3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        }
        public static vector3 operator-(vector3 a, vector3 b)
        {
            return new vector3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        }
        public static vector3 operator*(double t, vector3 a)
        {
            return new vector3(t * a.X, t * a.Y, t * a.Z);
        }
        public static vector3 operator-(vector3 a)
        {
            return -1 * a;
        }
        public static double operator*(vector3 a, vector3 b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }
        public static double operator*(vector3 a, direction3 b)
        {
            return a * (new vector3(b, 1));
        }

        /* Methods */
        public double Length()
        {
            return Math.Sqrt(X * X + Y * Y + Z * Z);
        }
        public direction3 Normalized()
        {
            double lat = Math.Asin(Z / Length());
            double lon = Math.Atan2(Y, X);// + (Comparison.IsPositive(X) ? 0 : Math.PI);
            return new direction3(lat, lon);
        }
        public override string ToString()
        {
            return Normalized().ToString();
        }
    }

    public class direction3
    {
        private double lon, lat;

        /* Properties */
        public double Lon {
            get { return lon; }
            set
            {
                while (Comparison.IsBigger(value, Math.PI))
                    value -= 2 * Math.PI;
                while (Comparison.IsSmaller(value, -Math.PI))
                    value += 2 * Math.PI;

                lon = value;
            }
        }
        public double Lat {
            get { return lat; }
            set
            {
                bool needShiftLon = false;

                while (Comparison.IsBigger(value, Math.PI))
                {
                    value -= 2 * Math.PI;
                }
                while (Comparison.IsSmaller(value, -Math.PI))
                {
                    value += 2 * Math.PI;
                }

                if (Comparison.IsBigger(value, Math.PI / 2))
                {
                    value = Math.PI - value;
                    needShiftLon = true;
                }
                else if (Comparison.IsSmaller(value, -Math.PI / 2))
                {
                    value = -Math.PI - value;
                    needShiftLon = true;
                }
                lat = value;
                Lon = lon + (needShiftLon ? Math.PI : 0);
            }
        }

        /* Constructors */
        public direction3(double _lat, double _lon)
        {
            Lon = _lon;
            Lat = _lat;
        }
        public direction3() : this(0, 0) { }

        /* Operators */
        public static direction3 operator+(direction3 a, direction3 b)
        {
            return new direction3(a.Lat + b.Lat, a.Lon + b.Lon);
        }
        public static direction3 operator-(direction3 a, direction3 b)
        {
            return new direction3(a.Lat - b.Lat, a.Lon - b.Lon);
        }
        public static double operator*(direction3 a, direction3 b)
        {
            return (new vector3(a, 1)) * (new vector3(b, 1));
        }

        /* Methods */
        public direction3 Antipodal()
        {
            return new direction3(-lat, lon + Math.PI);
        }
        public override string ToString()
        {
            return lat * 180 / Math.PI + ", " + lon * 180 / Math.PI;
        }
    }

    public class ReferenceFrame
    {
        /* Properties */
        public vector3 OX { get; private set; }
        public vector3 OY { get; private set; }
        public vector3 OZ { get; private set; }

        /* Constructors */
        public ReferenceFrame(vector3 _ox, vector3 _oy, vector3 _oz)
        {
            OX = (1 / _ox.Length()) * _ox;
            OY = (1 / _oy.Length()) * _oy;
            OZ = (1 / _oz.Length()) * _oz;
        }
        public ReferenceFrame(vector3 _oy)
            : this(new vector3(_oy.Y, -_oy.X, 0),
                    _oy,
                    new vector3(-_oy.X * _oy.Z, -_oy.Y * _oy.Z, _oy.X * _oy.X + _oy.Y * _oy.Y))
        { }
        public ReferenceFrame()
            : this(new vector3(1,0,0), new vector3(0,1,0), new vector3(0, 0, 1))
        { }

        /* Methods */
        public vector3 ToThisFrame(vector3 a)
        {
            return new vector3(
                OX.X * a.X + OX.Y * a.Y + OX.Z * a.Z,
                OY.X * a.X + OY.Y * a.Y + OY.Z * a.Z,
                OZ.X * a.X + OZ.Y * a.Y + OZ.Z * a.Z);
        }
        public vector3 ToThisFrame(double x, double y, double z)
        {
            return new vector3(
                OX.X * x + OX.Y * y + OX.Z * z,
                OY.X * x + OY.Y * y + OY.Z * z,
                OZ.X * x + OZ.Y * y + OZ.Z * z);
        }
        public vector3 ToBaseFrame(vector3 a)
        {
            return new vector3(
                OX.X * a.X + OY.X * a.Y + OZ.X * a.Z,
                OX.Y * a.X + OY.Y * a.Y + OZ.Y * a.Z,
                OX.Z * a.X + OY.Z * a.Y + OZ.Z * a.Z);
        }
        public vector3 ToBaseFrame(double x, double y, double z)
        {
            return new vector3(
                OX.X * x + OY.X * y + OZ.X * z,
                OX.Y * x + OY.Y * y + OZ.Y * z,
                OX.Z * x + OY.Z * y + OZ.Z * z);
        }
    }

    public class Camera
    {
        private static double sightAngleStep = 1e-1;
        private static double sightDirectionStep = 5e-3;
        // Right orthonormal basis with -position as OY such that OZY contains the North pole
        private vector3 position;
        private direction3 positionDirection;
        private ReferenceFrame camPositionFrame;

        /* Properties */
        public vector3 Position
        {
            get { return position; ; }
            set
            {
                position = value;
                positionDirection = value.Normalized();

                camPositionFrame = new ReferenceFrame(
                    new vector3(-value.Y, value.X, 0),
                    -value,
                    new vector3(-value.X * value.Z, -value.Y * value.Z, value.X * value.X + value.Y * value.Y));
            }
        }
        public direction3 PositionDirection
        { get { return positionDirection; } }

        public double verticalHalfAngleOfView { get; set; }
        public double horizontalHalfAngleOfView { get; set; }

        /* Constructors */
        public Camera(vector3 _position, double _verticalHalfAngleOfView, double _horizontalHalfAngleOfView)
        {
            Position = _position;
            verticalHalfAngleOfView = _verticalHalfAngleOfView;
            horizontalHalfAngleOfView = _horizontalHalfAngleOfView;
        }
        public Camera() : this(new direction3(), 1, Math.PI / 6, Math.PI / 6) { }
        public Camera(direction3 direction, double altitude, double _verticalHalfAngleOfView, double _horizontalHalfAngleOfView)
            : this(new vector3(direction, altitude + 1), _verticalHalfAngleOfView, _horizontalHalfAngleOfView)
        { }

        /* Methods */
        public Tuple<bool, direction3, double> RegionCanBeSeen(List<vector3> pointsOfInterest)
        {
            if (pointsOfInterest.All(point => CanBeSeenFromPosition(point)))
                foreach (double sightAngle in PossibleSightAngles())
                {
                    foreach (var sightDirection in PossibleSightDirections())
                    {
                        ReferenceFrame camSightFrame = new ReferenceFrame(sightDirection.Item1);
                        if (GetPositiveNormalsToSightBoundaries(camSightFrame, sightAngle).All(normal => RegionInPositiveHalfspace(normal, pointsOfInterest)))
                            return new Tuple<bool, direction3, double>(true, sightDirection.Item2, sightAngle);
                    }
                }

            return new Tuple<bool, direction3, double>(false, new direction3(), 0);
        }


        /* Private methods */
        private IEnumerable<double> PossibleSightAngles()
        {
            double cur = 0;
            while (Comparison.IsSmaller(cur, Math.PI))
            {
                yield return cur;
                cur += sightAngleStep;
            }
        }
        private IEnumerable<Tuple<vector3, direction3>> PossibleSightDirections()
        {
            double maxAngle = Math.Asin(1 / Position.Length());
            double horizontalMaxAngle = maxAngle; //- horizontalHalfAngleOfView;
            double verticalMaxAngle = maxAngle; //- verticalHalfAngleOfView;

            double ch, cv, sh, sv;

            double verticalAngle = 0;
            double horizontalAngle;

            do
            {
                cv = Math.Cos(verticalAngle);
                sv = Math.Sin(verticalAngle);
                horizontalAngle = 0;
                do
                {
                    ch = Math.Cos(horizontalAngle);
                    sh = Math.Sin(horizontalAngle);

                    yield return new Tuple<vector3, direction3>(camPositionFrame.ToBaseFrame(sh * cv, ch * cv, sv), new direction3(verticalAngle, horizontalAngle));
                    yield return new Tuple<vector3, direction3>(camPositionFrame.ToBaseFrame(sh * cv, ch * cv, -sv), new direction3(-verticalAngle, horizontalAngle));
                    yield return new Tuple<vector3, direction3>(camPositionFrame.ToBaseFrame(-sh * cv, ch * cv, -sv), new direction3(-verticalAngle, -horizontalAngle));
                    yield return new Tuple<vector3, direction3>(camPositionFrame.ToBaseFrame(-sh * cv, ch * cv, sv), new direction3(verticalAngle, -horizontalAngle));

                    horizontalAngle += sightDirectionStep;
                } while (Comparison.IsSmaller(horizontalAngle, horizontalMaxAngle));
                verticalAngle += sightDirectionStep;
            } while (Comparison.IsSmaller(verticalAngle, verticalMaxAngle));
        }

        private bool CanBeSeenFromPosition(vector3 a)
        {
            return Comparison.IsBigger(Position * a, a.Length());
        }
        private List<vector3> GetPositiveNormalsToSightBoundaries(ReferenceFrame camSightFrame, double sightAngle)
        {
            double cv = Math.Cos(verticalHalfAngleOfView);
            double sv = Math.Sin(verticalHalfAngleOfView);
            double ch = Math.Cos(horizontalHalfAngleOfView);
            double sh = Math.Sin(horizontalHalfAngleOfView);
            double ca = Math.Cos(sightAngle);
            double sa = Math.Sin(sightAngle);

            return new List<vector3>()
            {
                camSightFrame.ToBaseFrame(sa*cv, sv, ca*cv),
                camSightFrame.ToBaseFrame(-sa*cv, sv, -ca*cv),
                camSightFrame.ToBaseFrame(ca*ch, sh, -sa*ch),
                camSightFrame.ToBaseFrame(-ca*ch, sh, sa*ch)
            };
        }
        private bool RegionInPositiveHalfspace(vector3 normal, List<vector3> pointsOfInterest)
        {
            double threshold = Position * normal;
            double[] pointsProjection = pointsOfInterest.Select(point => point * normal).ToArray();

            // For every arc of the spherical polygon, store the extreme value of the projection onto normal.
            double[] extremeProjection = new double[pointsProjection.Length];
            for (int i = 0; i < pointsProjection.Length; ++i)
            {
                extremeProjection[i] = ExtremeValueF(
                    pointsProjection[i],
                    pointsProjection[(i + 1) % pointsProjection.Length],
                    pointsOfInterest[i] * pointsOfInterest[(i + 1) % pointsProjection.Length]);
            }

            return (pointsProjection.All(proj => Comparison.IsBigger(proj, threshold)) && extremeProjection.All(proj => Comparison.IsBigger(proj, threshold)));
        }
        // F(t) measures the projection of a unit vector w(t) onto a given direction.
        // Here, w(t) follows a great circle between two unit vectors.
        // F1 = F(0), F2 = F(1), C = cos(w(0)^w(1))
        private double F(double F1, double F2, double C, double t)
        {
            return (F1 + t * (F2 - F1)) / Math.Sqrt(1 + 2 * (1 - C) * t * (t - 1));
        }
        // F(t) has a unique stationary point on (0,1), and we evaluate F at it.
        private double ExtremeValueF(double F1, double F2, double C)
        {
            double t;
            if (Comparison.IsZero(F1 - F2))
            {
                t = 0.5;
            }
            else
            {
                double a = F1 / (F2 - F1);
                double b = 2 * (1 - C);
                t = (2 + a * b) / b / (2 * a + 1);
            }

            return F(F1, F2, C, t);
        }
    }
}
