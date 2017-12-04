using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;

namespace Astronomy
{
    public abstract class PointsCurve3d<TTime, TSpan> : ICurve3d<TTime, TSpan>
            where TTime : IComparable<TTime>
    {
        private CurvePoint3d<TTime>[] points;

        protected PointsCurve3d(CurvePoint3d<TTime>[] points)
        {
            if (points == null) throw new ArgumentNullException("points");
            if (points.Length == 0) throw new ArgumentException("points is empty");
            this.points = points;
        }

        public TTime Min
        {
            get { return points[0].Time; }
        }

        public TTime Max
        {
            get { return points[points.Length - 1].Time; }
        }

        public ICurveCursor3d<TTime, TSpan> GetCursor(TTime time)
        {
            return new PointsCurve3dCursor(this, time);
        }

        protected abstract double GetInterpolationCoefficient(TTime t1, TTime t2, TTime t);

        protected abstract TTime Add(TTime t, TSpan delta);
        protected abstract TSpan Substract(TTime t1, TTime t2);


        private class PointsCurve3dCursor : ICurveCursor3d<TTime, TSpan>
        {
            private PointsCurve3d<TTime, TSpan> curve;
            private TTime time;
            private Point3D? position;
            private int indexGE = -1;

            internal PointsCurve3dCursor(PointsCurve3d<TTime, TSpan> curve, TTime time)
            {
                if (curve == null) throw new ArgumentNullException("curve");
                this.time = time;
                this.curve = curve;
            }

            public TTime Time
            {
                get { return time; }
            }

            public Point3D Position
            {
                get
                {
                    if (position == null)
                    {
                        if (indexGE == -1)
                            indexGE = GetNearestGE(time);
                        if (indexGE < 0 || indexGE >= curve.points.Length)
                            throw new ArgumentOutOfRangeException("Time is out of time range of the curve");
                        if (indexGE == 0) position = curve.points[0].Position;
                        else if (indexGE == curve.points.Length - 1) position = curve.points[curve.points.Length - 1].Position;
                        else position = GetPosition(time, indexGE);
                    }
                    return position.Value;
                }
            }

            public bool MoveTo(TTime time)
            {
                return Move(curve.Substract(time, this.time));
            }

            public bool Move(TSpan timeDelta)
            {
                TTime newTime = curve.Add(time, timeDelta);
                int r = newTime.CompareTo(time);
                if (r == 0) return true;

                int n = curve.points.Length;
                if (r > 0) // move forward
                {
                    if (indexGE != -1)
                    {
                        if (!(IsIndexGE(newTime, indexGE) ||
                            (++indexGE < n && IsIndexGE(newTime, indexGE)) ||
                            (++indexGE < n && IsIndexGE(newTime, indexGE))))
                        {
                            indexGE = GetNearestGE(newTime, indexGE + 1);
                        }
                    }
                    else
                    {
                        indexGE = GetNearestGE(newTime);
                    }
                }
                else // move backward
                {
                    if (indexGE != -1)
                    {
                        indexGE = GetNearestGE(newTime, 0, indexGE);
                        if (indexGE < 0) { position = curve.points[0].Position; return false; }
                        if (indexGE >= n) { position = curve.points[n - 1].Position; return false; }
                    }
                    else
                    {
                        indexGE = GetNearestGE(newTime);
                    }
                }

                bool result = true;
                if (indexGE < 0) { newTime = curve.points[0].Time; indexGE = 0; result = false; }
                if (indexGE >= n) { newTime = curve.points[0].Time; indexGE = n - 1; result = false; }

                Invalidate();
                time = newTime;
                return result;
            }

            private void Invalidate()
            {
                position = null;
            }

            private bool IsIndexGE(TTime t, int index)
            {
                return (curve.points[index].Time.CompareTo(t) >= 0);
            }

            /// <summary>
            /// Point with time "t" is between indexNext-1 and indexNext.
            /// </summary>
            /// <param name="t"></param>
            /// <param name="indexNext"></param>
            /// <returns></returns>
            private Point3D GetPosition(TTime t, int indexNext)
            {
                if (indexNext == 0)
                {
                    if (curve.points[0].Time.CompareTo(t) == 0) return curve.points[0].Position;
                    throw new ArgumentException("Value at the indexNext should be GE than given time");
                }
                TTime t0 = curve.points[indexNext - 1].Time;
                TTime t1 = curve.points[indexNext].Time;
                if (t.CompareTo(t0) == 0) return curve.points[indexNext - 1].Position;
                if (t.CompareTo(t1) == 0) return curve.points[indexNext].Position;
                double alpha = curve.GetInterpolationCoefficient(t0, t1, t);
                var p = alpha * (curve.points[indexNext].Position - curve.points[indexNext - 1].Position) + curve.points[indexNext - 1].Position;
                return p;
            }

            /// <summary>
            /// Gets the index of the nearest point, which has time greater or equal to the given.
            /// </summary>
            /// <param name="t"></param>
            /// <returns></returns>
            private int GetNearestGE(TTime t, int min = 0, int max = -1)
            {
                if (max == -1)
                {
                    int n = curve.points.Length;
                    max = n - 1;
                }
                if (t.CompareTo(curve.points[max].Time) > 0) return max + 1;
                if (t.CompareTo(curve.points[min].Time) < 0) return -1;
                var target = new CurvePoint3d<TTime>(t, new Point3D(), new Vector3D());
                int i = Array.BinarySearch(curve.points, min, max - min + 1, target, CurvePoint3dByTimeComparer<TTime>.Instance);
                if (i >= 0) return i;
                return ~i;
            }
        }
    }

    public class PointsCurve3dDateTime : PointsCurve3d<DateTime, TimeSpan>
    {
        public PointsCurve3dDateTime(CurvePoint3d<DateTime>[] points)
            : base(points)
        {
        }

        protected override double GetInterpolationCoefficient(DateTime t1, DateTime t2, DateTime t)
        {
            return (double)(t.Ticks - t1.Ticks) / (t2.Ticks - t1.Ticks);
        }

        protected override DateTime Add(DateTime t, TimeSpan delta)
        {
            return t + delta;
        }

        protected override TimeSpan Substract(DateTime t1, DateTime t2)
        {
            return t1 - t2;
        }
    }

    public class PointsCurve3dDouble : PointsCurve3d<double, double>
    {
        public PointsCurve3dDouble(CurvePoint3d<double>[] points)
            : base(points)
        {
        }

        protected override double GetInterpolationCoefficient(double t1, double t2, double t)
        {
            return (double)(t - t1) / (t2 - t1);
        }

        protected override double Add(double t, double delta)
        {
            return t + delta;
        }

        protected override double Substract(double t1, double t2)
        {
            return t1 - t2;
        }
    }

    public struct CurvePoint3d<TTime>
    {
        private TTime time;
        private Point3D position;
        private Vector3D velocity;

        public CurvePoint3d(TTime time, Point3D position, Vector3D velocity)
        {
            this.time = time;
            this.position = position;
            this.velocity = velocity;
        }

        public TTime Time { get { return time; } }
        /// <summary>
        /// Trajectory point in Greenwich coordinate systems (km).
        /// </summary>
        public Point3D Position { get { return position; } }
        /// <summary>
        /// Velocity in the point (km/s).
        /// </summary>
        public Vector3D Velocity { get { return velocity; } }

        public override string ToString()
        {
            return String.Format("{0}: pos {1}, vel {2}", time, position, velocity);
        }
    }

    internal class CurvePoint3dByTimeComparer<TTime> : IComparer<CurvePoint3d<TTime>>
            where TTime : IComparable<TTime>
    {
        private static CurvePoint3dByTimeComparer<TTime> instance;

        public static CurvePoint3dByTimeComparer<TTime> Instance
        {
            get
            {
                if (instance == null)
                    instance = new CurvePoint3dByTimeComparer<TTime>();
                return instance;
            }
        }

        #region IComparer<TrajectoryPoint> Members

        public int Compare(CurvePoint3d<TTime> x, CurvePoint3d<TTime> y)
        {
            return x.Time.CompareTo(y.Time);
        }

        #endregion
    }
}
