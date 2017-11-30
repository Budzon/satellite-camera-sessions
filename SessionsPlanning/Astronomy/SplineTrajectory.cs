using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Astronomy;
using System.Windows.Media.Media3D;
using System.Collections.ObjectModel;

namespace Astronomy
{
    public class SplineCurve3d : IOrbitalCurve3d<double, double>
    {
        private double start, end;
        private CubicSpline splineX, splineY, splineZ;
        //private BSpline spline;
        private double[] times;
        private Quaternion[] quats;
        private IOrbitalCurveCursor3d<double, double> sharedCursor;

        public SplineCurve3d(TimeTuple[] points)
        {
            if (points == null) throw new ArgumentNullException("points");
            if (points.Length == 0) throw new ArgumentException("points is empty");
            times = points.Select(p => p.Time).ToArray();
            splineX = new CubicSpline(times, points.Select(p => p.Position.X).ToArray());
            splineY = new CubicSpline(times, points.Select(p => p.Position.Y).ToArray());
            splineZ = new CubicSpline(times, points.Select(p => p.Position.Z).ToArray());
            //spline = new BSpline(times, points.Select(p => p.Position.X).ToArray(), points.Select(p => p.Position.Y).ToArray(), points.Select(p => p.Position.Z).ToArray());
            quats = points.Select(p => p.Orientation).ToArray();
            this.start = times[0];
            this.end = times[times.Length - 1];
            sharedCursor = new SplineCurveCursor3d(this, start);
          
        }

        public SplineCurve3d GetPiece(double startTime, double endTime)
        {
            const double tolerance = 2.0;
            if (startTime - this.start < -tolerance || endTime - this.end > tolerance) throw new ArgumentException("invalid time range");
            List<TimeTuple> pts = new List<TimeTuple>();
            startTime = Math.Max(startTime, this.start);
            endTime = Math.Min(endTime, this.end);
            var cur = this.GetCursor(startTime);
            pts.Add(new TimeTuple(0, cur.Position, cur.Orientation));
            int i = 0;
            int maxi = times.Length;
            while (i < maxi && times[i] <= startTime) ++i;
            while (i < maxi && times[i] < endTime)
            {
                cur.MoveTo(times[i]);
                pts.Add(new TimeTuple(cur.Time - startTime, cur.Position, cur.Orientation));
                ++i;
            }
            cur.MoveTo(endTime);
            pts.Add(new TimeTuple(cur.Time - startTime, cur.Position, cur.Orientation));
            return new SplineCurve3d(pts.ToArray());
        }

        public double Min
        {
            get { return start; }
        }

        public double Max
        {
            get { return end; }
        }

        ICurveCursor3d<double, double> ICurve3d<double, double>.GetCursor(double time)
        {
            sharedCursor = new SplineCurveCursor3d(this, time);
            return sharedCursor;
        }

        public IOrbitalCurveCursor3d<double, double> GetCursor(double time)
        {
            sharedCursor = new SplineCurveCursor3d(this, time);
            return sharedCursor;
        }

        public IOrbitalCurveCursor3d<double, double> SharedCursor
        {
            get { return sharedCursor; }
        }

        private class SplineCurveCursor3d : IOrbitalCurveCursor3d<double, double>
        {
            private SplineCurve3d curve;
            private double time;
            private Point3D? position;
            private Vector3D? transv;
            private Quaternion? quat;
            private Vector3D? velocity;

            public SplineCurveCursor3d(SplineCurve3d curve, double time)
            {
                this.curve = curve;
                this.time = time;
            }

            public double Time
            {
                get { return time; }
            }

            public Point3D Position
            {
                get
                {
                    if (position == null)
                        //position = new Point3D(curve.spline.GetX(time), curve.spline.GetY(time), curve.spline.GetZ(time));
                        position = new Point3D(curve.splineX[time, true, true], curve.splineY[time, true, true], curve.splineZ[time, true, true]);
                    return position.Value;
                }
            }

            public Vector3D Velocity
            {
                get
                {
                    if (velocity == null)
                    {
                        Point3D prev, next;
                        const double delta = 1;
                        var position = Position;
                        double k = 1;

                        if (curve.end - time >= delta)
                        {
                            next = GetPosition(time + delta);
                            k = 2;
                        }
                        else
                            next = position;

                        if (time - curve.start >= delta)
                            prev = GetPosition(time - delta);
                        else
                        {
                            k = 1;
                            prev = position;
                        }

                        velocity = (next - prev) / (k * delta);

                    }
                    return velocity.Value;
                }
            }

            public Vector3D Transversal
            {
                get
                {
                    if (transv == null)
                    {
                        const double delta = 2;
                        var s0 = Position;
                        Vector3D a;
                        if (curve.end - time >= delta)
                        {
                            var s1 = GetPosition(time + delta);
                            a = s1 - s0;
                        }
                        else
                        {
                            var s1 = GetPosition(time - delta);
                            a = s0 - s1;
                        }

                        var b = new Point3D(0, 0, 0) - s0;
                        var k = Vector3D.CrossProduct(a, b);
                        var ys = Vector3D.CrossProduct(k, -b);
                        ys.Normalize();
                        transv = ys;
                    }
                    return transv.Value;
                }
            }

            public Quaternion Orientation
            {
                get
                {
                    if (quat == null)
                    {
                        int i = Array.BinarySearch(curve.times, time);
                        if (i >= 0)
                            quat = curve.quats[i];
                        else
                        {
                            i = ~i;
                            if (i == 0)
                                quat = curve.quats[0];
                            else if (i == curve.times.Length)
                                quat = curve.quats[i - 1];
                            else
                            {
                                double t = (time - curve.times[i - 1]) / (curve.times[i] - curve.times[i - 1]);
                                quat = Quaternion.Slerp(curve.quats[i - 1], curve.quats[i], t, true);
                            }
                        }
                    }
                    return quat.Value;
                }
            }

            public bool Move(double timeDelta)
            {
                if (timeDelta == 0)
                    return time > curve.start && time < curve.end;
                time = (timeDelta > 0) ? Math.Min(curve.end, time + timeDelta) : Math.Max(curve.start, time + timeDelta);
                position = null;
                transv = null;
                quat = null;
                velocity = null;
                return time > curve.start && time < curve.end;
            }

            public bool MoveTo(double time)
            {
                return Move(time - this.time);
            }

            private Point3D GetPosition(double time)
            {
                return new Point3D(//curve.spline.GetX(time), curve.spline.GetY(time), curve.spline.GetZ(time));
                    curve.splineX[time, true, true], curve.splineY[time, true, true], curve.splineZ[time, true, true]);
            }
        }
    }


    public struct TimeTuple
    {
        private double time;
        private Point3D position;
        private Quaternion quaternion;

        /// <summary>
        /// Time in seconds.
        /// </summary>
        public double Time { get { return time; } } // sec
        /// <summary>
        /// Position in GSK, km.
        /// </summary>
        public Point3D Position { get { return position; } }
        public Quaternion Orientation { get { return quaternion; } }

        public TimeTuple(double time, Point3D pos)
        {
            this.time = time;
            this.position = pos;
            this.quaternion = new Quaternion();
        }

        public TimeTuple(double time, Point3D pos, Quaternion quaternion)
        {
            this.time = time;
            this.position = pos;
            this.quaternion = quaternion;
        }

        public override string ToString()
        {
            return String.Format("{0} sec: {1} ({2} km height)", time, position, position.ToVector().Length - Constants.EarthRadius);
        }
    }
}
