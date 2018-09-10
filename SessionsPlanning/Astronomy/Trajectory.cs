using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;
using System.IO;
using System.Globalization;
using MNCbicSplne = MathNet.Numerics.Interpolation.CubicSpline; // пересечения с Astronomy CubicSpline
namespace Astronomy
{
    /// <summary>
    /// Represents a temporal-spatial trajectory.
    /// </summary>
    public class Trajectory : IEnumerable<TrajectoryPoint>
    {
        private TrajectoryPoint[] points;
        private TrajectoryCircuit[] cachedCircuits;

        private Tuple<MNCbicSplne, MNCbicSplne, MNCbicSplne> PosAkimaInterpolation;
        private Tuple<MNCbicSplne, MNCbicSplne, MNCbicSplne> VeloAkimaInterpolation;
        
        private Trajectory(TrajectoryPoint[] points)
        {
            if (points == null) throw new ArgumentNullException("points");
            if (points.Count() < 5) throw new ArgumentNullException("Number of points is less than 5");
            this.points = points;
            var timeArray = points.Select(p => (double)p.Time.Ticks).ToArray();

            PosAkimaInterpolation = Tuple.Create(
                MNCbicSplne.InterpolateAkima(timeArray, points.Select(p => p.Position.X).ToArray()),
                MNCbicSplne.InterpolateAkima(timeArray, points.Select(p => p.Position.Y).ToArray()),
                MNCbicSplne.InterpolateAkima(timeArray, points.Select(p => p.Position.Z).ToArray())
                );

            VeloAkimaInterpolation = Tuple.Create(
                MNCbicSplne.InterpolateAkima(timeArray, points.Select(p => p.Velocity.X).ToArray()),
                MNCbicSplne.InterpolateAkima(timeArray, points.Select(p => p.Velocity.Y).ToArray()),
                MNCbicSplne.InterpolateAkima(timeArray, points.Select(p => p.Velocity.Z).ToArray())
                );
        }
        
        /// <summary>
        /// минимальное количетсво точек, при котором возможно обеспечить интерполяцию спрайлами Акима
        /// </summary>
        public const int minNumPoints = 5;

        public TrajectorySource Source { get; set; }

        public TrajectoryPoint[] Points { get { return points; } }

        public TrajectoryPoint this[int index] { get { return points[index]; } }

        public int Count { get { return points.Length; } }

        /// <summary>
        /// продолжительность траектории в секундах
        /// </summary>
        public double Duration 
        { 
            get 
            {
                if (Count == 0)
                    return 0;
                else
                    return (points.Last().Time - points[0].Time).TotalSeconds;
            } 
        }

        public void Save(string path)
        {
            using (StreamWriter sw = new StreamWriter(path, false))
            {
                sw.WriteLine("Time,X,Y,Z,Vx,Vy,Vz");
                int length = points.Length;
                for (int i = 0; i < length; i++)
                {
                    var p = points[i];
                    sw.Write(p.Time.ToString(CultureInfo.InvariantCulture));
                    sw.Write(',');
                    sw.Write(p.Position.X.ToString(CultureInfo.InvariantCulture));
                    sw.Write(',');
                    sw.Write(p.Position.Y.ToString(CultureInfo.InvariantCulture));
                    sw.Write(',');
                    sw.Write(p.Position.Z.ToString(CultureInfo.InvariantCulture));
                    sw.Write(',');
                    sw.Write(p.Velocity.X.ToString(CultureInfo.InvariantCulture));
                    sw.Write(',');
                    sw.Write(p.Velocity.Y.ToString(CultureInfo.InvariantCulture));
                    sw.Write(',');
                    sw.Write(p.Velocity.Z.ToString(CultureInfo.InvariantCulture));
                    sw.WriteLine();
                }
            }
        }

        public static Trajectory Load(string path, DateTime? zeroTime = null)
        {
            List<TrajectoryPoint> points = new List<TrajectoryPoint>(128000);
            using (StreamReader sr = new StreamReader(path))
            {
                sr.ReadLine();
                string s;
                DateTime t;
                while (!String.IsNullOrEmpty((s = sr.ReadLine())))
                {
                    var items = s.Split(',');
                    if (zeroTime.HasValue) // then we have seconds from zero time
                    {
                        t = zeroTime.Value.AddSeconds(double.Parse(items[0], CultureInfo.InvariantCulture));
                    }
                    else
                    {
                        t = DateTime.Parse(items[0], CultureInfo.InvariantCulture);
                    }
                    double x = double.Parse(items[1], CultureInfo.InvariantCulture);
                    double y = double.Parse(items[2], CultureInfo.InvariantCulture);
                    double z = double.Parse(items[3], CultureInfo.InvariantCulture);
                    Point3D p = new Point3D(x, y, z);
                    x = double.Parse(items[4], CultureInfo.InvariantCulture);
                    y = double.Parse(items[5], CultureInfo.InvariantCulture);
                    z = double.Parse(items[6], CultureInfo.InvariantCulture);
                    Vector3D v = new Vector3D(x, y, z);
                    points.Add(new TrajectoryPoint(t, p, v));
                }
            }
            return new Trajectory(points.ToArray());
        }

        public static Trajectory Create(TrajectoryPoint[] points)
        {
            return new Trajectory(points.ToArray());
        }

        /// <summary>
        /// Computes the orbit with the starting time moment <paramref name="startTime"/>, <paramref name="location"/> 
        /// in the Greenwich coordinate system (km) and <paramref name="velocity"/> (km/s) for the given <paramref name="duration"/>.
        /// </summary>
        /// <param name="startTime"></param>
        /// <param name="location">In the Greenwich coordinate system (km)</param>
        /// <param name="velocity">in the Greenwich coordinate system (km/s)</param>
        /// <param name="duration"></param>
        /// <returns></returns>
        public static Trajectory Compute(DateTime startTime, Vector3D location, Vector3D velocity, TimeSpan duration)
        {
            // Atmoshpere coefficients (?):
            const double ap = 7;
            const double d_F = 76;
            const double d_F81 = 76;
            const double d_BK = 3.40e-2;
            double d_Kp = Magnit.ap2kp(ap);

            OrbitDE orbit = new OrbitDE(
                startTime.Day, startTime.Month, startTime.Year, startTime.Hour, startTime.Minute, startTime.Second,
                location.X / 1000, location.Y / 1000, location.Z / 1000, velocity.X, velocity.Y, velocity.Z,
                d_Kp, d_F, d_F81, d_BK);
            orbit.ComputeOrbit(duration.TotalSeconds / 1000);

            var data = orbit.m_data;
            int n = data.Rows.Count;
            TrajectoryPoint[] points = new TrajectoryPoint[n];
            // Columns: [0]Time [1]X [2]Y [3]Z [4]Xp [5]Yp [6]Zp
            for (int i = 0; i < n; i++)
            {
                var row = data.Rows[i];
                DateTime t = JulianDate.ToDateTime((double)row[0]);
                Point3D p = new Point3D((double)row[1] * 1000, (double)row[2] * 1000, (double)row[3] * 1000);
                Vector3D v = new Vector3D((double)row[4], (double)row[5], (double)row[6]);
                points[i] = new TrajectoryPoint(t, p, v);
            }
            return new Trajectory(points);
        }

        public override string ToString()
        {
            return String.Format("{0} points", points.Length);
        }

        /// <summary>
        /// Gets the index of the nearest point, which has time greater or equal to the given.
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public int GetNearestGE(DateTime t)
        {
            int n = points.Length;
            if (n == 0) throw new NotSupportedException("Trajectory has no points");
            if (t > points[n - 1].Time) return n - 1;
            if (t <= points[0].Time) return 0;
            TrajectoryPoint target = new TrajectoryPoint(t, new Point3D(), new Vector3D());
            int i = Array.BinarySearch<TrajectoryPoint>(points, target, TrajectoryPointsByTimeComparer.Instance);
            if (i >= 0) return i;
            return ~i;
        }

        public TrajectoryPoint GetPoint(DateTime t)
        {
            return new TrajectoryPoint(t, GetPosition(t), GetVelocity(t));
        }

        /// <summary>
        /// Point with time "t".
        /// </summary>
        /// <param name="t"></param>
        /// <param name="indexNext"></param>
        /// <returns></returns>
        public Point3D GetPosition(DateTime t)
        {
            double x = PosAkimaInterpolation.Item1.Interpolate(t.Ticks);
            double y = PosAkimaInterpolation.Item2.Interpolate(t.Ticks);
            double z = PosAkimaInterpolation.Item3.Interpolate(t.Ticks);
            return new Point3D(x, y, z);
        }

        /// <summary>
        /// Point velocity with time "t".
        /// </summary>
        /// <param name="t"></param>
        /// <param name="indexNext"></param>
        /// <returns></returns>
        public Vector3D GetVelocity(DateTime t)
        {
            double x = VeloAkimaInterpolation.Item1.Interpolate(t.Ticks);
            double y = VeloAkimaInterpolation.Item2.Interpolate(t.Ticks);
            double z = VeloAkimaInterpolation.Item3.Interpolate(t.Ticks);
            return new Vector3D(x, y, z);
        }

        public TrajectoryCircuit GetCircuit(int index)
        {
            if (cachedCircuits == null)
                BuildCircuits();
            if (index < 0) index = 0;
            else if (index >= cachedCircuits.Length) index = cachedCircuits.Length - 1;
            return cachedCircuits[index];
        }

        /// <summary>
        /// Gets the curcuit so that given time moment belongs to it.
        /// </summary>
        /// <param name="time"></param>
        /// <returns></returns>
        public TrajectoryCircuit? GetCircuit(DateTime time)
        {
            if (points.Length == 0) throw new InvalidOperationException("Trajectory has no points");
            if (cachedCircuits == null)
                BuildCircuits();

            int n = cachedCircuits.Length;
            int l = 0;
            int r = n - 1;
            while (r >= l)
            {
                int i = (l + r) / 2;
                TrajectoryCircuit c = cachedCircuits[i];

                bool left = time >= c.Start.Time;
                bool right = time <= c.End.Time;
                if (left && right) return c;
                if (!left) r = i - 1;
                else l = i + 1;
            }
            return null;
        }

        private void BuildCircuits()
        {
            int index = 0;
            List<TrajectoryCircuit> circuits = new List<TrajectoryCircuit>(128);
            TrajectoryPoint start = points[0];
            int length = points.Length;
            int k = 2;
            for (int i = 1; i < length; i++)
            {
                TrajectoryPoint pointPrev = points[i - 1];
                TrajectoryPoint point = points[i];
                if (point.Position.Z * pointPrev.Position.Z < 0 || point.Position.Z == 0) // end of the circuit
                {
                    if (--k != 0) continue;
                    TrajectoryPoint end = points[i - 1];
                    TrajectoryCircuit circuit = new TrajectoryCircuit(index++, start, end);
                    circuits.Add(circuit);
                    start = end;
                    k = 2;
                }
            }
            cachedCircuits = circuits.ToArray();
        }

        /// <summary>
        /// Проверяет максимальный временной шаг траектории, в случае необходимости интерполирует новую траеткорию с заданным шагом
        /// </summary>
        /// <param name="trajectory">базовая траектория</param>
        /// <param name="maxTimeStep">максимально допустимый шаг траектории по времени</param>
        /// <returns>траектория с шагом, меньшим чем maxTimeStep </returns>
        public static Trajectory changeMaximumTimeStep(Trajectory trajectory, double maxTimeStep)
        { 
            // максимальный шаг по времени между точками траектории
            double maxStep = trajectory.Points.Skip(1).Zip(trajectory.Points, (curr, prev) => (curr.Time - prev.Time).TotalSeconds).Max();
 
            if (maxStep <= maxTimeStep)
                return trajectory;
             
            int size = (int)Math.Ceiling(trajectory.Duration / maxTimeStep) + 1;
            TrajectoryPoint[] newTrajsPoints = new TrajectoryPoint[size];

            for (int i = 0; i < size-1; i++ )
            {
                DateTime t = trajectory.points[0].Time.AddSeconds(i*maxTimeStep);                
                newTrajsPoints[i] = trajectory.GetPoint(t);
            }

            newTrajsPoints[size - 1] = trajectory.GetPoint(trajectory.points.Last().Time);

            return Trajectory.Create(newTrajsPoints.ToArray());
        }

        //public static Trajectory calculateVelocity(Trajectory trajectory, double maxTimeStep)
        //{
        //}

        #region IEnumerable<TrajectoryPoint> Members

        public IEnumerator<TrajectoryPoint> GetEnumerator()
        {
            return points.Cast<TrajectoryPoint>().GetEnumerator();
        }

        #endregion

        #region IEnumerable Members

        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()
        {
            return points.GetEnumerator();
        }

        #endregion
    }

    public struct TrajectoryCircuit
    {
        private TrajectoryPoint start;
        private TrajectoryPoint end;
        private int index;

        public TrajectoryCircuit(int index, TrajectoryPoint start, TrajectoryPoint end)
        {
            this.index = index;
            this.start = start;
            this.end = end;
        }

        public TrajectoryPoint Start { get { return start; } }

        public TrajectoryPoint End { get { return end; } }

        public int Index { get { return index; } }

        public override string ToString()
        {
            return String.Format("{0}: {1} - {2}", index, start, end);
        }
    }

    public struct TrajectoryPoint
    {
        private DateTime time;
        private Point3D position;
        private Vector3D velocity;

        public TrajectoryPoint(DateTime time, Point3D position, Vector3D velocity)
        {
            this.time = time;
            this.position = position;
            this.velocity = velocity;
        }

        public DateTime Time { get { return time; } }
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

        public double getAngleToEartchSurface()
        {
            return Math.Asin(Astronomy.Constants.EarthRadius / this.Position.ToVector().Length);
        }

    }

    internal class TrajectoryPointsByTimeComparer : IComparer<TrajectoryPoint>
    {
        private static TrajectoryPointsByTimeComparer instance;

        public static TrajectoryPointsByTimeComparer Instance
        {
            get
            {
                if (instance == null)
                    instance = new TrajectoryPointsByTimeComparer();
                return instance;
            }
        }

        #region IComparer<TrajectoryPoint> Members

        public int Compare(TrajectoryPoint x, TrajectoryPoint y)
        {
            return DateTime.Compare(x.Time, y.Time);
        }

        #endregion
    }
}