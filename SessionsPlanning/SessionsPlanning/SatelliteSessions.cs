using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

using DataParsers;
using SatelliteTrajectory;
using Astronomy;
using Common;
using OptimalChain;
using SphericalGeom;

namespace SatelliteSessions
{
    public class Sessions
    {
        public static bool isRequestFeasible(RequestParams request, DateTime data_begin, DateTime data_end)
        {
            string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "trajectory_1day.dat";
            Astronomy.Trajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, data_begin, data_end); // @todo временно            
            double viewAngle = request.Max_SOEN_anlge + OptimalChain.Constants.camera_angle; // Math.PI / 2;  
            SatLane viewLane = new SatLane(trajectory, 0, viewAngle);
            List<CaptureConf> confs = viewLane.getCaptureConfs(request);

            double summ = 0;
            List<SphericalGeom.Polygon> region = new List<SphericalGeom.Polygon> { new SphericalGeom.Polygon(request.wktPolygon) };
            foreach (var conf in confs)
            {
                foreach (var order in conf.orders)
                {
                    var notCoveredBefore = new List<SphericalGeom.Polygon>();
                    var toBeCoveredAfter = new List<SphericalGeom.Polygon>();
                    for (int i = 0; i < region.Count; ++i)
                    {
                        var intAndSub = SphericalGeom.Polygon.IntersectAndSubtract(region[i], order.captured);
                        notCoveredBefore.AddRange(intAndSub.Item1);
                        toBeCoveredAfter.AddRange(intAndSub.Item2);
                    }
                    double areaNotCoveredBefore = 0;
                    for (int j = 0; j < notCoveredBefore.Count; ++j)
                    {
                        areaNotCoveredBefore += notCoveredBefore[j].Area;
                    }
                    summ += order.intersection_coeff * areaNotCoveredBefore / order.captured.Area;
                    notCoveredBefore.Clear();
                    region.Clear();
                    region = toBeCoveredAfter;
                }
            }
            return (summ >= request.minCoverPerc);
        }

        public static List<CaptureConf> getCaptureConfs(IList<RequestParams> requests, DateTime data_begin, DateTime data_end)
        {
            if (requests.Count == 0)
                throw new ArgumentException("Requests array is empty!");

            List<CaptureConf> captureConfs = new List<CaptureConf>();

            string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "\\" + "trajectory_1day.dat";
            Astronomy.Trajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, data_begin, data_end); // @todo временно

            // @todo вынести это в константы
            double viewAngle = OptimalChain.Constants.camera_angle; // угол обзора камеры
            //double viewAngle = AstronomyMath.ToRad(5); // на время тестирования
            double angleStep = viewAngle; // шаг равен углу обзора

            double Max_SOEN_anlge = requests[0].Max_SOEN_anlge;
            foreach (var req in requests)
            {
                if (req.Max_SOEN_anlge > Max_SOEN_anlge)
                    Max_SOEN_anlge = req.Max_SOEN_anlge;
            }

            double max_roll_angle = Math.Min(Max_SOEN_anlge, AstronomyMath.ToRad(45));
            double min_roll_angle = Math.Max(-Max_SOEN_anlge, AstronomyMath.ToRad(-45));

            //for (double rollAngle = min_roll_angle + angleStep; rollAngle <= max_roll_angle; rollAngle += angleStep)

            int num_steps = (int)((max_roll_angle - min_roll_angle) / angleStep); /// @todo что делать с остатком от деления?
            //for (double rollAngle = min_roll_angle; rollAngle <= max_roll_angle; rollAngle += angleStep)    {        
            Parallel.For(0, num_steps, index =>
            {
                double rollAngle = min_roll_angle + index * angleStep;
                List<CaptureConf> laneCaptureConfs = new List<CaptureConf>(); // участки захвата для текущий линии захвата
                SatLane viewLane = new SatLane(trajectory, rollAngle, viewAngle, polygonStep: 15);
                foreach (var request in requests)
                {
                    if (Math.Abs(rollAngle) > Math.Abs(request.Max_SOEN_anlge))
                        continue;
                    List<CaptureConf> confs = viewLane.getCaptureConfs(request);
                    compressCConfArray(ref confs);
                    compressTwoCConfArrays(ref laneCaptureConfs, ref confs);
                    laneCaptureConfs.AddRange(confs);
                }

                foreach (var conf in laneCaptureConfs)
                {
                    var segmentTuple = viewLane.getSegment(conf.dateFrom, conf.dateTo);
                    var pol = segmentTuple.Item1;
                    TrajectoryPoint pointFrom = segmentTuple.Item2;
                    TrajectoryPoint pointTo = segmentTuple.Item3;

                    conf.wktPolygon = pol.ToWtk();
                    conf.square = pol.Area;
                    conf.rollAngle = rollAngle;

                    calculatePitchArrays(conf, rollAngle, pointFrom);
                }
                captureConfs.AddRange(laneCaptureConfs);
            }
            );

            for (int ci = 0; ci < captureConfs.Count; ci++)
                captureConfs[ci].id = ci;

            return captureConfs;
            
            //var rnd = new Random();
            //var result = captureConfs.OrderBy(item => rnd.Next());
            //captureConfs = result.ToList<CaptureConf>();

            //var cconfss = new List<StaticConf>();
            //foreach (var cc in captureConfs)
            //{
            //    var stc = new StaticConf(cc.id, cc.dateFrom, cc.dateTo, cc.timeDelta, cc.rollAngle, cc.square, cc.orders, cc.wktPolygon);
            //    cconfss.Add(stc);
            //}
            //return cconfss;

        }
        public static IList<MPZ> getMPZArray(List<CaptureConf> captureConfs)
        {
            Graph g = new Graph(captureConfs);
            List<MPZ> optimalChain = g.findOptimalChain();// .findOptimalChain();
            return optimalChain;
        }
        public static IList<MPZ> getCaptureConfArray(IList<RequestParams> requests, DateTime data_begin, DateTime data_end)
        {
            List<CaptureConf> captureConfs = getCaptureConfs(requests, data_begin, data_end);
            return getMPZArray(captureConfs);            
        }

        private static void calculatePitchArrays(CaptureConf conf, double rollAngle, TrajectoryPoint pointFrom)
        {
            double maxAngle = conf.orders[0].request.Max_SOEN_anlge;
            foreach (var req in conf.orders)
            {
                if (req.request.Max_SOEN_anlge < maxAngle)
                    maxAngle = req.request.Max_SOEN_anlge;
            }

            double maxPitchAngle = Math.Abs(maxAngle) - Math.Abs(rollAngle);
            if (0 == maxPitchAngle)
            {
                conf.timeDelta = 0;
            }
            else
            {
                Vector3D rollPoint = LanePos.getSurfacePoint(pointFrom, rollAngle, 0);
                Vector3D PitchRollPoint = LanePos.getSurfacePoint(pointFrom, rollAngle, maxPitchAngle);
                rollPoint.Normalize();
                PitchRollPoint.Normalize();
                // расстояние в километрах между точкой c нулевым тангажом и точкой, полученной при максимальном угле тангажа
                double dist = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(rollPoint), GeoPoint.FromCartesian(PitchRollPoint)) * Astronomy.Constants.EarthRadius;
                // время, за которое спутник преодалевает dist по поверхности земли.
                conf.timeDelta = dist / pointFrom.Velocity.Length;
            }
            int pitchStep = OptimalChain.Constants.pitchStep;
            conf.pitchArray[0] = 0;

            Vector3D dirRollPoint = LanePos.getSurfacePoint(pointFrom, rollAngle, 0);
            for (int pitch_degr = pitchStep; pitch_degr <= AstronomyMath.ToDegrees(maxPitchAngle); pitch_degr += pitchStep)
            {
                double pitch = AstronomyMath.ToRad(pitch_degr);
                Vector3D dirPitchPoint = LanePos.getSurfacePoint(pointFrom, rollAngle, pitch);
                double distOverSurf = GeoPoint.DistanceOverSurface(GeoPoint.FromCartesian(dirPitchPoint), GeoPoint.FromCartesian(dirRollPoint)) * Astronomy.Constants.EarthRadius;
                double t = distOverSurf / pointFrom.Velocity.Length;
                conf.pitchArray[pitch] = t;
            }
        }

        private static CaptureConf unitCaptureConfs(CaptureConf confs1, CaptureConf confs2)
        {
            CaptureConf newConf = new CaptureConf();
            newConf.rollAngle = confs1.rollAngle;
            newConf.dateFrom = (confs1.dateFrom < confs2.dateFrom) ? confs1.dateFrom : confs2.dateFrom;
            newConf.dateTo = (confs1.dateTo > confs2.dateTo) ? confs1.dateTo : confs2.dateTo;
            newConf.orders.AddRange(confs1.orders);
            newConf.orders.AddRange(confs2.orders);

            /*
            for (int i = 0; i < confs1.orders.Count; i++)
            {
                newConf.orders.Add(confs1.orders[i]);
                for (int j = 0; j < confs2.orders.Count; j++)
                {
                    if (confs1.orders[i].request.id != confs2.orders[j].request.id)
                    {                       
                        newConf.orders.Add(confs2.orders[j]);                        
                    }
                    else
                    {
                        var order = new Order();
                        
                    }                    
                }
            }*/

            return newConf;
        }

        private static bool isNeedUnit(CaptureConf c1, CaptureConf c2)
        {
            /// @todo добавить минимально допустимое расстояние (по времени)
            return ((c1.dateFrom <= c2.dateTo && c2.dateTo <= c1.dateTo) || (c1.dateFrom <= c2.dateFrom && c2.dateFrom <= c1.dateTo)
                  || (c2.dateFrom <= c1.dateTo && c1.dateTo <= c2.dateTo) || (c2.dateFrom <= c1.dateFrom && c1.dateFrom <= c2.dateTo));
        }

        private static void compressCConfArray(ref List<CaptureConf> confs)
        {
            for (int i = 0; i < confs.Count; i++)
            {
                for (int j = i + 1; j < confs.Count; j++)
                {
                    if (isNeedUnit(confs[i], confs[j]))
                    {
                        CaptureConf comConf = unitCaptureConfs(confs[i], confs[j]);
                        confs[i] = comConf;
                        confs.RemoveAt(j);
                    }
                }
            }
        }

        private static void compressTwoCConfArrays(ref List<CaptureConf> confs1, ref List<CaptureConf> confs2)
        {
            for (int i = 0; i < confs1.Count; i++)
            {
                for (int j = 0; j < confs2.Count; j++)
                {
                    if (isNeedUnit(confs1[i], confs2[j]))
                    {
                        CaptureConf unitConf = unitCaptureConfs(confs1[i], confs2[j]);
                        confs1[i] = unitConf;
                        confs2.RemoveAt(j);
                    }
                }
            }
        }
    }
}
