﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using DataParsers;
using SatelliteTrajectory;
using Astronomy;
using OptimalChain;

namespace SatelliteSessions
{
    public class Sessions
    {
        public static bool isRequestFeasible(RequestParams request, DateTime data_begin, DateTime data_end)
        {
            string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "trajectory_1day.dat";
            SatTrajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, data_begin, data_end); // @todo временно 
            //double viewAngle = AstronomyMath.ToRad(request.max_roll_angle);  // @todo так будем задавать предел по качеству (если оно будет задаваться максимальным углом крена) 
            double viewAngle = Math.PI / 2;  // пока берем полосу максимальной ширины

            SatLane viewLane = trajectory.getCaptureLane(0, viewAngle);
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

        public static IList<CaptureConf> getCaptureConfArray(IList<RequestParams> requests, DateTime data_begin, DateTime data_end)
        {
            List<CaptureConf> captureConfs = new List<CaptureConf>();

            string trajFileName = AppDomain.CurrentDomain.BaseDirectory + "\\" + "trajectory_1day.dat";
            SatTrajectory trajectory = DatParser.getTrajectoryFromDatFile(trajFileName, data_begin, data_end); // @todo временно
            // @todo вынести это в константы
            double viewAngle = OptimalChain.Constants.camera_angle; // угол обзора камеры
            //double viewAngle = AstronomyMath.ToRad(5); // на время тестирования
            double angleStep = viewAngle; // шаг равен углу обзора
            double min_roll_angle = AstronomyMath.ToRad(-45); // @todo пока нету предела по углу крена
            double max_roll_angle = AstronomyMath.ToRad(45);
            for (double rollAngle = min_roll_angle; rollAngle <= max_roll_angle; rollAngle += angleStep)
            {
                List<CaptureConf> laneCaptureConfs = new List<CaptureConf>(); // участки захвата для текущий линии захвата
                SatLane viewLane = trajectory.getCaptureLane(rollAngle, viewAngle, polygonStep: 15);
                foreach (var request in requests)
                {
                    List<CaptureConf> confs = viewLane.getCaptureConfs(request);
                    compressCConfArray(ref confs);
                    compressTwoCConfArrays(ref laneCaptureConfs, ref confs);       
                    laneCaptureConfs.AddRange(confs);
                }

                foreach (var conf in laneCaptureConfs)
                {                        
                    var pol = viewLane.getSegment(conf.dateFrom, conf.dateTo);
                    conf.wktPolygon = pol.ToWtk();
                    conf.square = pol.Area;
                    conf.rollAngle = rollAngle;
                    conf.pitchAngle = 0;
                }

                captureConfs.AddRange(laneCaptureConfs);
            }

            // return captureConfs;

            Graph g = new Graph(captureConfs);
            List<CaptureConf> optimalChain = g.findOptimalChain();
            return optimalChain;
        }

        //public static IList<CaptureConf> getOptimalChain(List<CaptureConf> strips)
        //{
        //    Graph g = new Graph(strips);
        //    List<CaptureConf> optimalChain = g.findOptimalChain();         
        //    return optimalChain;
        //}

        private static CaptureConf unitCaptureConfs(CaptureConf confs1, CaptureConf confs2)
        {
            CaptureConf newConf = new CaptureConf();
            newConf.pitchAngle = confs1.pitchAngle;
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

        private static bool isNeedUnit(CaptureConf confs1, CaptureConf confs2)
        {
            /// @todo добавить минимально допустимое расстояние (по времени)
            return ((confs1.dateFrom <= confs2.dateTo && confs2.dateTo <= confs1.dateTo)
                   || (confs1.dateFrom <= confs2.dateFrom && confs2.dateFrom <= confs1.dateTo));
        }

        private static void compressCConfArray(ref List<CaptureConf> confs)
        {
            for (int i = 0; i < confs.Count; i++)
            {
                for (int j = i+1; j < confs.Count; j++)
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
