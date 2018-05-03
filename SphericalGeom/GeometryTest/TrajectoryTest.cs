using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

using System.Collections.Generic;
using System.Linq;
using Astronomy;
using System.Windows.Media.Media3D;
using SphericalGeom;
using Common;
using OptimalChain;
using SatelliteSessions;
using DBTables;


namespace GeometryTest
{
    [TestClass]
    public class TrajectoryTest
    {
        [TestMethod]
        public void Test_GetTrajectorySat()
        {
            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);

            DateTime dt1 = new DateTime(2019, 1, 5, 21, 23, 1);
            DateTime dt2 = new DateTime(2019, 1, 6, 2, 1, 34);

            DataFetcher fetcher = new DataFetcher(manager);
            Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

            Assert.IsTrue(trajectory.Points[0].Time == dt1);
            Assert.IsTrue(trajectory.Points.Last().Time == dt2);

            Assert.IsTrue(trajectory.Points[0].Position == trajectory.GetPosition(dt1));
            Assert.IsTrue(trajectory.Points.Last().Position == trajectory.GetPosition(dt2));

            for (int i = 0; i < trajectory.Count; i++)
            {
                var dtime = trajectory.Points[i].Time;
                TrajectoryPoint? point = fetcher.GetPositionSat(dtime);
                Assert.IsTrue(point != null);
                var trpoint = (TrajectoryPoint)point;
                //var diff = Math.Abs((trpoint.Time - dtime).TotalMilliseconds);
                Assert.IsTrue(trpoint.Time == dtime);
            }
        }
    }
}
