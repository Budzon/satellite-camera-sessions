using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.Windows.Media.Media3D;
using System.Linq;
using Astronomy;
using DBTables;
using SatelliteSessions;

namespace GeometryTest
{
    [TestClass]
    public class CreateSessionTest
    {
        [TestMethod]
        public void TestCreateCommunicationSessions()
        {
            CommunicationZoneSNKPOI sZone = new CommunicationZoneSNKPOI();
            sZone.Radius5 = 2437.6495031903974;
            sZone.Radius7 = 2257.4966496643665;
            sZone.CentreLon = 31.690301;
            sZone.CentreLat = 30.185089;

            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager managerDB = new DIOS.Common.SqlManager(cs);
            DataFetcher fetcher = new DataFetcher(managerDB);

            DateTime timeFrom = DateTime.Parse("01.01.2019 18:20:48");
            DateTime timeTo = DateTime.Parse("01.01.2019 18:50:51");

            Trajectory fullTrajectory = fetcher.GetTrajectorySat(timeFrom, timeTo);

            List<CommunicationSession> sessions = new List<CommunicationSession>();
            CommunicationSession.getSessionFromZone(sZone, fullTrajectory, sessions);

            CommunicationSession testSession = sessions[0];

            DateTime dtFrom = testSession.DropInterval.dateFrom;
            DateTime dtTo = testSession.DropInterval.dateTo;

            for (int i = 0; i < 10; i++)
            {
                int diff = -5 * i - 2;
                DateTime testDt = dtFrom.AddSeconds(diff);
                TrajectoryPoint testPoint = fullTrajectory.GetPoint(testDt);
                if (sZone.isPointInZone(testPoint.Position, sZone.Radius5))
                {
                    throw new Exception(String.Format("Точка, Упреждающая dtFrom на {0} сек, попала в зону", diff));
                }
            }

            int durationSec = (int)(dtTo - dtFrom).TotalSeconds;
            for (int i = 0; i < durationSec; i++)
            {
                DateTime testDt = dtFrom.AddSeconds(i);
                TrajectoryPoint testPoint = fullTrajectory.GetPoint(testDt);
                if (!sZone.isPointInZone(testPoint.Position, sZone.Radius5))
                {
                    throw new Exception(String.Format("Точка, Упреждающая dtFrom на {0} сек, не попала в зону, хотя должна", i));
                }
            }

            for (int i = 0; i < 10; i++)
            {
                int diff = 5 * i + 1;
                DateTime testDt = dtTo.AddSeconds(diff);
                TrajectoryPoint testPoint = fullTrajectory.GetPoint(testDt);
                if (sZone.isPointInZone(testPoint.Position, sZone.Radius5))
                {
                    throw new Exception(String.Format("Точка, опережающая dtTo на {0} сек, попала в зону", diff));
                }
            }

            //Sessions target = new Sessions();
            //PrivateObject obj = new PrivateObject(target);
            //object[] parametersArray = new object[] { sZone, fullTrajectory, sessions };
            //var retVal = obj.Invoke("getSessionFromZone", parametersArray);

            //Assert.AreEqual(retVal, expectedVal);
        }
    }
}
