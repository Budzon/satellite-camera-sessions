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

        public const double TrajectoryEps = 0.250;

        [TestMethod]
        public void Test_GetTrajectorySat()
        {
            string cs = System.IO.File.ReadLines("DBstring.conf").First();
            DIOS.Common.SqlManager manager = new DIOS.Common.SqlManager(cs);
            DataFetcher fetcher = new DataFetcher(manager);

            {
                DateTime dt1 = new DateTime(2019, 2, 2, 7, 20, 0);
                DateTime dt2 = new DateTime(2019, 2, 2, 8, 20, 0);
                              
                Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);

                Assert.IsTrue(trajectory.Points[0].Time == dt1);
                Assert.IsTrue(trajectory.Points.Last().Time == dt2);

                Assert.IsTrue(trajectory.Points[0].Position == trajectory.GetPosition(dt1));
                Assert.IsTrue(trajectory.Points.Last().Position == trajectory.GetPosition(dt2));

                //for (int i = 0; i < trajectory.Count; i++)
                //{
                //    var dtime = trajectory.Points[i].Time;
                //    TrajectoryPoint? point = fetcher.GetSingleTragectoryPoint(dtime);
                //    Assert.IsTrue(point != null);
                //    var trpoint = (TrajectoryPoint)point; 
                //    Assert.IsTrue(trpoint.Time == dtime);
                //    double dist = (trpoint.Position - trajectory.Points[i].Position).Length;
                //    Assert.IsTrue(dist < TrajectoryEps);
                //}
            }


            /// Проверим обработку отсутствия траектории в базе данных
            bool fail = false;
            {
                DateTime dt1 = new DateTime(2000, 2, 2, 0, 0, 0);
                DateTime dt2 = new DateTime(2000, 2, 3, 0, 0, 0);
                //мы ожидаем, что на заданный диапазон нет траектории 

                try
                {
                    Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2); 
                    // если мы дошли сюда, значит исключение небыло сгенерировано, что неверно
                    fail = true;                     
                }
                catch (ArgumentException ex)
                { } // если исключение было сгенерировано, значит всё ок
            }
            {
                DateTime dt1 = new DateTime(2000, 2, 2, 0,0,0);
                DateTime dt2 = new DateTime(2000, 2, 2, 0, 0, 10);
                //мы ожидаем, что на заданный диапазон нет траектории 
                try
                {
                    Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);
                    // если мы дошли сюда, значит исключение небыло сгенерировано, что неверно
                    fail = true;
                   
                }
                catch (ArgumentException ex)
                { } // если исключение было сгенерировано, значит всё ок
            }
            {
                DateTime dt1 = new DateTime(2000, 2, 2, 0, 0, 0);
                DateTime dt2 = new DateTime(2014, 7, 13, 0, 20, 10);
                //мы ожидаем, что только на часть заданного диапазона имеется траектория в БД, должно быть сгенерировано исключение
                try
                {
                    Trajectory trajectory = fetcher.GetTrajectorySat(dt1, dt2);
                    // если мы дошли сюда, значит исключение небыло сгенерировано, что неверно
                    fail = true;
                }
                catch (ArgumentException ex)
                { } // если исключение было сгенерировано, значит всё ок
            }
            Assert.IsFalse(fail);

        }




    }
}
