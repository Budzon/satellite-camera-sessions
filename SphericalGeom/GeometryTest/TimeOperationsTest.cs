using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Collections.Generic;
using System.Linq;

using SatelliteSessions;
using OptimalChain;

namespace GeometryTest
{
    [TestClass]
    public class TimeOperationsTest
    {

        bool isEqual(Tuple<DateTime, DateTime> span1, Tuple<DateTime, DateTime> span2)
        {
            if (span1.Item1 != span2.Item1)
                return false;
            if (span1.Item2 != span2.Item2)
                return false;
            return true;
        }

        [TestMethod]
        public void Test_invertTimeSpans()
        {
            {
                DateTime timeFrom = new DateTime(2000, 1, 4);
                DateTime timeTo = new DateTime(2000, 1, 9);

                List<Tuple<DateTime, DateTime>> inputSpan = new List<Tuple<DateTime, DateTime>>();

                inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 01), new DateTime(2000, 01, 02)));
                inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 3), new DateTime(2000, 01, 5)));
                inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 6), new DateTime(2000, 01, 7)));
                inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 10), new DateTime(2000, 01, 12)));

                List<Tuple<DateTime, DateTime>> res = Sessions.getFreeIntervals(inputSpan, timeFrom, timeTo);

                Assert.IsTrue(res.Count == 2);

                Assert.IsTrue(isEqual(res[0], Tuple.Create(new DateTime(2000, 01, 5), new DateTime(2000, 01, 6))));
                Assert.IsTrue(isEqual(res[1], Tuple.Create(new DateTime(2000, 01, 7), new DateTime(2000, 01, 9))));
            }

            {
                DateTime timeFrom = new DateTime(2000, 1, 4);
                DateTime timeTo   = new DateTime(2000, 1, 5);

                List<Tuple<DateTime, DateTime>> inputSpan = new List<Tuple<DateTime, DateTime>>();

                inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 01), new DateTime(2000, 01, 02)));
                inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 9), new DateTime(2000, 01, 15)));

                List<Tuple<DateTime, DateTime>> res = Sessions.getFreeIntervals(inputSpan, timeFrom, timeTo);

                Assert.IsTrue(res.Count == 1);

                Assert.IsTrue(isEqual(res[0], Tuple.Create(new DateTime(2000, 01, 4), new DateTime(2000, 01, 5)))); 
            }

            {
                DateTime timeFrom = new DateTime(2000, 1, 4);
                DateTime timeTo = new DateTime(2000, 1, 5);

                List<Tuple<DateTime, DateTime>> inputSpan = new List<Tuple<DateTime, DateTime>>();

                inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 01), new DateTime(2000, 01, 06)));
                inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 9), new DateTime(2000, 01, 15)));

                List<Tuple<DateTime, DateTime>> res = Sessions.getFreeIntervals(inputSpan, timeFrom, timeTo);

                Assert.IsTrue(res.Count == 0);
            }

            {
                DateTime timeFrom = new DateTime(2000, 1, 4);
                DateTime timeTo = new DateTime(2000, 1, 6);

                List<Tuple<DateTime, DateTime>> inputSpan = new List<Tuple<DateTime, DateTime>>();

                inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));
                inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 5), new DateTime(2000, 01, 15)));

                List<Tuple<DateTime, DateTime>> res = Sessions.getFreeIntervals(inputSpan, timeFrom, timeTo);

                Assert.IsTrue(res.Count == 0);
            }

        }


        [TestMethod]
        public void Test_compressTimeSpans()
        {

            List<Tuple<DateTime, DateTime>> inputSpan = new List<Tuple<DateTime, DateTime>>();

            inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));
            inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));
            inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));
            inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));
            inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));
            inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));
            inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 10), new DateTime(2000, 01, 20)));
            inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 2), new DateTime(2000, 01, 15)));
            inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 22), new DateTime(2000, 01, 24)));
            inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 23), new DateTime(2000, 01, 25))); 
            inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 23), new DateTime(2000, 01, 25)));

            List<Tuple<DateTime, DateTime>> res = Sessions.compressTimePeriods(inputSpan);

            Assert.IsTrue(res.Count == 2);
             
            Assert.IsTrue(res[0].Item1.Month == 1);
            Assert.IsTrue(res[0].Item1.Day == 1);
             
            Assert.IsTrue(res[0].Item2.Month == 1);
            Assert.IsTrue(res[0].Item2.Day == 20);
                         
            Assert.IsTrue(res[1].Item1.Month == 1);
            Assert.IsTrue(res[1].Item1.Day == 22);
             
            Assert.IsTrue(res[1].Item2.Month == 1);
            Assert.IsTrue(res[1].Item2.Day == 25);   
        }
 
        


    }
}
