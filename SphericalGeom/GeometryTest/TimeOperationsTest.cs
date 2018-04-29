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

        bool isEqual(TimePeriod span1, TimePeriod span2)
        {
            if (span1.dateFrom != span2.dateFrom)
                return false;
            if (span1.dateTo != span2.dateTo)
                return false;
            return true;
        }

        [TestMethod]
        public void Test_invertTimeSpans()
        {
            {
                DateTime timeFrom = new DateTime(2000, 1, 4);
                DateTime timeTo = new DateTime(2000, 1, 9);

                List<TimePeriod> inputSpan = new List<TimePeriod>();

                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 01), new DateTime(2000, 01, 02)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 3), new DateTime(2000, 01, 5)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 6), new DateTime(2000, 01, 7)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 10), new DateTime(2000, 01, 12)));

                List<TimePeriod> res = TimePeriod.getFreeIntervals(inputSpan, timeFrom, timeTo);

                Assert.IsTrue(res.Count == 2);

                Assert.IsTrue(isEqual(res[0], new TimePeriod(new DateTime(2000, 01, 5), new DateTime(2000, 01, 6))));
                Assert.IsTrue(isEqual(res[1], new TimePeriod(new DateTime(2000, 01, 7), new DateTime(2000, 01, 9))));
            }

            {
                DateTime timeFrom = new DateTime(2000, 1, 4);
                DateTime timeTo = new DateTime(2000, 1, 5);

                List<TimePeriod> inputSpan = new List<TimePeriod>();

                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 01), new DateTime(2000, 01, 02)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 9), new DateTime(2000, 01, 15)));

                List<TimePeriod> res = TimePeriod.getFreeIntervals(inputSpan, timeFrom, timeTo);

                Assert.IsTrue(res.Count == 1);

                Assert.IsTrue(isEqual(res[0], new TimePeriod(new DateTime(2000, 01, 4), new DateTime(2000, 01, 5))));
            }

            {
                DateTime timeFrom = new DateTime(2000, 1, 4);
                DateTime timeTo = new DateTime(2000, 1, 5);

                List<TimePeriod> inputSpan = new List<TimePeriod>();

                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 01), new DateTime(2000, 01, 06)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 9), new DateTime(2000, 01, 15)));

                List<TimePeriod> res = TimePeriod.getFreeIntervals(inputSpan, timeFrom, timeTo);

                Assert.IsTrue(res.Count == 0);
            }

            {
                DateTime timeFrom = new DateTime(2000, 1, 4);
                DateTime timeTo = new DateTime(2000, 1, 6);

                List<TimePeriod> inputSpan = new List<TimePeriod>();

                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 5), new DateTime(2000, 01, 15)));

                List<TimePeriod> res = TimePeriod.getFreeIntervals(inputSpan, timeFrom, timeTo);

                Assert.IsTrue(res.Count == 0);
            }

        }




        [TestMethod]
        public void Test_compressTimeSpans()
        {
            {
                List<TimePeriod> inputSpan = new List<TimePeriod>();

                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 10), new DateTime(2000, 01, 20)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 2), new DateTime(2000, 01, 15)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 22), new DateTime(2000, 01, 24)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 23), new DateTime(2000, 01, 25)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 23), new DateTime(2000, 01, 25)));

                List<TimePeriod> res = TimePeriod.compressTimePeriods(inputSpan);

                Assert.IsTrue(res.Count == 2);


                Assert.IsTrue(res[0].Equals(new TimePeriod(new DateTime(2000, 01, 01), new DateTime(2000, 01, 20))));
                Assert.IsTrue(res[1].Equals(new TimePeriod(new DateTime(2000, 01, 22), new DateTime(2000, 01, 25))));
            }
            
            {
                List<TimePeriod> inputSpan = new List<TimePeriod>();

                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));

                List<TimePeriod> res = TimePeriod.compressTimePeriods(inputSpan);

                Assert.IsTrue(res.Count == 1);

                Assert.IsTrue(res[0].Equals( new TimePeriod(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)))); 
            } 

            {
                List<TimePeriod> inputSpan = new List<TimePeriod>();

                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 01,0,0,0), new DateTime(2000, 01, 05,0,0,0)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 05, 0, 0, 10), new DateTime(2000, 01, 16, 0, 0, 0)));

                List<TimePeriod> res = TimePeriod.compressTimePeriods(inputSpan, 0);

                Assert.IsTrue(res.Count == 2);

                Assert.IsTrue(res[0].Equals(new TimePeriod(new DateTime(2000, 01, 01, 0, 0, 0), new DateTime(2000, 01, 05, 0, 0, 0))));
                Assert.IsTrue(res[1].Equals(new TimePeriod(new DateTime(2000, 01, 05, 0, 0, 10), new DateTime(2000, 01, 16, 0, 0, 0))));
            }



            {
                List<TimePeriod> inputSpan = new List<TimePeriod>();

                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 01, 0, 0, 0), new DateTime(2000, 01, 05, 0, 0, 0)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 05, 0, 0, 10), new DateTime(2000, 01, 16, 0, 0, 0)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 08, 0, 0, 0), new DateTime(2000, 01, 12, 0, 0, 0)));

                List<TimePeriod> res = TimePeriod.compressTimePeriods(inputSpan, 20);

                Assert.IsTrue(res.Count == 1);

                Assert.IsTrue(res[0].Equals(new TimePeriod(new DateTime(2000, 01, 01, 0, 0, 0), new DateTime(2000, 01, 16, 0, 0, 0)))); 
            }


            {
                List<TimePeriod> inputSpan = new List<TimePeriod>();

                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 01, 0, 0, 0), new DateTime(2000, 01, 05, 0, 0, 0)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 05, 0, 0, 10), new DateTime(2000, 01, 16, 0, 0, 0)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 08, 0, 0, 0), new DateTime(2000, 01, 12, 0, 0, 0)));

                List<TimePeriod> res = TimePeriod.compressTimePeriods(inputSpan, 9);

                Assert.IsTrue(res.Count == 2);

                Assert.IsTrue(res[0].Equals(new TimePeriod(new DateTime(2000, 01, 01, 0, 0, 0), new DateTime(2000, 01, 5, 0, 0, 0))));
                Assert.IsTrue(res[1].Equals(new TimePeriod(new DateTime(2000, 01, 05, 0, 0, 10), new DateTime(2000, 01, 16, 0, 0, 0))));
            }

            
            {
                List<TimePeriod> inputSpan = new List<TimePeriod>();

                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 01, 0, 0, 0), new DateTime(2000, 01, 05, 0, 0, 0)));
                inputSpan.Add(new TimePeriod(new DateTime(2000, 01, 05, 0, 0, 10), new DateTime(2000, 01, 16, 0, 0, 0)));

                List<TimePeriod> res = TimePeriod.compressTimePeriods(inputSpan, 10);

                Assert.IsTrue(res.Count == 1);

                Assert.IsTrue(res[0].Equals(new TimePeriod(new DateTime(2000, 01, 01, 0, 0, 0), new DateTime(2000, 01, 16, 0, 0, 0))));
            }

        }
    }
}
