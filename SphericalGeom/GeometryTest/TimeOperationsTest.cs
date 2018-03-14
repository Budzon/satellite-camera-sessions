using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Collections.Generic;
using System.Linq;

using SatelliteSessions;

namespace GeometryTest
{
    [TestClass]
    public class TimeOperationsTest
    {
        [TestMethod]
        public void Test_invertTimeSpans()
        {
            /// public static List<Tuple<DateTime, DateTime>> invertTimeSpans(List<Tuple<DateTime, DateTime>> inputSpans, DateTime timeFrom, DateTime timeTo)
          
            DateTime timeFrom = new DateTime(2000, 01, 01);
            DateTime timeTo = new DateTime(2000, 02, 01);
            
            List<Tuple<DateTime, DateTime>> inputSpan = new List<Tuple<DateTime, DateTime>>();

            inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 01), new DateTime(2000, 01, 05)));
            inputSpan.Add(new Tuple<DateTime, DateTime>(new DateTime(2000, 01, 10), new DateTime(2000, 01, 20)));
            
            List<Tuple<DateTime, DateTime>> res = Sessions.invertTimeSpans(inputSpan, timeFrom, timeTo);

            Assert.IsTrue(res.Count == 2);

            Assert.IsTrue(res[0].Item1.Month == 1);
            Assert.IsTrue(res[0].Item1.Day == 5);

            Assert.IsTrue(res[0].Item2.Month == 1);
            Assert.IsTrue(res[0].Item2.Day == 10);


            Assert.IsTrue(res[1].Item1.Month == 1);
            Assert.IsTrue(res[1].Item1.Day == 20);

            Assert.IsTrue(res[1].Item2.Month == 2);
            Assert.IsTrue(res[1].Item2.Day == 1);                       
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

            List<Tuple<DateTime, DateTime>> res = Sessions.compressTimeSpans(inputSpan);

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
