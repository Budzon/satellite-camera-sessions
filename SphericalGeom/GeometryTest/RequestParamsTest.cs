using System;
using System.Text;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;

using OptimalChain;
using SessionsPlanning;

namespace GeometryTest
{
    /// <summary>
    /// Summary description for UnitTest1
    /// </summary>
    [TestClass]
    public class RequestParamsTest
    {
        [TestMethod]
        public void Test_breakRequestsIntoGroups()
        {
            {
                string s = "POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))";
                List<RequestParams> list = new List<RequestParams>()
             {
                 new RequestParams(1, 1, new DateTime(),new DateTime(), 1, 1,1,1,s, _shootingType : ShootingType.StereoTriplet, _compression : 10, _requestChannel : ShootingChannel.cm),
                 new RequestParams(2, 1, new DateTime(),new DateTime(), 1, 1,1,1,s, _shootingType : ShootingType.StereoTriplet, _compression : 10, _requestChannel : ShootingChannel.cm),
                 
                 new RequestParams(3, 1, new DateTime(),new DateTime(), 1, 1,1,1,s, _shootingType : ShootingType.StereoTriplet, _compression : 1, _requestChannel :  ShootingChannel.cm),                 

                 new RequestParams(4, 1, new DateTime(),new DateTime(), 1, 1,1,1,s, _shootingType : ShootingType.StereoTriplet, _compression : 10, _requestChannel :  ShootingChannel.pk),
                 new RequestParams(5, 1, new DateTime(),new DateTime(), 2, 2,2,2,s, _shootingType : ShootingType.StereoTriplet, _compression : 10, _requestChannel : ShootingChannel.pk),

                 new RequestParams(6, 1, new DateTime(),new DateTime(), 1, 1,1,1,s, _shootingType : ShootingType.StereoTriplet, _compression : 1, _requestChannel : ShootingChannel.pk),

                 new RequestParams(7, 1, new DateTime(),new DateTime(), 1, 1,1,1,s, _shootingType : ShootingType.Coridor, _compression : 1, _requestChannel : ShootingChannel.pk)
             };

                List<List<RequestParams>> res = RequestParams.breakRequestsIntoGroups(list);

                Assert.IsTrue(res.Count == 5);

                Assert.IsTrue(res[0].Count == 2);
                Assert.IsTrue(res[1].Count == 1);
                Assert.IsTrue(res[2].Count == 2);
                Assert.IsTrue(res[3].Count == 1);
                Assert.IsTrue(res[4].Count == 1);

                Assert.IsTrue(res[0][0].id == 1);
                Assert.IsTrue(res[0][1].id == 2);

                Assert.IsTrue(res[1][0].id == 3);

                Assert.IsTrue(res[2][0].id == 4);
                Assert.IsTrue(res[2][1].id == 5);

                Assert.IsTrue(res[3][0].id == 6);

                Assert.IsTrue(res[4][0].id == 7);
            }

            {
                string s = "POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))";
                List<RequestParams> list = new List<RequestParams>();

                List<List<RequestParams>> res = RequestParams.breakRequestsIntoGroups(list);

                Assert.IsTrue(res.Count == 0);
            }

            {
                string s = "POLYGON ((2 -2, 2 2, -2 2, -2 -2, 2 -2))";
                List<RequestParams> list = new List<RequestParams>()
             {
                 new RequestParams(1, 1, new DateTime(),new DateTime(), 1, 1,1,1,s, _shootingType : ShootingType.StereoTriplet, _compression : 10, _requestChannel : ShootingChannel.cm),
                     
             };

                List<List<RequestParams>> res = RequestParams.breakRequestsIntoGroups(list);

                Assert.IsTrue(res.Count == 1);

                Assert.IsTrue(res[0].Count == 1);

                Assert.IsTrue(res[0][0].id == 1);

            }

        }
    }
}
