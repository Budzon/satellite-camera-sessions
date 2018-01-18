using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Windows.Media.Media3D;
using SphericalGeom;
using Common;

namespace GeometryTest
{
    [TestClass]
    public class ReferenceFrameTest
    {
        [TestMethod]
        public void TestToBaseFrame()
        {
            ReferenceFrame frame = new ReferenceFrame(new Vector3D(1, 0, 0));
            Vector3D target = new Vector3D(2, -1, 3);
            Vector3D actual = frame.ToBaseFrame(1, 2, 3);

            Assert.IsTrue(Comparison.IsZero(target - actual));
        }

        [TestMethod]
        public void TestToThisFrame()
        {
            ReferenceFrame frame = new ReferenceFrame(new Vector3D(1, 0, 0));
            Vector3D target = new Vector3D(1, 2, 3);
            Vector3D actual = frame.ToThisFrame(2, -1, 3);

            Assert.IsTrue(Comparison.IsZero(target - actual));
        }

        [TestMethod]
        public void TestRotateBy()
        {
            ReferenceFrame frame = new ReferenceFrame();
            frame.RotateBy(new Vector3D(1, 0, 0), 90);
            Vector3D target = new Vector3D(0, -1, 0);
            Vector3D actual = frame.ToThisFrame(0, 0, 1);

            Assert.IsTrue(Comparison.IsZero(target - actual));
        }

        [TestMethod]
        public void TestConcatenate()
        {
            ReferenceFrame frame1 = new ReferenceFrame();
            frame1.RotateBy(new Vector3D(1, 0, 0), 90);
            ReferenceFrame frame2 = new ReferenceFrame();
            frame2.RotateBy(new Vector3D(0, 1, 0), 90);
            ReferenceFrame frame = ReferenceFrame.Concatenate(frame1, frame2);
            Vector3D target = new Vector3D(0, 0, -1);
            Vector3D actual = frame.ToThisFrame(1, 0, 0);

            Assert.IsTrue(Comparison.IsZero(target - actual));
        }
    }
}
