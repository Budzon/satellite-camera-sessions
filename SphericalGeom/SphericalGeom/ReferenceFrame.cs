using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

using Astronomy;

namespace SphericalGeom
{
    public class ReferenceFrame
    {
        private Matrix3D basis; 
        private Matrix3D basisT;

        public ReferenceFrame(Vector3D ox, Vector3D oy, Vector3D oz)
        {
            basisT = AstronomyMath.CS2toCS1(ox / ox.Length, oy / oy.Length, oz / oz.Length);
            basis = Transpose(basisT);
        }
        public ReferenceFrame(Vector3D oy) 
            : this(new Vector3D(oy.Y, -oy.X, 0),
                   oy,
                   new Vector3D(-oy.X * oy.Z, -oy.Y * oy.Z, oy.X * oy.X + oy.Y * oy.Y)) { }
        public ReferenceFrame()
            : this(new Vector3D(1, 0, 0), new Vector3D(0, 1, 0), new Vector3D(0, 0, 1)) { }

        public Vector3D ToThisFrame(Vector3D v)
        {
            return v * basis;
        }
        public Vector3D ToThisFrame(double x, double y, double z)
        {
            return ToThisFrame(new Vector3D(x, y, z));
        }
        public Vector3D ToBaseFrame(Vector3D v)
        {
            return v * basisT;
        }
        public Vector3D ToBaseFrame(double x, double y, double z)
        {
            return ToBaseFrame(new Vector3D(x, y, z));
        }

        public void RotateBy(Quaternion q)
        {
            basis.Rotate(q);
            basisT = Transpose(basis);
        }
        public void RotateBy(Vector3D axis, double angle)
        {
            RotateBy(new Quaternion(axis, angle));
        }
        public void RotateBy(double angle)
        {
            RotateBy(new Vector3D(basis.M12, basis.M22, basis.M32), angle);
        }
        private Matrix3D Transpose(Matrix3D m)
        {
            return new Matrix3D(m.M11, m.M21, m.M31, m.OffsetX,
                                m.M12, m.M22, m.M32, m.OffsetY,
                                m.M13, m.M23, m.M33, m.OffsetZ,
                                m.M14, m.M24, m.M34, m.M44);
        }
    }
}
