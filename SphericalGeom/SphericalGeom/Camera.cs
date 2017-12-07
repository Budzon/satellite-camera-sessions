using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

//using Astronomy;
using Common;

namespace SphericalGeom
{
    public class Camera
    {
        //private static double sightAngleStep = 1e-1;
        //private static double sightDirectionStep = 5e-3;
        
        private Vector3D position;
        private GeoPoint positionDirection;
        // Right orthonormal basis with -position as OY such that OZY contains the North pole
        private ReferenceFrame camPositionFrame;

        public Vector3D Position
        {
            get { return position; }
            set
            {
                position = value;
                positionDirection = GeoPoint.FromCartesian(position);
                camPositionFrame = new ReferenceFrame(-value);
            }
        }
        public GeoPoint PositionDirection { get { return positionDirection; } }
        public double VerticalHalfAngleOfView { get; set; }
        public double HorizontalHalfAngleOfView { get; set; }
        public IList<Vector3D> InnerNormalsToCone
        {
            get { return SightPyramidInnerNormals(camPositionFrame); }
        }

        public Camera(Vector3D position, double verticalHalfAngleOfView, double horizontalHalfAngleOfView)
        {
            Position = position;
            VerticalHalfAngleOfView = verticalHalfAngleOfView;
            HorizontalHalfAngleOfView = horizontalHalfAngleOfView;
        }
        public Camera(GeoPoint direction, double altitude, double verticalHalfAngleOfView, double horizontalHalfAngleOfView)
            : this(GeoPoint.ToCartesian(direction, 1 + altitude), verticalHalfAngleOfView, horizontalHalfAngleOfView) { }
        public Camera() : this(new GeoPoint(), 0.2, Math.PI / 6, Math.PI / 6) { }

        private List<Vector3D> SightPyramidInnerNormals(ReferenceFrame camSightFrame)
        {
            double cv = Math.Cos(VerticalHalfAngleOfView);
            double sv = Math.Sin(VerticalHalfAngleOfView);
            double ch = Math.Cos(HorizontalHalfAngleOfView);
            double sh = Math.Sin(HorizontalHalfAngleOfView);

            var res = new List<Vector3D> { 
                camSightFrame.ToBaseFrame(0, sv, cv), 
                camSightFrame.ToBaseFrame(ch, sh, 0), 
                camSightFrame.ToBaseFrame(0, sv, -cv), 
                camSightFrame.ToBaseFrame(-ch, sh, 0) 
            };
            //res.ForEach(normal => normal.Normalize());
            return res;
        }

        /// TODO: Give angles to capture region
        //private IEnumerable<ReferenceFrame> PossibleSightFrames()
        //{
        //    double yaw = 0, maxAngle = Math.Asin(1 / Position.Length);
        //    while (Comparison.IsSmaller(yaw, Math.PI))
        //    {

                
        //        yaw += sightAngleStep;
        //    }
        //}
    }
}
