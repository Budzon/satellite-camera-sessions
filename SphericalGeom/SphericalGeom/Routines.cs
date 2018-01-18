using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

using Common;

namespace SphericalGeom
{
    public static class Routines
    {
        public static List<Polygon> SliceIntoSquares(Polygon p, Vector3D latticeOrigin, double latticeAxisInclination, double squareSide)
        {
            if (latticeOrigin.X == 0 && latticeOrigin.Y == 0)
                throw new ArgumentException("latticeOrigin is north pole, try perturbing it a little bit");

            var latticeFrame = new ReferenceFrame(
                   latticeOrigin,
                   new Vector3D(-latticeOrigin.Y, latticeOrigin.X, 0),
                   new Vector3D(-latticeOrigin.X * latticeOrigin.Z,
                                -latticeOrigin.Y * latticeOrigin.Z,
                                latticeOrigin.X * latticeOrigin.X + latticeOrigin.Y * latticeOrigin.Y));
            latticeFrame.RotateBy(new Vector3D(1, 0, 0), -latticeAxisInclination);
            
            // Place p at lat=0, lon=0
            Vector3D pMiddle = latticeFrame.ToThisFrame(p.Middle);
            var pFrame = ReferenceFrame.Concatenate(latticeFrame,
                new ReferenceFrame(pMiddle,
                    new Vector3D(-pMiddle.Y, pMiddle.X, 0),
                    new Vector3D(-pMiddle.X * pMiddle.Z,
                        -pMiddle.Y * pMiddle.Z,
                        pMiddle.X * pMiddle.X + pMiddle.Y * pMiddle.Y)));
            p.ToThisFrame(pFrame);

            GeoRect boundingBox = p.BoundingBox();
            List<GeoRect> checkerboard = SliceIntoSquares(boundingBox, squareSide);
            List<Polygon> squares = new List<Polygon>();
            foreach (GeoRect square in checkerboard)
                squares.AddRange(Polygon.Intersect(p, new Polygon(square)));
            squares.ForEach(square => square.FromThisFrame(pFrame));
            p.FromThisFrame(pFrame);
            return squares;
        }

        /// <summary>
        /// Break the rectangle into squares of size <paramref name="squareSide"/>,
        /// assuming that it does not contain poles and does not cross the 180 meridian.
        /// </summary>
        /// <param name="rect"></param>
        /// <param name="squareSide">Square side in degrees.</param>
        /// <returns></returns>
        public static List<GeoRect> SliceIntoSquares(GeoRect rect, double squareSide)
        {
            List<GeoRect> checkerboard = new List<GeoRect>();

            bool cropHeight = Comparison.IsPositive(rect.HeightLatitude % squareSide);
            bool cropWidth = Comparison.IsPositive(rect.WidthLongitude % squareSide);

            int latSlices = (int)((rect.HeightLatitude - (rect.HeightLatitude % squareSide)) / squareSide);
            int lonSlices = (int)((rect.WidthLongitude - (rect.WidthLongitude % squareSide)) / squareSide);

            for (int i = 0; i < latSlices; ++i)
                for (int j = 0; j < lonSlices; ++j)
                    checkerboard.Add(new GeoRect(
                        rect.LeftLongitude + j * squareSide, rect.LeftLongitude + (j + 1) * squareSide,
                        rect.BottomLatitude + i * squareSide, rect.BottomLatitude + (i + 1) * squareSide));

            if (cropHeight)
                for (int j = 0; j < lonSlices; ++j)
                    checkerboard.Add(new GeoRect(
                        rect.LeftLongitude + j * squareSide, rect.LeftLongitude + (j + 1) * squareSide,
                        rect.BottomLatitude + latSlices * squareSide, rect.TopLatitude));
            if (cropWidth)
                for (int i = 0; i < latSlices; ++i)
                    checkerboard.Add(new GeoRect(
                        rect.LeftLongitude + lonSlices * squareSide, rect.RightLongitude,
                        rect.BottomLatitude + i * squareSide, rect.BottomLatitude + (i + 1) * squareSide));
            if (cropHeight && cropWidth)
                checkerboard.Add(new GeoRect(
                        rect.LeftLongitude + lonSlices * squareSide, rect.RightLongitude,
                        rect.BottomLatitude + latSlices * squareSide, rect.TopLatitude));

            return checkerboard;
        }

        //public static GeoPoint SafeShift(GeoPoint gp, double lat, double lon)
        //{
        //    GeoPoint outGp = gp;
        //    outGp.Latitude += lat;
        //    if (outGp.Latitude > 90.0)
        //    {
        //        outGp.Latitude = 180.0 - gp.Latitude;
        //        outGp.Longitude *= -1;
        //    }
        //    else if (outGp.Latitude < -90.0)
        //    {
        //        outGp.Latitude = -180 - gp.Latitude;
        //        outGp.Longitude *= -1;
        //    }

        //    outGp.Longitude += lon;
        //    while (Comparison.IsBigger(outGp.Longitude, 180.0))
        //        outGp.Longitude -= 360.0;
        //    while (Comparison.IsSmaller(outGp.Longitude, -180.0))
        //        outGp.Longitude += 360.0;
        //    return outGp;
        //}

        public static Polygon ProjectConeOntoSphere(Vector3D apex, IList<Vector3D> normals)
        {
            var coneEdgeDirections = new List<Vector3D>();
            coneEdgeDirections.Add(Vector3D.CrossProduct(normals[normals.Count - 1], normals[0]));
            for (int i = 1; i < normals.Count; ++i)
                coneEdgeDirections.Add(Vector3D.CrossProduct(normals[i-1], normals[i]));

            var result = new List<Vector3D>();
            foreach (var direction in coneEdgeDirections)
            {
                var intersection = IntersectLineUnitSphere(apex, direction);
                // Assume a good cone: all its edges intersect the unit sphere
                result.Add(intersection.Where(v => Comparison.IsPositive(Vector3D.DotProduct(apex, v) - 1)).ElementAt(0));
            }
            return new Polygon(result, apex);
        }

        /// <summary>
        /// Solve a 2 by 3 system of linear equations.
        /// </summary>
        /// <param name="a1">Matrix first row.</param>
        /// <param name="a2">Matrix second row.</param>
        /// <param name="b1">First right hand side.</param>
        /// <param name="b2">Second right hand side.</param>
        /// <returns></returns>
        public static Vector3D SolveSLE2x3(Vector3D a1, Vector3D a2, double b1, double b2)
        {
            double x, y, z;
            double detXY = a1.X * a2.Y - a1.Y * a2.X;
            double detXZ = a1.X * a2.Z - a1.Z * a2.X;
            double detYZ = a1.Y * a2.Z - a1.Z * a2.Y;

            if (!Comparison.IsZero(detXY))
            {
                z = 0;
                x = (b1 * a2.Y - b2 * a1.Y) / detXY;
                y = (b2 * a1.X - b1 * a2.X) / detXY;
            }
            else if (!Comparison.IsZero(detXZ))
            {
                y = 0;
                x = (b1 * a2.Z - b2 * a1.Z) / detXZ;
                z = (b2 * a1.X - b1 * a2.X) / detXZ;
            }
            else if (!Comparison.IsZero(detYZ))
            {
                x = 0;
                y = (b1 * a2.Z - b2 * a1.Z) / detYZ;
                z = (b2 * a1.Y - b1 * a2.Y) / detYZ;
            }
            else
                throw new System.ArgumentException("System is rank deficient.");

            return new Vector3D(x, y, z);
        }

        public static List<double> SolveQuadraticEquation(double a, double b, double c)
        {
            List<double> res = new List<double>();
            double D = b * b - 4 * a * c;
            if (Comparison.IsPositive(D))
            {
                double sqrt = Math.Sqrt(D);
                res.Add((-b - sqrt) / (2 * a));
                res.Add((-b + sqrt) / (2 * a));
            }
            else if (Comparison.IsZero(D))
            {
                res.Add(-b / (2 * a));
            }
            return res;
        }

        public static IEnumerable<Vector3D> IntersectLineUnitSphere(Vector3D a, Vector3D dir)
        {
            List<double> parameters = SolveQuadraticEquation(dir.LengthSquared, 
                                                             2 * Vector3D.DotProduct(a, dir), 
                                                             a.LengthSquared - 1);
            return parameters.Select(t => a + t * dir);
        }

        public delegate double FuncDoubleToDouble(double x);
        public static double GoldenRatio = 1.61803398875;
        public static double FindMax(FuncDoubleToDouble f, double left, double right)
        {
            double maxf = f(left);
            double dx = 1e-2, x = left + dx;
            while (x < right)
            {
                maxf = Math.Max(maxf, f(x));
                x += dx;
            }
            return maxf;
            //double midLeft = right - (right - left) / GoldenRatio;
            //double midRight = left + (right - left) / GoldenRatio;
            //double fLeft = f(left), fMidLeft = f(midLeft), fMidRight = f(midRight), fRight = f(right);
            //while (Math.Abs(left - right) > 1e-3 * (Math.Abs(midLeft) + Math.Abs(midRight)) / 2)
            //{
            //    if (fMidLeft < fMidRight)
            //    {
            //        left = midLeft;
            //        fLeft = fMidLeft;

            //        midLeft = midRight;
            //        fMidLeft = fMidRight;

            //        midRight = left + (right - left) / GoldenRatio;
            //        fMidRight = f(midRight);
            //    }
            //    else
            //    {
            //        right = midRight;
            //        fRight = fMidRight;

            //        midRight = midLeft;
            //        fMidRight = fMidLeft;

            //        midLeft = right - (right - left) / GoldenRatio;
            //        fMidLeft = f(midLeft);
            //    }
            //}
            //return f((left + right) / 2);
        }
        public static double FindMin(FuncDoubleToDouble f, double left, double right)
        {
            double minf = f(left);
            double dx = 1e-2, x = left + dx;
            while (x < right)
            {
                minf = Math.Min(minf, f(x));
                x += dx;
            }
            return minf;
            //double midLeft = right - (right - left) / GoldenRatio;
            //double midRight = left + (right - left) / GoldenRatio;
            //double fLeft = f(left), fMidLeft = f(midLeft), fMidRight = f(midRight), fRight = f(right);
            //while (Math.Abs(left - right) > 1e-2 * (Math.Abs(midLeft) + Math.Abs(midRight)) / 2)
            //{
            //    if (fMidLeft > fMidRight)
            //    {
            //        left = midLeft;
            //        fLeft = fMidLeft;

            //        midLeft = midRight;
            //        fMidLeft = fMidRight;

            //        midRight = left + (right - left) / GoldenRatio;
            //        fMidRight = f(midRight);
            //    }
            //    else
            //    {
            //        right = midRight;
            //        fRight = fMidRight;

            //        midRight = midLeft;
            //        fMidRight = fMidLeft;

            //        midLeft = right - (right - left) / GoldenRatio;
            //        fMidLeft = f(midLeft);
            //    }
            //}
            //return f((left + right) / 2);
        }
    }
}
