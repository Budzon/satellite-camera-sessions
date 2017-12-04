using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Astronomy
{
    public class LinearInterpolation
    {
        private double[] x;
        private double[] y;

        private double minX;
        private double maxX;

        public double MinX
        {
            get { return this.minX; }
        }

        public double MaxX
        {
            get { return this.maxX; }
        }

        public LinearInterpolation(double[] x, double[] y)
        {
            if (x == null)
                throw new ArgumentNullException("y");
            if (y == null)
                throw new ArgumentNullException("y");
            if (x.Length != y.Length)
                throw new ArgumentException("Length of x and y arrays must be equal");
            if (x.Length == 0)
                throw new ArgumentException("Arrays must have at least one point");
            this.x = x;
            this.y = y;

            this.minX = x[0];
            this.maxX = x[x.Length - 1];
        }

        public double GetValue(double xVal)
        {
            if (xVal < this.minX)
            {
                return this.y[0];
            }
            if (xVal > this.maxX)
            {
                return this.y[this.y.Length - 1];
            }

            int indAppr = Array.BinarySearch(this.x, xVal);

            if (indAppr < 0)
                indAppr = ~indAppr;

            if (indAppr > x.Length - 1)
                indAppr = x.Length - 1;
            if (indAppr < 0)
                indAppr = 0;

            int leftInd = 0;
            int rightInd = 0;

            double indApprTime = this.x[indAppr];

            if (indApprTime == xVal)
            {
                return y[indAppr];
            }
            else if (indApprTime < xVal)
            {
                if (indAppr == x.Length - 1)
                    return y[indAppr];
                else
                {
                    leftInd = indAppr;
                    rightInd = indAppr + 1;
                }
            }
            else if (indApprTime > xVal)
            {
                if (indAppr == 0)
                    return this.y[indAppr];
                else
                {
                    leftInd = indAppr - 1;
                    rightInd = indAppr;
                }
            }
            double leftY = this.y[leftInd];
            double rightY = this.y[rightInd];

            return Interpolate(xVal, leftInd, rightInd, leftY, rightY);
        }

        private static double Interpolate(double value, double x0, double x1, double y0, double y1)
        {
            return y0 + (value - x0) * (y1 - y0) / (x1 - x0);
        }
    }
}
