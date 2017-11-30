using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Astronomy
{
    public class BSpline
    {
        private double[] t;
        private double[] x;
        private double[] y;
        private double[] z;
        private int length;
        private double min;
        private double max;
        private BSplineCoeffs coeffs;

        public BSpline(double[] t, double[] x, double[] y, double[] z)
        {
            if (t == null)
                throw new ArgumentNullException("t");
            if (x == null)
                throw new ArgumentNullException("x");
            if (y == null)
                throw new ArgumentNullException("y");
            if (z == null)
                throw new ArgumentNullException("z");
            if (t.Length != x.Length && x.Length != y.Length && y.Length != z.Length)
                throw new ArgumentException("arrays length differs");
            this.t = t;
            this.x = x;
            this.y = y;
            this.z = z;
            this.length = t.Length;
            this.min = this.t.Min();
            this.max = this.t.Max();

            double[] knots = new double[length + 4];
            int i = 0;
            for (; i < 4; i++)
                knots[i] = 0;
            for (; i < length; i++)
                knots[i] = 1.0 / (length - 3) * (i - 3);
            for (; i < knots.Length; i++)
                knots[i] = 1;
            coeffs = new BSplineCoeffs(length, 3, knots);
        }

        //private int GetAlpha(double t)
        //{
        //    double t0 = 1.0 / (max - min) * (t - min);

        //    int i = 3, j = length - 4;
        //    while (i + 1 < j)
        //    {
        //        int k = i + (j - i) / 2;
        //        if (t0 <= this.coeffs.Knots[k])
        //            j = k;
        //        else
        //            i = k;
        //    }
        //    return j;
        //}

        public double GetX(double t)
        {
            double t0 = 1.0 / (max - min) * (t - min);
            int interval = (int)Math.Floor(t0 / (1.0 / (length - 3)));
            if (interval == length - 3)
                interval--;
            double x = 0;
            
            for (int i = 0; i < length; i++)
            {
                x += this.x[i] * coeffs.GetBasisValue(interval, i, t);
            }
            return x;
        }

        public double GetY(double t)
        {
            double t0 = 1.0 / (max - min) * (t - min);
            int interval = (int)Math.Floor(t0 / (1.0 / (length - 3)));
            if (interval == length - 3)
                interval--;
            double y = 0;
            
            for (int i = 0; i < length; i++)
            {
                y += this.y[i] * coeffs.GetBasisValue(interval, i, t);
            }
            return y;
        }

        public double GetZ(double t)
        {
            double t0 = 1.0 / (max - min) * (t - min);
            int interval = (int)Math.Floor(t0 / (1.0 / (length - 3)));
            if (interval == length - 3)
                interval--;
            double z = 0;
            
            for (int i = 0; i < length; i++)
            {
                z += this.z[i] * coeffs.GetBasisValue(interval, i, t);
            }
            return z;
        }
    }

    public class BSplineCoeffs
    {

        protected int numPoints;
        protected int degree;
        protected double[] knots;
        protected double[, ,] coeffs;

        public BSplineCoeffs(int numPoints, int degree, double[] knots)
        {
            this.numPoints = numPoints;
            this.degree = degree;
            if (knots.Length != numPoints + degree + 1)
                throw new ArgumentException("Wrong size of knots vector", "kn");
            this.knots = (double[])knots.Clone();
            ComputeCoefficients();
        }

        protected void ComputeCoefficients()
        {
            coeffs = new double[knots.Length - 2 * degree - 1, degree + 1, degree + 1];
            for (int i = 0; i < knots.Length - 2 * degree - 1; i++)
                for (int j = 0; j <= degree; j++)
                    for (int k = 0; k <= degree; k++)
                        coeffs[i, j, k] = ComputeCoefficient(i + degree, degree, i + j, k);
        }

        protected double ComputeCoefficient(int interval, int n, int i, int k)
        {
            if (n == 0)
                return (interval == i) ? (1.0) : (0);
            else
            {
                double result = 0;
                double d1 = knots[i + n + 1] - knots[i + 1];
                double d0 = knots[i + n] - knots[i];
                if (k == 0)
                {
                    if (d1 > 0)
                        result += ComputeCoefficient(interval, n - 1, i + 1, 0) * knots[i + n + 1] / d1;
                    if (d0 > 0)
                        result -= ComputeCoefficient(interval, n - 1, i, 0) * knots[i] / d0;
                }
                else if (k == n)
                {
                    if (d0 > 0)
                        result += ComputeCoefficient(interval, n - 1, i, n - 1) / d0;
                    if (d1 > 0)
                        result -= ComputeCoefficient(interval, n - 1, i + 1, n - 1) / d1;
                }
                else
                {
                    if (d0 > 0)
                        result += (ComputeCoefficient(interval, n - 1, i, k - 1) -
                            knots[i] * ComputeCoefficient(interval, n - 1, i, k)) / d0;
                    if (d1 > 0)
                        result -= (ComputeCoefficient(interval, n - 1, i + 1, k - 1) -
                            knots[i + n + 1] * ComputeCoefficient(interval, n - 1, i + 1, k)) / d1;
                }
                return result;
            }
        }

        public int Order
        {
            get
            {
                return degree + 1;
            }
        }

        public int Degree
        {
            get
            {
                return degree;
            }
        }

        public int PointCount
        {
            get
            {
                return numPoints;
            }
        }

        public double[] Knots
        {
            get
            {
                return knots;
            }
        }

        public double GetCoeff(int interval, int i, int k)
        {
            if (i < interval || i > interval + degree)
                return 0;
            return coeffs[interval, i - interval, k];
        }

        public double GetBasisValue(int interval, int i, double t)
        {
            if (i < interval || i > interval + degree)
                return 0;
            double result = coeffs[interval, i - interval, degree];
            for (int k = degree - 1; k >= 0; k--)
                result = t * result + coeffs[interval, i - interval, k];
            return result;
        }

        public double GetDerivativeValue(int interval, int i, double t)
        {
            if (i < interval || i > interval + degree)
                return 0;
            double result = degree * coeffs[interval, i - interval, degree];
            for (int k = degree - 1; k >= 1; k--)
                result = t * result + k * coeffs[interval, i - interval, k];
            return result;
        }

        public double ParameterMin
        {
            get
            {
                return knots[degree];
            }
        }

        public double ParameterMax
        {
            get
            {
                return knots[numPoints];
            }
        }
    }
}
