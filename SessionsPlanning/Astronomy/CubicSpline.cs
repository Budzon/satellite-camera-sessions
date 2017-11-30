using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Astronomy
{
    /// <summary>
    /// Represents class for cubic spline interpolation
    /// </summary>
    public class CubicSpline
    {
        SplineTuple[] splines;

        /// <summary>
        /// Ordered argument values
        /// </summary>
        private double[] x;
        /// <summary>
        /// Function values
        /// </summary>
        private double[] y;

        private int length;

        public double MinArg
        {
            get { return x[0]; }
        }

        public double MaxArg
        {
            get { return x[length - 1]; }
        }

        private double minValue;
        private double maxValue;

        /// <summary>
        /// Creates new instance of <see cref="CubicSpline"/> class
        /// </summary>
        /// <param name="x">Ordered argument values</param>
        /// <param name="y">Function values</param>
        public CubicSpline(double[] x, double[] y)
        {
            if (x == null)
                throw new ArgumentNullException("x");
            if (y == null)
                throw new ArgumentNullException("y");
            if (x.Length != y.Length)
                throw new ArgumentException("x and y arrays length differs");
            this.x = x;
            this.y = y;

            this.minValue = this.y.Min();
            this.maxValue = this.y.Max();

            this.length = x.Length;

            this.splines = null;
        }

        private void BuildSpline()
        {
            int n = this.length;
            splines = new SplineTuple[n];
            for (int i = 0; i < n; ++i)
            {
                splines[i].x = x[i];
                splines[i].a = y[i];
            }
            splines[0].c = splines[n - 1].c = 0.0;

            double[] alpha = new double[n - 1];
            double[] beta = new double[n - 1];
            alpha[0] = beta[0] = 0.0;
            for (int i = 1; i < n - 1; ++i)
            {
                if (y[i] != y[i - 1])
                {
                    double h_i = x[i] - x[i - 1], h_i1 = x[i + 1] - x[i];
                    double A = h_i;
                    double C = 2.0 * (h_i + h_i1);
                    double B = h_i1;
                    double F = 6.0 * ((y[i + 1] - y[i]) / h_i1 - (y[i] - y[i - 1]) / h_i);
                    double z = (A * alpha[i - 1] + C);
                    alpha[i] = -B / z;
                    beta[i] = (F - A * beta[i - 1]) / z;
                }
                else
                {
                    alpha[i] = 0;
                    beta[i] = 0;
                }
            }

            for (int i = n - 2; i > 0; --i)
                splines[i].c = alpha[i] * splines[i + 1].c + beta[i];

            for (int i = n - 1; i > 0; --i)
            {
                double h_i = x[i] - x[i - 1];
                splines[i].d = (splines[i].c - splines[i - 1].c) / h_i;
                splines[i].b = h_i * (2.0 * splines[i].c + splines[i - 1].c) / 6.0 + (y[i] - y[i - 1]) / h_i;
            }
        }

        public double GetDerivative(double x, bool clipX = false)
        {
            if (splines == null)
                this.BuildSpline();

            int n = this.length;
            SplineTuple s;

            if (x < splines[0].x && !clipX)
                throw new ArgumentOutOfRangeException("x");
            else if (x > splines[n - 1].x && !clipX)
                throw new ArgumentOutOfRangeException("x");
            else if (x <= splines[0].x)
                s = splines[0];
            else if (x >= splines[n - 1].x)
                s = splines[n - 1];
            else
            {
                int i = 0, j = n - 1;
                while (i + 1 < j)
                {
                    int k = i + (j - i) / 2;
                    if (x <= splines[k].x)
                        j = k;
                    else
                        i = k;
                }
                s = splines[j];
            }

            double dx = (x - s.x);
            double result = s.b + (s.c + s.d * dx / 2.0) * dx;
            return result;
        }

        public SplineTuple GetTuple(double x)
        {
            int n = this.length;
            SplineTuple s;

            if (x < splines[0].x)
                throw new ArgumentOutOfRangeException("x");
            else if (x > splines[n - 1].x)
                throw new ArgumentOutOfRangeException("x");
            else if (x == splines[0].x)
                s = splines[0];
            else if (x == splines[n - 1].x)
                s = splines[n - 2];
            else
            {
                int i = 0, j = n - 1;
                while (i + 1 < j)
                {
                    int k = i + (j - i) / 2;
                    if (x <= splines[k].x)
                        j = k;
                    else
                        i = k;
                }
                s = splines[j];
            }
            return s;
        }

        public double this[double x, bool clipX, bool clipY]
        {
            get
            {
                if (splines == null)
                    this.BuildSpline();

                int n = this.length;
                SplineTuple s;

                if (x < splines[0].x && !clipX)
                    throw new ArgumentOutOfRangeException("x");
                else if (x > splines[n - 1].x && !clipX)
                    throw new ArgumentOutOfRangeException("x");
                else if (x <= splines[0].x)
                    s = splines[0];
                else if (x >= splines[n - 1].x)
                    s = splines[n - 1];
                else
                {
                    int i = 0, j = n - 1;
                    while (i + 1 < j)
                    {
                        int k = i + (j - i) / 2;
                        if (x <= splines[k].x)
                            j = k;
                        else
                            i = k;
                    }
                    s = splines[j];
                }

                double dx = (x - s.x);
                double result = s.a + (s.b + (s.c / 2.0 + s.d * dx / 6.0) * dx) * dx;

                if (clipY)
                {
                    if (result < this.minValue)
                        return this.minValue;
                    else if (result > this.maxValue)
                        return this.maxValue;
                    else
                        return result;
                }
                else return result;
            }
        }

        public double this[double x]
        {
            get
            {
                return this[x, false, false];
            }
        }
    }

    public struct SplineTuple
    {
        public double a, b, c, d, x;
    }
}
