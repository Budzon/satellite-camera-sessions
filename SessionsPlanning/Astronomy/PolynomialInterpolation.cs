using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Astronomy
{
    public class PolynomialInterpolation
    {
        double[] x;
        double[] y;
        double[] coeffs;

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

        int power;

        public PolynomialInterpolation(double[] x, double[] y, int power)
        {
            if (x == null)
                throw new ArgumentNullException("x");
            if (y == null)
                throw new ArgumentNullException("y");
            if (x.Length != y.Length)
                throw new ArgumentException("x length must be the same as y length");
            this.power = power;
            this.x = x;
            this.y = y;

            this.minX = x[0];
            this.maxX = x[x.Length - 1];

            double[,] coeffs = new double[power + 1, power + 2];

            for (int j = 0; j <= power; j++)
            {
                coeffs[j, power + 1] = 0;
                for (int i = 0; i < x.Length; i++)
                {
                    coeffs[j, power + 1] -= Math.Pow(x[i], j) * y[i];
                }

                for (int a_sub = 0; a_sub <= power; a_sub++)
                {
                    coeffs[j, a_sub] = 0;
                    for (int i = 0; i < x.Length; i++)
                    {
                        coeffs[j, a_sub] -= Math.Pow(x[i], a_sub + j);
                    }
                }
            }

            int max_equation = coeffs.GetUpperBound(0);
            int max_coeff = coeffs.GetUpperBound(1);
            for (int i = 0; i <= max_equation; i++)
            {
                if (coeffs[i, i] == 0)
                {
                    for (int j = i + 1; j <= max_equation; j++)
                    {
                        if (coeffs[j, i] != 0)
                        {
                            for (int k = i; k <= max_coeff; k++)
                            {
                                double temp = coeffs[i, k];
                                coeffs[i, k] = coeffs[j, k];
                                coeffs[j, k] = temp;
                            }
                            break;
                        }
                    }
                }
                double coeff_i_i = coeffs[i, i];
                if (coeff_i_i == 0)
                {
                    throw new ArithmeticException(String.Format(
                        "There is no unique solution for these points.",
                        coeffs.GetUpperBound(0) - 1));
                }

                for (int j = i; j <= max_coeff; j++)
                {
                    coeffs[i, j] /= coeff_i_i;
                }

                for (int j = 0; j <= max_equation; j++)
                {
                    if (j != i)
                    {
                        double coef_j_i = coeffs[j, i];
                        for (int d = 0; d <= max_coeff; d++)
                        {
                            coeffs[j, d] -= coeffs[i, d] * coef_j_i;
                        }
                    }
                }
            }

            double[] solution = new double[max_equation + 1];
            for (int i = 0; i <= max_equation; i++)
            {
                solution[i] = coeffs[i, max_coeff];
            }
            this.coeffs = solution;
        }

        public double GetValue(double x)
        {
            double total = 0;
            double x_factor = 1;
            for (int i = 0; i < coeffs.Length; i++)
            {
                total += x_factor * coeffs[i];
                x_factor *= x;
            }
            return total;
        }
    }
}