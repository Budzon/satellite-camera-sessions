using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Windows.Media.Media3D;

namespace GUI
{
    public class Ellipse3D : ColorMesh3D
    {
        public List<int> PaintOnlyThese;
        // the first 3 parameters are the ellipse size, last parameter is the smoothness of the ellipse
        public Ellipse3D(double a, double b, double h, int nRes)
        {
            PaintOnlyThese = new List<int>();
            SetMesh(nRes);
            SetData(a, b, h);
        }

        // set the mesh structure (triangle connection)
        void SetMesh(int nRes)
        {
            // one vertex at top and bottom, other ring has nRes vertex
            int nVertNo = (nRes - 2) * nRes + 2;
            // the top and bottom band has nRes triangle
            // middle band has 2*nRes*(nRes - 2 - 1)
            int nTriNo = 2 * nRes * (nRes - 3) + 2 * nRes;
            SetSize(nVertNo, nTriNo);

            // set triangle
            int n00, n01, n10, n11;
            int nTriIndex = 0;
            int nI2;
            // first set top band
            int i;
            int j = 1;
            for (i = 0; i < nRes; i++)
            {
                if (i == (nRes - 1)) nI2 = 0;
                else nI2 = i + 1;

                n00 = 1 + (j - 1) * nRes + i;
                n10 = 1 + (j - 1) * nRes + nI2;
                n01 = 0;

                SetTriangle(nTriIndex, n00, n10, n01);
                nTriIndex++;
            }
            // set middle bands
            for (j = 1; j < (nRes - 2); j++)
            {
                for (i = 0; i < nRes; i++)
                {
                    if (i == (nRes - 1)) nI2 = 0;
                    else nI2 = i + 1;
                    n00 = 1 + (j - 1) * nRes + i;
                    n10 = 1 + (j - 1) * nRes + nI2;
                    n01 = 1 + j * nRes + i;
                    n11 = 1 + j * nRes + nI2;

                    SetTriangle(nTriIndex, n00, n01, n10);
                    SetTriangle(nTriIndex + 1, n01, n11, n10);
                    nTriIndex += 2;
                }
            }

            j = nRes - 2;
            for (i = 0; i < nRes; i++)
            {
                if (i == (nRes - 1)) nI2 = 0;
                else nI2 = i + 1;

                n00 = 1 + (j - 1) * nRes + i;
                n10 = 1 + (j - 1) * nRes + nI2;
                n01 = nVertNo - 1;

                SetTriangle(nTriIndex, n00, n01, n10);
                nTriIndex++;
            }
            m_nRes = nRes;
        }

        // the vertex location according the ellipse size
        void SetData(double a, double b, double h)
        {
            double aXYStep = 2.0f * 3.1415926f / ((float)m_nRes);
            double aZStep = 3.1415926f / ((float)m_nRes - 1);

            SetPoint(0, 0, 0, h);            // first vertex is at top

            int i, j;
            double x1, y1, z1;
            for (j = 1; j < (m_nRes - 1); j++)
            {
                for (i = 0; i < m_nRes; i++)
                {
                    double aXY = ((double)i) * aXYStep;
                    double aZAngle = ((double)j) * aZStep;

                    x1 = a * System.Math.Sin(aZAngle) * System.Math.Cos(aXY);
                    y1 = b * System.Math.Sin(aZAngle) * System.Math.Sin(aXY);
                    z1 = h * System.Math.Cos(aZAngle);
                    SetPoint((j - 1) * m_nRes + i + 1, x1, y1, z1);

                    if (Math.Abs(90 - aZAngle * 180 / Math.PI) < 25 
                        && (aXY * 180 / Math.PI < 25 || aXY * 180 / Math.PI > 335))
                        PaintOnlyThese.Add((j - 1) * m_nRes + i + 1);
                }
            }
            SetPoint((m_nRes - 2) * m_nRes + 1, 0, 0, -h);

            m_xMin = -a;
            m_xMax = a;
            m_yMin = -b;
            m_yMax = b;
            m_zMin = -h;
            m_zMax = h;
        }

        private int m_nRes;
    }

    public class EllipseRegion3D : ColorMesh3D
    {
        private double squareHalfSize;
        public EllipseRegion3D(double a, double b, double h, int squareHalfSizeDeg, int nRes)
        {
            squareHalfSize = squareHalfSizeDeg * Math.PI / 180;
            SetMesh(nRes);
            SetData(a, b, h);
        }

        // set the mesh structure (triangle connection)
        void SetMesh(int nRes)
        {
            int nVertNo = nRes * nRes;
            int nTriNo = 2 * (nRes - 1) * (nRes - 1);
            SetSize(nVertNo, nTriNo);

            int n00, n10, n01, n11, nTriIndex = 0;
            for (int j = 0; j < nRes - 1; j++)
            {
                for (int i = 0; i < nRes - 1; i++)
                {
                    n00 = j * nRes + i;
                    n10 = j * nRes + i + 1;
                    n01 = (j+1) * nRes + i;
                    n11 = (j+1) * nRes + i + 1;

                    SetTriangle(nTriIndex, n00, n01, n10);
                    SetTriangle(nTriIndex + 1, n01, n11, n10);
                    nTriIndex += 2;
                }
            }

            m_nRes = nRes;
        }

        // the vertex location according the ellipse size
        void SetData(double a, double b, double h)
        {

            double aXYStep = 2 * squareHalfSize / (m_nRes - 1);
            double aZStep = 2 * squareHalfSize / (m_nRes - 1);

            double x1, y1, z1;
            for (int j = 0; j < m_nRes; j++)
            {
                for (int i = 0; i < m_nRes; i++)
                {
                    double aXY = -squareHalfSize + i * aXYStep;
                    double aZAngle = -squareHalfSize + j * aZStep;

                    x1 = a * System.Math.Cos(aZAngle) * System.Math.Cos(aXY);
                    y1 = b * System.Math.Cos(aZAngle) * System.Math.Sin(aXY);
                    z1 = h * System.Math.Sin(aZAngle);
                    SetPoint(j * m_nRes + i, x1, y1, z1);
                }
            }

            m_xMin = -a * Math.Cos(squareHalfSize);
            m_xMax = a * Math.Cos(squareHalfSize);
            m_yMin = -b * Math.Cos(squareHalfSize);
            m_yMax = b * Math.Cos(squareHalfSize);
            m_zMin = -h * Math.Sin(squareHalfSize);
            m_zMax = h * Math.Sin(squareHalfSize);
        }

        private int m_nRes;
    }
}
