using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GUI
{
    public class Triangle3D
    {
        public Triangle3D(int m0, int m1, int m2)
        {
            n0 = m0; n1 = m1; n2 = m2;
        }

        public int n0, n1, n2;                      // vertex indice of the triangle
    }
}
