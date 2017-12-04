using System;

namespace Astronomy
{
    class HarmonOscilator
    {
        public const double omega = Math.PI / 4;
        public static void HO_right_part(int nVar, double T, ref double[] Y, ref double[] G)
        {
            G[0] = Y[1];
            G[1] = -omega * omega * Y[0];
        }
    }
}