using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Common;

namespace Astronomy
{
    internal class MoonInternal
    {
        const double PI2 = 6.283185308;  //(* 2*pi;  pi=3.141592654...        *)
        const double ARC = 206264.81;    //(* 3600*180/pi = arcsec per radian *)

        double DGAM, FAC;
        double DLAM, N, GAM1C, SINPI;
        double L0, L, LS, F, D, S;
        double DL0, DL, DLS, DF, DD, DS;
        double[,] CO = new double[12, 4];//ARRAY[-6..6,1..4]
        double[,] SI = new double[12, 4];//ARRAY[-6..6,1..4]

        int iOffset = 6;//because ARRAY[-6..6,1..4]
        int jOffset = -1;//because ARRAY[-6..6,1..4]


        double T;

        public MoonInternal(double time)
        {
            this.T = time;
        }

        /// <summary>
        ///  (* fractional part of a number; with several compilers it may be    *)
        ///  (* necessary to replace TRUNC by LONG_TRUNC or INT if T<-24!        *)
        /// </summary>
        internal static double FRAC(double x)
        {
            x = x - (int)x;
            if (x < 0) { x = x + 1; }
            return x;
        }

        /// <summary>
        /// (* calculate c=cos(a1+a2) and s=sin(a1+a2) from the addition theo-  *)
        /// (* rems for c1=cos(a1), s1=sin(a1), c2=cos(a2) and s2=sin(a2)       *)
        /// </summary>
        internal static void ADDTHE(double C1, double S1, double C2, double S2, ref double C, ref double S)
        {
            C = C1 * C2 - S1 * S2;
            S = S1 * C2 + C1 * S2;
        }

        /// <summary>
        /// (* calculate sin(phi); phi in units of 1 revolution = 360 degrees   *)
        /// </summary>
        internal static double SINE(double PHI)
        {
            return Math.Sin(PI2 * FRAC(PHI));
        }

        /// <summary>
        /// (* calculate long-periodic changes of the mean elements             *)
        /// (* l,l',F,D and L0 as well as dgamma                                *)
        /// </summary>
        internal static void LONG_PERIODIC(double T, ref double DL0, ref double DL, ref double DLS, ref double DF, ref double DD, ref double DGAM)
        {
            double S1, S2, S3, S4, S5, S6, S7;
            S1 = SINE(0.19833 + 0.05611 * T); S2 = SINE(0.27869 + 0.04508 * T);
            S3 = SINE(0.16827 - 0.36903 * T); S4 = SINE(0.34734 - 5.37261 * T);
            S5 = SINE(0.10498 - 5.37899 * T); S6 = SINE(0.42681 - 0.41855 * T);
            S7 = SINE(0.14943 - 5.37511 * T);
            DL0 = 0.84 * S1 + 0.31 * S2 + 14.27 * S3 + 7.26 * S4 + 0.28 * S5 + 0.24 * S6;
            DL = 2.94 * S1 + 0.31 * S2 + 14.27 * S3 + 9.34 * S4 + 1.12 * S5 + 0.83 * S6;
            DLS = -6.40 * S1 - 1.89 * S6;
            DF = 0.21 * S1 + 0.31 * S2 + 14.27 * S3 - 88.70 * S4 - 15.30 * S5 + 0.24 * S6 - 1.86 * S7;
            DD = DL0 - DLS;
            DGAM = -3332E-9 * SINE(0.59734 - 5.37261 * T)
                      - 539E-9 * SINE(0.35498 - 5.37899 * T)
                       - 64E-9 * SINE(0.39943 - 5.37511 * T);
        }

        /// <summary>
        /// (* INIT: calculates the mean elements and their sine and cosine   *)
        /// (* l mean anomaly of the Moon     l' mean anomaly of the Sun      *)
        /// (* F mean distance from the node  D  mean elongation from the Sun *)
        /// </summary>
        internal void INIT()
        {

            int MAX = 0;

            double T2 = 0;

            double ARG = 0;
            double FAC = 0;

            T2 = T * T;
            DLAM = 0; DS = 0; GAM1C = 0; SINPI = 3422.7000;
            LONG_PERIODIC(T, ref DL0, ref DL, ref DLS, ref DF, ref DD, ref DGAM);
            L0 = PI2 * FRAC(0.60643382 + 1336.85522467 * T - 0.00000313 * T2) + DL0 / ARC;
            L = PI2 * FRAC(0.37489701 + 1325.55240982 * T + 0.00002565 * T2) + DL / ARC;
            LS = PI2 * FRAC(0.99312619 + 99.99735956 * T - 0.00000044 * T2) + DLS / ARC;
            F = PI2 * FRAC(0.25909118 + 1342.22782980 * T - 0.00000892 * T2) + DF / ARC;
            D = PI2 * FRAC(0.82736186 + 1236.85308708 * T - 0.00000397 * T2) + DD / ARC;
            for (int i = 1; i <= 4; i++)
            {
                if (i == 1) { ARG = L; MAX = 4 + jOffset; FAC = 1.000002208; }
                if (i == 2) { ARG = LS; MAX = 3 + jOffset; FAC = 0.997504612 - 0.002495388 * T; }
                if (i == 3) { ARG = F; MAX = 4 + jOffset; FAC = 1.000002708 + 139.978 * DGAM; }
                if (i == 4) { ARG = D; MAX = 6 + jOffset; FAC = 1.0; }

                CO[0 + iOffset, i + jOffset] = 1.0; CO[1 + iOffset, i + jOffset] = Math.Cos(ARG) * FAC;
                SI[0 + iOffset, i + jOffset] = 0.0; SI[1 + iOffset, i + jOffset] = Math.Sin(ARG) * FAC;
                for (int j = 2; j <= MAX; j++)
                {
                    ADDTHE(CO[j - 1 + iOffset, i + jOffset], SI[j - 1 + iOffset, i + jOffset], CO[1 + iOffset, i + jOffset], SI[1 + iOffset, i + jOffset], ref CO[j + iOffset, i + jOffset], ref SI[j + iOffset, i + jOffset]);
                }
                for (int j = 1; j <= MAX; j++)
                {
                    CO[-j + iOffset, i + jOffset] = CO[j + iOffset, i + jOffset]; SI[-j + iOffset, i + jOffset] = -SI[j + iOffset, i + jOffset];
                }
            }
        }


        /// <summary>
        ///(* TERM calculates X=cos(p*arg1+q*arg2+r*arg3+s*arg4) and*)
        ///(* Y=sin(p*arg1+q*arg2+r*arg3+s*arg4)*)
        /// </summary>
        internal void TERM(int P, int Q, int R, int S, ref double X, ref double Y)
        {
            var I = new int[4];
            I[0] = P; I[1] = Q; I[2] = R; I[3] = S; X = 1.0; Y = 0.0;
            for (int i = 1; i <= 4; i++)
            {
                if (I[i + jOffset] != 0)
                    ADDTHE(X, Y, CO[I[i + jOffset] + iOffset, i + jOffset], SI[I[i + jOffset] + iOffset, i + jOffset], ref X, ref Y);
            }
        }

        internal void ADDSOL(double COEFFL, double COEFFS, double COEFFG, double COEFFP, int P, int Q, int R, int S)
        {
            double X = 0, Y = 0;
            TERM(P, Q, R, S, ref X, ref Y);
            DLAM = DLAM + COEFFL * Y; DS = DS + COEFFS * Y;
            GAM1C = GAM1C + COEFFG * X; SINPI = SINPI + COEFFP * X;
        }


        internal void SOLAR1()
        {
            ADDSOL(13.902, 14.06, -0.001, 0.2607, 0, 0, 0, 4);
            ADDSOL(0.403, -4.01, +0.394, 0.0023, 0, 0, 0, 3);
            ADDSOL(2369.912, 2373.36, +0.601, 28.2333, 0, 0, 0, 2);
            ADDSOL(-125.154, -112.79, -0.725, -0.9781, 0, 0, 0, 1);
            ADDSOL(1.979, 6.98, -0.445, 0.0433, 1, 0, 0, 4);
            ADDSOL(191.953, 192.72, +0.029, 3.0861, 1, 0, 0, 2);
            ADDSOL(-8.466, -13.51, +0.455, -0.1093, 1, 0, 0, 1);
            ADDSOL(22639.500, 22609.07, +0.079, 186.5398, 1, 0, 0, 0);
            ADDSOL(18.609, 3.59, -0.094, 0.0118, 1, 0, 0, -1);
            ADDSOL(-4586.465, -4578.13, -0.077, 34.3117, 1, 0, 0, -2);
            ADDSOL(+3.215, 5.44, +0.192, -0.0386, 1, 0, 0, -3);
            ADDSOL(-38.428, -38.64, +0.001, 0.6008, 1, 0, 0, -4);
            ADDSOL(-0.393, -1.43, -0.092, 0.0086, 1, 0, 0, -6);
            ADDSOL(-0.289, -1.59, +0.123, -0.0053, 0, 1, 0, 4);
            ADDSOL(-24.420, -25.10, +0.040, -0.3000, 0, 1, 0, 2);
            ADDSOL(18.023, 17.93, +0.007, 0.1494, 0, 1, 0, 1);
            ADDSOL(-668.146, -126.98, -1.302, -0.3997, 0, 1, 0, 0);
            ADDSOL(0.560, 0.32, -0.001, -0.0037, 0, 1, 0, -1);
            ADDSOL(-165.145, -165.06, +0.054, 1.9178, 0, 1, 0, -2);
            ADDSOL(-1.877, -6.46, -0.416, 0.0339, 0, 1, 0, -4);
            ADDSOL(0.213, 1.02, -0.074, 0.0054, 2, 0, 0, 4);
            ADDSOL(14.387, 14.78, -0.017, 0.2833, 2, 0, 0, 2);
            ADDSOL(-0.586, -1.20, +0.054, -0.0100, 2, 0, 0, 1);
            ADDSOL(769.016, 767.96, +0.107, 10.1657, 2, 0, 0, 0);
            ADDSOL(+1.750, 2.01, -0.018, 0.0155, 2, 0, 0, -1);
            ADDSOL(-211.656, -152.53, +5.679, -0.3039, 2, 0, 0, -2);
            ADDSOL(+1.225, 0.91, -0.030, -0.0088, 2, 0, 0, -3);
            ADDSOL(-30.773, -34.07, -0.308, 0.3722, 2, 0, 0, -4);
            ADDSOL(-0.570, -1.40, -0.074, 0.0109, 2, 0, 0, -6);
            ADDSOL(-2.921, -11.75, +0.787, -0.0484, 1, 1, 0, 2);
            ADDSOL(+1.267, 1.52, -0.022, 0.0164, 1, 1, 0, 1);
            ADDSOL(-109.673, -115.18, +0.461, -0.9490, 1, 1, 0, 0);
            ADDSOL(-205.962, -182.36, +2.056, +1.4437, 1, 1, 0, -2);
            ADDSOL(0.233, 0.36, 0.012, -0.0025, 1, 1, 0, -3);
            ADDSOL(-4.391, -9.66, -0.471, 0.0673, 1, 1, 0, -4);
        }


        internal void SOLAR2()
        {
            ADDSOL(0.283, 1.53, -0.111, +0.0060, 1, -1, 0, +4);
            ADDSOL(14.577, 31.70, -1.540, +0.2302, 1, -1, 0, 2);
            ADDSOL(147.687, 138.76, +0.679, +1.1528, 1, -1, 0, 0);
            ADDSOL(-1.089, 0.55, +0.021, 0.0, 1, -1, 0, -1);
            ADDSOL(28.475, 23.59, -0.443, -0.2257, 1, -1, 0, -2);
            ADDSOL(-0.276, -0.38, -0.006, -0.0036, 1, -1, 0, -3);
            ADDSOL(0.636, 2.27, +0.146, -0.0102, 1, -1, 0, -4);
            ADDSOL(-0.189, -1.68, +0.131, -0.0028, 0, 2, 0, 2);
            ADDSOL(-7.486, -0.66, -0.037, -0.0086, 0, 2, 0, 0);
            ADDSOL(-8.096, -16.35, -0.740, 0.0918, 0, 2, 0, -2);
            ADDSOL(-5.741, -0.04, 0.0, -0.0009, 0, 0, 2, 2);
            ADDSOL(0.255, 0.0, 0.0, 0.0, 0, 0, 2, 1);
            ADDSOL(-411.608, -0.20, 0.0, -0.0124, 0, 0, 2, 0);
            ADDSOL(0.584, 0.84, 0.0, +0.0071, 0, 0, 2, -1);
            ADDSOL(-55.173, -52.14, 0.0, -0.1052, 0, 0, 2, -2);
            ADDSOL(0.254, 0.25, 0.0, -0.0017, 0, 0, 2, -3);
            ADDSOL(+0.025, -1.67, 0.0, +0.0031, 0, 0, 2, -4);
            ADDSOL(1.060, 2.96, -0.166, 0.0243, 3, 0, 0, +2);
            ADDSOL(36.124, 50.64, -1.300, 0.6215, 3, 0, 0, 0);
            ADDSOL(-13.193, -16.40, +0.258, -0.1187, 3, 0, 0, -2);
            ADDSOL(-1.187, -0.74, +0.042, 0.0074, 3, 0, 0, -4);
            ADDSOL(-0.293, -0.31, -0.002, 0.0046, 3, 0, 0, -6);
            ADDSOL(-0.290, -1.45, +0.116, -0.0051, 2, 1, 0, 2);
            ADDSOL(-7.649, -10.56, +0.259, -0.1038, 2, 1, 0, 0);
            ADDSOL(-8.627, -7.59, +0.078, -0.0192, 2, 1, 0, -2);
            ADDSOL(-2.740, -2.54, +0.022, 0.0324, 2, 1, 0, -4);
            ADDSOL(1.181, 3.32, -0.212, 0.0213, 2, -1, 0, +2);
            ADDSOL(9.703, 11.67, -0.151, 0.1268, 2, -1, 0, 0);
            ADDSOL(-0.352, -0.37, +0.001, -0.0028, 2, -1, 0, -1);
            ADDSOL(-2.494, -1.17, -0.003, -0.0017, 2, -1, 0, -2);
            ADDSOL(0.360, 0.20, -0.012, -0.0043, 2, -1, 0, -4);
            ADDSOL(-1.167, -1.25, +0.008, -0.0106, 1, 2, 0, 0);
            ADDSOL(-7.412, -6.12, +0.117, 0.0484, 1, 2, 0, -2);
            ADDSOL(-0.311, -0.65, -0.032, 0.0044, 1, 2, 0, -4);
            ADDSOL(+0.757, 1.82, -0.105, 0.0112, 1, -2, 0, 2);
            ADDSOL(+2.580, 2.32, +0.027, 0.0196, 1, -2, 0, 0);
            ADDSOL(+2.533, 2.40, -0.014, -0.0212, 1, -2, 0, -2);
            ADDSOL(-0.344, -0.57, -0.025, +0.0036, 0, 3, 0, -2);
            ADDSOL(-0.992, -0.02, 0.0, 0.0, 1, 0, 2, 2);
            ADDSOL(-45.099, -0.02, 0.0, -0.0010, 1, 0, 2, 0);
            ADDSOL(-0.179, -9.52, 0.0, -0.0833, 1, 0, 2, -2);
            ADDSOL(-0.301, -0.33, 0.0, 0.0014, 1, 0, 2, -4);
            ADDSOL(-6.382, -3.37, 0.0, -0.0481, 1, 0, -2, 2);
            ADDSOL(39.528, 85.13, 0.0, -0.7136, 1, 0, -2, 0);
            ADDSOL(9.366, 0.71, 0.0, -0.0112, 1, 0, -2, -2);
            ADDSOL(0.202, 0.02, 0.0, 0.0, 1, 0, -2, -4);
        }

        internal void SOLAR3()
        {
            ADDSOL(0.415, 0.10, 0.0, 0.0013, 0, 1, 2, 0);
            ADDSOL(-2.152, -2.26, 0.0, -0.0066, 0, 1, 2, -2);
            ADDSOL(-1.440, -1.30, 0.0, +0.0014, 0, 1, -2, 2);
            ADDSOL(0.384, -0.04, 0.0, 0.0, 0, 1, -2, -2);
            ADDSOL(+1.938, +3.60, -0.145, +0.0401, 4, 0, 0, 0);
            ADDSOL(-0.952, -1.58, +0.052, -0.0130, 4, 0, 0, -2);
            ADDSOL(-0.551, -0.94, +0.032, -0.0097, 3, 1, 0, 0);
            ADDSOL(-0.482, -0.57, +0.005, -0.0045, 3, 1, 0, -2);
            ADDSOL(0.681, 0.96, -0.026, 0.0115, 3, -1, 0, 0);
            ADDSOL(-0.297, -0.27, 0.002, -0.0009, 2, 2, 0, -2);
            ADDSOL(0.254, +0.21, -0.003, 0.0, 2, -2, 0, -2);
            ADDSOL(-0.250, -0.22, 0.004, 0.0014, 1, 3, 0, -2);
            ADDSOL(-3.996, 0.0, 0.0, +0.0004, 2, 0, 2, 0);
            ADDSOL(0.557, -0.75, 0.0, -0.0090, 2, 0, 2, -2);
            ADDSOL(-0.459, -0.38, 0.0, -0.0053, 2, 0, -2, 2);
            ADDSOL(-1.298, 0.74, 0.0, +0.0004, 2, 0, -2, 0);
            ADDSOL(0.538, 1.14, 0.0, -0.0141, 2, 0, -2, -2);
            ADDSOL(0.263, 0.02, 0.0, 0.0, 1, 1, 2, 0);
            ADDSOL(0.426, +0.07, 0.0, -0.0006, 1, 1, -2, -2);
            ADDSOL(-0.304, +0.03, 0.0, +0.0003, 1, -1, 2, 0);
            ADDSOL(-0.372, -0.19, 0.0, -0.0027, 1, -1, -2, 2);
            ADDSOL(+0.418, 0.0, 0.0, 0.0, 0, 0, 4, 0);
            ADDSOL(-0.330, -0.04, 0.0, 0.0, 3, 0, 2, 0);
        }


        /// <summary>
        /// (* part N of the perturbations of ecliptic latitude*)
        /// </summary>
        internal void SOLARN(ref double N)
        {
            double X = 0, Y = 0;
            N = 0.0;
            ADDN(-526.069, 0, 0, 1, -2, ref N, ref X, ref Y); ADDN(-3.352, 0, 0, 1, -4, ref N, ref X, ref Y);
            ADDN(+44.297, +1, 0, 1, -2, ref N, ref X, ref Y); ADDN(-6.000, +1, 0, 1, -4, ref N, ref X, ref Y);
            ADDN(+20.599, -1, 0, 1, 0, ref N, ref X, ref Y); ADDN(-30.598, -1, 0, 1, -2, ref N, ref X, ref Y);
            ADDN(-24.649, -2, 0, 1, 0, ref N, ref X, ref Y); ADDN(-2.000, -2, 0, 1, -2, ref N, ref X, ref Y);
            ADDN(-22.571, 0, +1, 1, -2, ref N, ref X, ref Y); ADDN(+10.985, 0, -1, 1, -2, ref N, ref X, ref Y);
        }

        internal void ADDN(double COEFFN, int P, int Q, int R, int S, ref double N, ref double X, ref double Y)
        {
            TERM(P, Q, R, S, ref X, ref Y); N = N + COEFFN * Y;
        }


        /// <summary>
        ///       (* perturbations of ecliptic latitude by Venus and Jupiter          *)
        /// </summary>
        internal void PLANETARY(ref double DLAM)
        {
            DLAM = DLAM
              + 0.82 * SINE(0.7736 - 62.5512 * T) + 0.31 * SINE(0.0466 - 125.1025 * T)
              + 0.35 * SINE(0.5785 - 25.1042 * T) + 0.66 * SINE(0.4591 + 1335.8075 * T)
              + 0.64 * SINE(0.3130 - 91.5680 * T) + 1.14 * SINE(0.1480 + 1331.2898 * T)
              + 0.21 * SINE(0.5918 + 1056.5859 * T) + 0.44 * SINE(0.5784 + 1322.8595 * T)
              + 0.24 * SINE(0.2275 - 5.7374 * T) + 0.28 * SINE(0.2965 + 2.6929 * T)
              + 0.33 * SINE(0.3132 + 6.3368 * T);
        }

        //(*-----------------------------------------------------------------------*)
        //(* MOON: analytical lunar theory by E.W.Brown (Improved Lunar Ephemeris) *)
        //(*       with an accuracy of approx. 1"                                  *)
        //(*                                                                       *)
        //(*       T:      time in Julian centuries since J2000 (Ephemeris Time)   *)
        //(*               (T=(JD-2451545.0)/36525.0)                              *)
        //(*       LAMBDA: geocentric ecliptic longitude (equinox of date)         *)
        //(*       BETA:   geocentric ecliptic latitude  (equinox of date)         *)
        //(*       R:      geocentric distance (in Earth radii)                    *)
        //(*                                                                       *)
        //(*-----------------------------------------------------------------------*)
        public void Calculate(out double LAMBDA, out double BETA, out double R)
        {
            INIT();
            SOLAR1();
            SOLAR2();
            SOLAR3();

            SOLARN(ref N);
            PLANETARY(ref DLAM);

            LAMBDA = 360.0 * FRAC((L0 + DLAM / ARC) / PI2);

            S = F + DS / ARC;
            FAC = 1.000002708 + 139.978 * DGAM;
            BETA = (FAC * (18518.511 + 1.189 + GAM1C) * Math.Sin(S) - 6.24 * Math.Sin(3 * S) + N) / 3600.0;

            SINPI = SINPI * 0.999953253;
            R = ARC / SINPI;
        }
    }

    /// <summary>
    /// Calculates Moon coordinates in specified time.
    /// </summary>
    public static class Moon
    {
        /// <summary>
        /// Calculates current ecliptic coordinates of Moon.
        /// </summary>
        /// <param name="time">Time in Julian centuries since J2000 (Ephemeris Time)</param>
        /// <param name="lambda">Geocentric ecliptic longitude (equinox of date)</param>
        /// <param name="beta">Geocentric ecliptic latitude  (equinox of date)</param>
        /// <param name="r">Geocentric distance (in Earth radius units)</param>
        public static void GetEclipticCoordinates(double time, out double lambda, out double beta, out double r)
        {
            var moonInternal = new MoonInternal(time);
            moonInternal.Calculate(out lambda, out beta, out r);
        }
    }
}