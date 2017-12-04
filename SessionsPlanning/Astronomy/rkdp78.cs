using System;

namespace Astronomy
{
  class RKDP78
  {
    // constants of method
    // 2..13 1..12
    double[,] A;
    // 1..13
    double[]  B = { 0.0, // 0
                    1400.5451 / 33548.0064, // 1 
                    0.0, 0.0, 0.0, 0.0,  // 2, 3, 4, 5
                    -5923.8493 / 106827.7825, // 6
                    18160.6767 / 75886.7731, // 7
                    56129.2985 / 79784.5732, // 8 
                    -104189.143 / 137134.3529, // 9 
                    76041.7239 / 115116.5299, // 10 
                    11882.0643 / 75113.8087, // 11 
                    -52874.7749 / 222060.717, // 12
                    1.0 / 4  // 13
                  };

     
    // 2..13
    double[] C = { 
                   0.0, 0.0, // 0, 1
                   1.0 / 18, // 2
                   1.0 / 12, // 3
                   1.0 / 8, // 4
                   5.0 / 16, // 5
                   3.0 / 8, // 6
                   59.0 / 400, // 7
                   93.0 / 200, // 8
                   549002.3248 / 971916.9821, // 9 
                   13.0 / 20, // 10
                   120114.6811 / 129901.9798, // 11 
                   1.0, // 12
                   1.0  // 13
                 };
      
    
    double[] D; 
    double[] F;

    private void InitVar()
    {
      D = new double[] { 
                         0.0, // 0
                         1345.1932 / 45517.6623, // 1
                         0.0, 0.0, 0.0, 0.0, // 2, 3, 4, 5,
                         -80871.9846 / 97600.0145, // 6
                         175700.4468 / 564515.9321, // 7
                         65604.5339 / 26589.1186, // 8
                         -386757.4721 / 151851.7206, // 9
                         46588.5868 / 32273.6535, // 10
                         5301.1238 / 66751.6719, // 11
                         2.0 / 45, // 12
                         0.0
                       };

      F = new double[14];
    }

    public void Initialization()
    {
      A = new double[14,13];
      for (int i = 0; i < 14; i++)
        for (int j = 0; j < 13; j++)
          A[i, j] = 0.0;

      A[2,1] = 1.0 / 18.0;
      A[3,1] = 1.0 / 48.0; 
      A[3,2] = 1.0 / 16.0;
      A[4,1] = 1.0 / 32.0; 
      A[4,3] = 3.0 / 32.0;
      A[5,1] = 5.0 / 16.0; 
      A[5,3] = -75.0 / 64.0; 
      A[5,4] = -A[5,3];
      A[6,1] = 3.0 / 80.0; 
      A[6,4] = 3.0 / 16.0; 
      A[6,5] = 3.0 / 20.0;
      A[7,1] = 2944.3841 / 61456.3906; 
      A[7,4] = 7773.6538 / 69253.8347;
      A[7,5] = -2869.3883 / 112500.0; 
      A[7,6] = 2312.4283 / 180000.0;
      A[8,1] = 1601.6141 / 94669.2911; 
      A[8,4] = 6156.418 / 15873.2637;
      A[8,5] = 2278.9713 / 63344.5777; 
      A[8,6] = 54581.5736 / 277105.7229;
      A[8,7] = -18019.3667 / 104330.7555;
      A[9,1] = 3963.2708 / 57359.1083; 
      A[9,4] = -43363.6366 / 68370.1615;
      A[9,5] = -42173.9975 / 261629.2301; 
      A[9,6] = 10030.2831 / 72342.3059;
      A[9,7] = 79020.4164 / 83981.3087; 
      A[9,8] = 80063.531 / 378307.1287;
      A[10,1] = 24612.1993 / 134084.7787; 
      A[10,4] = -3769504.2795 / 1526876.6246;
      A[10,5] = -30912.1744 / 106122.7803; 
      A[10,6] = -1299.2083 / 49076.6935;
      A[10,7] = 600594.3493 / 210894.7869; 
      A[10,8] = 39300.6217 / 139667.3457;
      A[10,9] = 12387.2331 / 100102.9789;
      A[11,1] = -102846.8189 / 84618.0014; 
      A[11,4] = 847823.5783 / 50851.2852;
      A[11,5] = 131172.9495 / 143242.2823; 
      A[11,6] = -1030412.9995 / 170130.4382;
      A[11,7] = -4877792.5059 / 304793.956; 
      A[11,8] = 1533672.6248 / 103282.4649;
      A[11,9] = -4544286.8181 / 339846.7696; 
      A[11,10] = 306599.3473 / 59717.2653;
      A[12,1] = 18589.2177 / 71811.6043; 
      A[12,4] = -318509.4517 / 66710.7341;
      A[12,5] = -47775.5414 / 109805.3517; 
      A[12,6] = -70363.5378 / 23073.9211;
      A[12,7] = 573156.6787 / 102754.5527; 
      A[12,8] = 523286.6602 / 85006.6563;
      A[12,9] = -409366.4535 / 80868.8257; 
      A[12,10] = 396213.7247 / 180595.7418;
      A[12,11] = 6568.6358 / 48791.0083;
      A[13,1] = 40386.3854 / 49106.3109; 
      A[13,4] = -506849.2393 / 43474.0067;
      A[13,5] = -41142.1997 / 54304.3805; 
      A[13,6] = 65278.3627 / 91429.6604;
      A[13,7] = 1117396.2825 / 92532.0556; 
      A[13,8] = -1315899.0841 / 618472.7034;
      A[13,9] = 393664.7629 / 197804.968; 
      A[13,10] = -16052.8059 / 68517.8525;
      A[13,11] = 24863.8103 / 141353.1060; 
      A[13,12] = 0;
    }

    public RKDP78()
    {
      Initialization();
    }

    public delegate void Del_right_part(int nVar, double T, ref double[] Y, ref double[] G);
    public delegate void Del_output(double T, double H, int nVar, double[] Y);
    public int  check_constants(ref double[] v)
    {
      this.InitVar();
      double S;
      int num_v = 0;
      for(int i = 2; i < 14; i++)
      {
        S = -C[i];
        for (int j = 1; j < i; j++)
          S = S + A[i, j];
        v[num_v++] = S;
      }

      
      for( int i = 3; i < 14; i++)
      {
        S = -(C[i] * C[i]) / 2;
        for (int j = 2 ;  j < i; j++)
          S = S + A[i,j] * C[j];
        v[num_v++] = S;
      }
             
      for (int i=3; i < 14; i++)
      {
        S = C[i];
        S = - S * S * S / 3;
        for (int j = 2; j < i; j++) 
          S += A[i,j]*(C[j] * C[j]);
        v[num_v++] = S;
      }
      
      
      for(int i = 6; i < 14; i++) 
      {
        S = C[i];
        S = -S * S * S * S / 4;
        for(int j = 2;  j < i; j++) 
          S += A[i,j] * C[j] * C[j] * C[j];
        v[num_v++] = S;
      }
      
      
      for(int i = 1; i < 9; i++) 
      {
        S = - 1.0 / (i + 1);
        for(int j = 2; j < 14; j++)
          S += B[j] * Math.Exp(i * Math.Log(C[j]));
        v[num_v++] = S;
      }
        
       

      for(int i = 1; i < 9; i++) 
      {
        S = -1.0 / (i + 1);
        for(int j = 2; j < 14; j++)
          S += D[j] * Math.Exp(i * Math.Log(C[j]));
        v[num_v++] = S;
      }

      
      S = -1; 
      for(int i = 1; i < 14; i++) 
        S += B[i];
      v[num_v++] = S;
     
      S = -1; 
      for(int i = 1; i < 14; i++)
        S += D[i];
      v[num_v++] = S;
    
      return num_v;
    }
    public bool intergrate(int Nvar, ref double[] Y, double t_beg, double t_end,
                           double beg_step, double start_precision, bool control,
                           Del_right_part RightPart, Del_output OutPut)
    {
      if (t_beg == t_end || beg_step <= 0 || start_precision <= 0|| control)
        return false;
      double[][] F = new double[15][];
      for(int q = 1 ; q < 15; q++)
        F[q] = new double[Nvar];

      InitVar();
      double TE;

      for(int i = 1; i < 14; i++)
        D[i] = B[i] - D[i];
      
      double T = t_beg;
      double step = T < t_end ? beg_step : -beg_step;
      while (Math.Abs(t_end - T) >= 1e-11)
      {
        if( t_end > t_beg && (step > t_end - T) || t_end < t_beg && (step < t_end - T))
          step = t_end - T;
        OutPut(T, step, Nvar, Y);
        if(control)
        {
          t_end = T;
          return true;
        }

        RightPart(Nvar, T, ref Y, ref F[1]);
        bool BOL = false;
        do
        {
          for(int i = 2; i < 14; i++)
          {
      
            for(int k = 0;  k < Nvar; k++)
            {
              double S = 0;
              for (int j = 1; j < i; j++)
                S += A[i,j]*F[j][k];
              F[14][k] = Y[k] + step * S;
            }
            RightPart(Nvar, T + step*C[i], ref F[14], ref F[i]);
          }
          TE = 0;
          for(int  k = 0; k < Nvar; k++)
          {
            double TE1 = 0;
            for (int j = 1; j < 14; j++)
              TE1 = TE1 + D[j]*F[j][k];
            TE1 = Math.Abs(TE1*step); 
            if (TE1 > TE) 
              TE = TE1;
          }

          if(TE > start_precision) 
          { 
            BOL = false; 
            step *= 0.7;
          }
          else 
            BOL = true;
        } 
        while(!BOL);

        for(int k = 0; k < Nvar; k++)
        {
          double S = 0;
          for(int i = 1; i < 14; i++)
            S += B[i] * F[i][k];
          Y[k] += S * step;
        }
    
        T += step;
        step *= 0.9 * Math.Exp(0.125 * Math.Log(start_precision / TE));
      }

      OutPut(T, step, Nvar, Y); 
      return true;
    }
  };

}