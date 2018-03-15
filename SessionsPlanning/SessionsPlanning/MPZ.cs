using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SatelliteSessions
{
    public class MPZ
    {
        private static int NextNumMpz = 101;
        private OptimalChain.MPZParams parameters;

        public HeaderMPZ Header { get; set; }
        public List<RouteMPZ> Routes { get; set; }
        public OptimalChain.MPZParams Parameters { get { return parameters; } }

        public MPZ(OptimalChain.MPZParams inpParameters)
        {
            List<RouteMPZ> routes = new List<RouteMPZ>();
            parameters = inpParameters;
            foreach (var rout_params in inpParameters.routes)
            {
                routes.Add(new RouteMPZ(rout_params));
            }

            loadRoutes(routes);
            ///@todo разобраться, реализовать
        }
 
        public MPZ(IList<RouteMPZ> routes)
        {
            loadRoutes(routes);
        }
 
        private void loadRoutes(IList<RouteMPZ> routes)
        {
            Header = new HeaderMPZ();
            Routes = new List<RouteMPZ>();
            Header.NPZ = NextNumMpz;
            NextNumMpz += 1;
            Header.Ntask = routes.Count;
            for (int i = 0; i < routes.Count; ++i)
            {
                Routes.Add(routes[i]);
                Routes[i].NPZ = Header.NPZ;
            }
        }

    }

    public class HeaderMPZ
    {
        public const int NKA = 3;
        public static TimeSpan TON_DELTA = new TimeSpan(0, 2, 30);
        public static TimeSpan TTASK_MAX = new TimeSpan(0, 23, 20);

        public int NPZ { get; set; } // 24 bit
        public int Nka { get; set; } // 4 bit
        public byte CONF_RLCI { get; set; } // 4 bit
        public int Ntask { get; set; } // 4 bit
        public int PWR_ON { get; set; } // 1 bit
        public int Session_key_ON { get; set; } // 1 bit
        public int Autotune_ON { get; set; } // 1 bit
        public int Autotune_R1 { get; set; } // 1 bit
        public int Autotune_R2 { get; set; } // 1 bit
        public int CONF_Test_Off { get; set; } // 11 bit
        public DateTime ton { get; set; }
        public TimeSpan Ttask { get; set; }
        public TimeSpan Tvideo { get; set; }
        public byte[] CONF_C { get; set; } // 2 bytes
        public byte[] CONF_B { get; set; } // 2 bytes
        public byte[] CONF_Z { get; set; } // 4 bytes
        public byte[] CONF_P { get; set; } // 10 bytes
        public byte[] CONF_M { get; set; } // 10 bytes
        public byte[] CONF_F { get; set; } // 14 bytes
        public int CodTm { get; set; } // 4 bit
        public int RegTM { get; set; } // 2 bit
        public int TypeTm { get; set; } // 2 bit
        public double[] Delta_Pasp { get; set; } // length 6
        public int Delta_Autotune { get; set; } // 2 bytes
        public byte[] TitleRes { get; set; } // 114 bytes

        public HeaderMPZ()
        {
            // NPZ to be filled in MPZ
            Nka = HeaderMPZ.NKA;
            // CONF_RLCI TO FILL
            // Ntask to be filled in MPZ
            PWR_ON = 0; // default
            Session_key_ON = 0; // default
            Autotune_ON = 0; // default
            Autotune_R1 = 0; // default
            Autotune_R2 = 0; // default
            CONF_Test_Off = 0;
            //TODO: ton = routes[0].
            //TODO: Ttask
            Tvideo = new TimeSpan(0, 0, 90); // default TODO
            CONF_C = new byte[2];
            CONF_B = new byte[2];
            CONF_Z = new byte[4];
            CONF_P = new byte[10];
            CONF_M = new byte[10];
            CONF_F = new byte[14];
            CodTm = 10; // default
            RegTM = 0; // default
            TypeTm = 0; // default
            Delta_Pasp = new double[6] { 0, 0, 0, 0, 0, 0 };
            Delta_Autotune = 0; //default
            TitleRes = new byte[114];
        }
    }

    public class RouteMPZ
    {
        public int NPZ { get; set; } // 24 bit
        public int Nroute { get; set; } // 4 bit
        public int REGka { get; set; } // 2 bit
        public Coord InitCoord { get; set; }
        public int N_PK { get; set; } // 3 bit
        public byte Z { get; set; } // 5 bit
        public int N_MK { get; set; } // 8 bit
        public TimeSpan Ts { get; set; }
        public TimeSpan Troute { get; set; }
        public byte[] REGta { get; set; } // 16 bit
        public RegimeTypes RegimeType { get; set; } 
        public byte[] REGta_Param { get; set; } // 16 bit
        public IdFile IDFile { get; set; }
        public int Delta_T { get; set; } // 1 byte
        public int Hroute { get; set; } // 1 byte
        public int Target_Rate { get; set; } // 2 byte
        public byte[] Session_key { get; set; } // 16 byte
        public byte[] Tune_Param { get; set; } // 64 byte
        public int W_D_MpZ { get; set; } // 4 byte
        public int Coef_tang { get; set; } // 2 byte
        public int Target_RatePK { get; set; } // 2 byte
        public int Target_RateMK { get; set; } // 2 byte
        public int Quant_InitValuePK { get; set; } // 14 bit
        public int Quant_InitValueMK { get; set; } // 14 bit
        public byte[] TaskRes { get; set; } // 106 byte

        private int zip_pk;
        private int zip_mk;

        private OptimalChain.RouteParams parameters;
        public OptimalChain.RouteParams Parameters { get {return parameters;} }

        public RouteMPZ(OptimalChain.RouteParams inpParameters) : this(new RegimeTypes() )
        {
            parameters = inpParameters;
            // parameters.File_Size = (int)Math.Ceiling(ComputeFileSize(parameters));
        }

        public RouteMPZ(RegimeTypes regimeType)
        {
            RegimeType = regimeType;
            NPZ = -1; // to be filled in the MPZ constructor
            Nroute = -1; // to be filled in the MPZ constructor

            switch (regimeType)
            {
                case RegimeTypes.NP:
                case RegimeTypes.ZI:
                case RegimeTypes.NP_fok_yust:
                case RegimeTypes.ZI_fok_yust:
                // case КАЛИБРОВКА:
                    REGka = 0;
                    break;
                case RegimeTypes.SI:
                case RegimeTypes.VI:
                // case КАЛИБРОВКА:
                case RegimeTypes.KPI_load:
                case RegimeTypes.KPI_unload:
                case RegimeTypes.PUF_control:
                case RegimeTypes.BBZU_control:
                case RegimeTypes.Special:
                    REGka = 1;
                    break;
                //default:
                //    REGka = -1; // something went wrong?
                //    break;
            }

            switch (REGka)
            {
                case 0:
                    //InitCoord = new Coord{Bc, Lc, Hc}
                    break;
                case 1:
                    InitCoord = new Coord { Bc = 0, Lc = 0, Hc = 0 };
                    break;
            }

            N_PK = 0; // default

            Z = unchecked((byte)(~7)); // default

            N_MK = 0; // default

            // Ts -- TODO
            // Troute -- TODO

            REGta = new byte[2] { 0, (byte)regimeType }; // 0-11 are 0 by default
            for (int i = 8; i < 12; ++i) // fill the regime type
                REGta[1] *= 2;

            REGta_Param = new byte[2];
            switch (regimeType)
            {
                case RegimeTypes.SI:
                case RegimeTypes.ZI:
                case RegimeTypes.VI:
                case RegimeTypes.NP:
                    REGta_Param[0] = 1 + 2 + 16 + 32; // default
                    REGta_Param[1] = 4 + 16; // default
                    break;
                case RegimeTypes.ZI_cal:
                case RegimeTypes.ZI_fok_yust:
                case RegimeTypes.NP_fok_yust:
                case RegimeTypes.KPI_load:
                case RegimeTypes.KPI_unload:
                case RegimeTypes.PUF_control:
                    // заполняет Пеленг
                    break;
                case RegimeTypes.BBZU_control:
                case RegimeTypes.Special:
                    // заполняет ЦУКС, но не заданы значения по умолчанию :(
                    break;
            }

            IDFile = new IdFile { TNPZ = 0, TNroute = 0, TNPos = 0 }; // болванка
            Delta_T = 0; // default
            Hroute = 0; // болванка
            Target_Rate = 0; // default
            Session_key = new byte[16]; // болванка
            Tune_Param = new byte[64]; // болванка
            W_D_MpZ = 0; // default
            Coef_tang = 0; // default
            Target_RatePK = 0; // default
            Target_RateMK = 0; // default
            Quant_InitValuePK = 0; // default
            Quant_InitValueMK = 0; // default
            TaskRes = new byte[106];
        }

        /// <summary>
        /// В Мб.
        /// </summary>
        /// <param name="routeParams"></param>
        /// <returns></returns>
        private double ComputeFileSize(OptimalChain.RouteParams routeParams)
        {
            int Nm = 0;
            for (int i = 3; i < 7; ++i)
                Nm += ByteRoutines.GetBit(Z, i);
            int Np = ByteRoutines.GetBit(Z, 7);

            int CodVznCalibr;
            if (RegimeType == RegimeTypes.ZI_cal)
                if (REGta_Param[1] == 0)
                    CodVznCalibr = 1;
                else
                    CodVznCalibr = REGta_Param[1];
            else
                CodVznCalibr = 1;

            return OptimalChain.RouteParams.InformationFluxInBits(
                routeParams.ShootingConf.roll, routeParams.ShootingConf.pitch,
                Hroute, CodVznCalibr, Nm, zip_mk, Np, zip_pk) * Troute.TotalSeconds / (1 << 23);
        }
    }

    public enum RegimeTypes 
    { 
        SI, ZI, VI, NP, // штатные
        Reserve4, ZI_cal, ZI_fok_yust, NP_fok_yust, // технологические
        Reserve8, KPI_load, KPI_unload, PUF_control, BBZU_control, Special, Reserve14, Reserve15 // служебные
    }

    public class Coord
    {
        public double Bc { get; set; }
        public double Lc { get; set; }
        public double Hc { get; set; }
    }

    public class IdFile
    {
        public int TNPZ { get; set; }
        public int TNroute { get; set; }
        public int TNPos { get; set; }
    }

    public static class ByteRoutines
    {
        public static int GetBit(byte[] data, int index)
        {
            return GetBit(data[index / 8], index % 8);
        }

        public static int GetBit(byte datum, int index)
        {
            if (index > 7)
                throw new IndexOutOfRangeException(
                    String.Format("Trying to access the {0}-th bit of a byte.", index));
            return (datum & (1 << index)) != 0 ? 1 : 0;
        }

        public static void SetBitOne(ref byte datum, int index)
        {
            if (index > 7)
                throw new IndexOutOfRangeException(
                    String.Format("Trying to access the {0}-th bit of a byte.", index));
            datum |= (byte)(1 << index);
        }

        public static void SetBitZero(ref byte datum, int index)
        {
            if (index > 7)
                throw new IndexOutOfRangeException(
                    String.Format("Trying to access the {0}-th bit of a byte.", index));
            datum &= unchecked((byte)(~(1 << index)));
        }
    }

    //public class Bytes
    //{
    //    private Bits[] bytes;

    //    /// <summary>
    //    /// Возвращает байт по его номеру.
    //    /// </summary>
    //    /// <param name="index"></param>
    //    /// <returns></returns>
    //    public Bits this[int index]
    //    {
    //        get
    //        {
    //            return bytes[index];
    //        }
    //    }

    //    /// <summary>
    //    /// Возвращает бит по его номеру в сквозной нумерации.
    //    /// </summary>
    //    /// <param name="index"></param>
    //    /// <returns></returns>
    //    public byte GetBit(int index)
    //    {
    //        return bytes[index / 8][index % 8];
    //    }
    //    /// <summary>
    //    /// Задает значение биту по его номеру в сквозной нумерации.
    //    /// </summary>
    //    /// <param name="index"></param>
    //    /// <param name="bit"></param>
    //    /// <returns></returns>
    //    public void SetBit(int index, byte bit)
    //    {
    //        bytes[index / 8][index % 8] = bit;
    //    }

    //    public int Count { get { return bytes.Length; } }

    //    public Bytes(int count)
    //    {
    //        bytes = new Bits[count];
    //        for (int i = 0; i < count; ++i)
    //            bytes[i] = new Bits();
    //    }
    //}


    //public class Bits
    //{
    //    private byte[] bits;
    //    private int count;

    //    /// <summary>
    //    ///  Возвращает бит по его номеру.
    //    /// </summary>
    //    /// <param name="index"></param>
    //    /// <returns></returns>
    //    public byte this[int index]
    //    {
    //        get
    //        {
    //            return bits[index];
    //        }
    //        set
    //        {
    //            bits[index] = value;
    //        }
    //    }
    //    public int Count { get { return count; } }

    //    public Bits(int count = 8)
    //    {
    //        bits = new byte[count];
    //        this.count = count;
    //    }
    //}
}
