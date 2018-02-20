using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SatelliteSessions
{
    class MPZ
    {
        private static int NextNumMpz = 101;

        public HeaderMPZ Header { get; set; }
        public List<RouteMPZ> Routes { get; set; }

        public MPZ(IList<RouteMPZ> routes)
        {
            for (int i = 0; i < routes.Count; ++i)
                Routes.Add(routes[i]);

            Header = new HeaderMPZ();

            Header.NPZ = NextNumMpz;
            NextNumMpz += 1;

            Header.Nka = HeaderMPZ.NKA;
            // TODO: Header.CONF_RLCI
            Header.Ntask = Routes.Count;
            Header.PWR_ON = false; // default
            Header.Session_key_ON = false; // default
            Header.Autotune_ON = false; // default
            Header.Autotune_R1 = false; // default
            Header.Autotune_R2 = false; // default
            Header.CONF_Test_Off = 0;
            //TODO: Header.ton = routes[0].
            //TODO: Header.Ttask
            Header.Tvideo = new TimeSpan(0, 0, 90); // default TODO

        }
    }

    class HeaderMPZ
    {
        public static const int NKA = 2;
        public static const TimeSpan TON_DELTA = new TimeSpan(0, 2, 30);
        public static const TimeSpan TTASK_MAX = new TimeSpan(0, 23, 20);

        public int NPZ { get; set; } // 24 bit
        public int Nka { get; set; } // 4 bit
        public ConfRLCI CONF_RLCI { get; set; } // 4 bit
        public int Ntask { get; set; } // 4 bit
        public bool PWR_ON { get; set; } // 1 bit
        public bool Session_key_ON { get; set; } // 1 bit
        public bool Autotune_ON { get; set; } // 1 bit
        public bool Autotune_R1 { get; set; } // 1 bit
        public bool Autotune_R2 { get; set; } // 1 bit
        public int CONF_Test_Off { get; set; } // 11 bit
        public DateTime ton { get; set; }
        public TimeSpan Ttask { get; set; }
        public TimeSpan Tvideo { get; set; }
        public ConfC CONF_C { get; set; } // 2 bytes
        public ConfB CONF_B { get; set; } // 2 bytes
        public ConfZ CONF_Z { get; set; } // 4 bytes
        public int CONF_P { get; set; } // 10 bytes
        public int CONF_M { get; set; } // 10 bytes
        public int CONF_F { get; set; } // 14 bytes
        public int CodTm { get; set; } // 4 bit
        public int RegTM { get; set; } // 2 bit
        public int TypeTm { get; set; } // 2 bit
        public double[] Delta_Pasp { get; set; } // length 6
        public int Delta_Autotune { get; set; } // 2 bytes
        public int TitleRes { get; set; } // 114 bytes
    }

    class RouteMPZ
    {
        public int NPZ { get; set; } // 24 bit
        public int Nroute { get; set; } // 4 bit
        public int REGka { get; set; } // 2 bit
        public Tuple<double, double, double> InitCoord { get; set; }
        public int N_PK { get; set; } // 3 bit
        public int Z { get; set; } // 5 bit
        public int N_MK { get; set; } // 8 bit
        public TimeSpan Ts { get; set; }
        public TimeSpan Troute { get; set; }
        public int REGta { get; set; } // 16 bit
        public int REGta_Param { get; set; } // 16 bit
        public Tuple<int, int, int> IDFile { get; set; }
        public int Delta_T { get; set; } // 1 byte
        public int Hroute { get; set; } // 1 byte
        public int Target_Rate { get; set; } // 2 byte
        public int Session_key { get; set; } // 16 byte
        public int Tune_Param { get; set; } // 64 byte
        public int W_D_MpZ { get; set; } // 4 byte
        public int Coef_tang { get; set; } // 2 byte
        public int Target_RatePK { get; set; } // 2 byte
        public int Target_RateMK { get; set; } // 2 byte
        public int Quant_InitValuePK { get; set; } // 14 bit
        public int Quant_InitValueMK { get; set; } // 14 bit
        public int TaskRes { get; set; } // 106 byte
    }

    /// <summary>
    /// Конфигурация каналов связи с СВРЛ.
    /// Если в МПЗ есть хотя бы один маршрут с режимом выдачи информации, 
    /// то D11 = true; иначе всё false.
    /// </summary>
    class ConfRLCI
    {
        /// <summary>
        /// Тип НКПОИ: true - МНКПОИ, false - СНКПОИ.
        /// </summary>
        public bool D8 { get; set; }
        /// <summary>
        /// Выбор усилителя антенны: true - 2й комплект, false - 1й комплект.
        /// </summary>
        public bool D9 { get; set; }
        /// <summary>
        /// Выбор модулятора и выхода СОЭН: true - 2й комплект, false - 1й комплект.
        /// </summary>
        public bool D10 { get; set; }
        /// <summary>
        ///  Состояние СВРЛ: true - включены, false - выключены.
        ///  
        /// </summary>
        public bool D11 { get; set; }
    }

    class ConfC
    {
        public ByteAsBits byte0 { get; set; }
        public ByteAsBits byte1 { get; set; }
    }

    class ConfB
    {
        public ByteAsBits byte0 { get; set; }
        public ByteAsBits byte1 { get; set; }
    }

    class ConfZ
    {
        public ByteAsBits byte0 { get; set; }
        public ByteAsBits byte1 { get; set; }
        public ByteAsBits byte2 { get; set; }
        public ByteAsBits byte3 { get; set; }
    }

    class ByteAsBits
    {
        public bool D0 { get; set; }
        public bool D1 { get; set; }
        public bool D2 { get; set; }
        public bool D3 { get; set; }
        public bool D4 { get; set; }
        public bool D5 { get; set; }
        public bool D6 { get; set; }
        public bool D7 { get; set; }
    }
}
