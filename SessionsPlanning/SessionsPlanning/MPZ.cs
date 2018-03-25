using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Astronomy;
using System.Windows.Media.Media3D;

namespace SatelliteSessions
{
    public class MPZ
    {
        private static int NextNumMpz = 101;
        private OptimalChain.MPZParams parameters = null;

        public HeaderMPZ Header { get; set; }
        public List<RouteMPZ> Routes { get; set; }
        public OptimalChain.MPZParams Parameters { get { return parameters; } }

        public MPZ(OptimalChain.MPZParams inpParameters, 
            bool mainKeyBVIP, bool mainKeyBBZU, bool mainKeyVIP1, bool mainKeyYKPD1, bool mainKeyYKPD2)
        {
            List<RouteMPZ> routes = new List<RouteMPZ>();
            parameters = inpParameters;
            foreach (var rout_params in inpParameters.routes)
            {
                routes.Add(new RouteMPZ(rout_params));
            }

            bool hasPK = parameters.routes.Any(route => route.shooting_channel == "pk" || route.shooting_channel == "cm");
            bool hasMK = parameters.routes.Any(route => route.shooting_channel == "mk" || route.shooting_channel == "cm");

            // ВКЛЮЧАЕМ ПЕРВУЮ ЯП
            bool useYKZU1 = true; //parameters.routes.Any(route => route.memoryCellMZU1 == 1); // пишет ли кто в 1ю ЯП
            bool useYKZU2 = false; //parameters.routes.Any(route => route.memoryCellMZU1 == 2); // пишет ли кто в 2ю ЯП
            // НУЖНО ЕЩЕ УЧЕСТЬ КОГДА ЯП НЕ ВЫБРАНА!!! (+ мзу1 и мзу2 симметричны всё-таки или нет?)

            Header = new HeaderMPZ(hasPK, hasMK, mainKeyBVIP, mainKeyBBZU, mainKeyVIP1, useYKZU1, useYKZU2, mainKeyYKPD1, mainKeyYKPD2);

            /* ---------- NPZ -----------*/
            Header.NPZ = parameters.id;
            //Header.NPZ = NextNumMpz;
            //NextNumMpz += 1;

            /* ---------- Ntask -----------*/
            Header.Ntask = parameters.N_routes;

            /* ---------- PWR_ON -----------*/
            Header.PWR_ON = (byte)(parameters.PWR_ON ? 1 : 0);

            loadRoutes(routes);

            /* ---------- CONF_RLCI -----------*/
            DIOS.Common.SqlManager DBmanager = new DIOS.Common.SqlManager("Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER");
            DBTables.DataFetcher fetcher = new DBTables.DataFetcher(DBmanager);
            if (nkpoiOnTheLeft(fetcher))
                Header.CONF_RLCI += 8; // ВЫБОР АНТЕННЫ АРМ2
            else
                Header.CONF_RLCI += 0; // ВЫБОР АНТЕННЫ АРМ1
            bool dumpsData = Routes.Any(route => route.RegimeType == RegimeTypes.VI || route.RegimeType == RegimeTypes.NP);
            if (dumpsData)
                Header.CONF_RLCI += 32; // ВКЛЮЧИТЬ СВРЛ
            else
                Header.CONF_RLCI += 0; // ВЫКЛЮЧИТЬ СВРЛ

            /* ---------- Session_key_On -----------*/
            //if (dumpsData)
            //    Header.Session_key_ON = 1; // использовать ключ при сбросе (УТОЧНИТЬ!)
            Header.Session_key_ON = 1; // ВСЕГДА ПОКА ЧТО

            /* ---------- Autotune_On -----------*/
            bool filmData = Routes.Any(route => route.RegimeType == RegimeTypes.ZI || route.RegimeType == RegimeTypes.NP);
            if (filmData)
                Header.Autotune_ON = 1; // автоюстировка при съемке


            /* ---------- REGta_Param -----------*/
            for (int i = 0; i < Routes.Count; ++i)
            {
                if (useYKZU1)
                {
                    ByteRoutines.SetBitOne(Routes[i].REGta_Param, 10);
                    ByteRoutines.SetBitOne(Routes[i].REGta_Param, 12);
                }
                else if (useYKZU2)
                {
                    ByteRoutines.SetBitOne(Routes[i].REGta_Param, 11);
                    ByteRoutines.SetBitOne(Routes[i].REGta_Param, 13);
                }
                else
                {
                    ByteRoutines.SetBitOne(Routes[i].REGta_Param, 10);
                    ByteRoutines.SetBitOne(Routes[i].REGta_Param, 12);
                    ByteRoutines.SetBitOne(Routes[i].REGta_Param, 11);
                    ByteRoutines.SetBitOne(Routes[i].REGta_Param, 13);
                }
            }
        }
 
        //public MPZ(IList<RouteMPZ> routes)
        //{
        //    loadRoutes(routes);
        //}
 
        private void loadRoutes(List<RouteMPZ> routes)
        {
            Routes = routes;
            
            for (int i = 0; i < routes.Count; ++i)
            {
                Routes[i].NPZ = Header.NPZ;
                Routes[i].Nroute = i;
                Routes[i].Ts = i == 0 ? (int)(HeaderMPZ.TON_DELTA.TotalSeconds * 5) : Routes[i - 1].Ts + Routes[i - 1].Troute; 
                if (i + 1 < routes.Count - 1)
                {
                    if ((Routes[i + 1].Parameters.start - Routes[i].Parameters.end).TotalSeconds > 60)
                        ByteRoutines.SetBitOne(Routes[i].REGta, 10); // suspend_mode
                }
            }
            if (Routes.Count > 0)
                Header.ton = (Routes[0].startTime - HeaderMPZ.TON_DELTA); // ПРВЕРИТЬ АДЕКВАТНОСТЬ
            Header.Ttask = (uint)((parameters.end - parameters.start).TotalSeconds * 5);
            try
            {
                RouteMPZ firstVideo = Routes.First(route => route.REGka == 0);
                Header.Tvideo = (uint)(firstVideo.Ts - 500);
            }
            catch
            {
                // Нет маршрутов со съемкой
                Header.Tvideo = (uint)(23 * 60 + 20) * 5;
            }
        }

        private bool nkpoiOnTheLeft(DBTables.DataFetcher fetcher)
        {
            Vector3D snkpoi = fetcher.GetPositionSNKPOI();
            TrajectoryPoint ka = fetcher.GetPositionSat(parameters.start).Value;
            Vector3D kapos = ka.Position.ToVector();
            Vector3D planeNormal = Vector3D.CrossProduct(kapos, ka.Velocity);
            planeNormal.Normalize();

            return Vector3D.DotProduct(snkpoi - kapos, planeNormal) > 0;
        }
    }

    public class HeaderMPZ
    {
        public const int NKA = 3;
        public static TimeSpan TON_DELTA = new TimeSpan(0, 2, 30);
        public static TimeSpan TTASK_MAX = new TimeSpan(0, 23, 20);

        public int NPZ { get; set; } // 24 bit
        public byte Nka { get; set; } // 4 bit
        public byte CONF_RLCI { get; set; } // 6 bit
        public int Ntask { get; set; } // 4 bit
        public byte PWR_ON { get; set; } // 1 bit
        public byte Session_key_ON { get; set; } // 1 bit
        public byte Autotune_ON { get; set; } // 1 bit
        public byte Autotune_R1 { get; set; } // 1 bit, ЦУП
        public byte Autotune_R2 { get; set; } // 1 bit, ЦУП
        public int CONF_Test_Off { get; set; } // 11 bit, ЦУП
        public DateTime ton { get; set; }
        public uint Ttask { get; set; }
        public uint Tvideo { get; set; }

        public byte[] CONF_C { get; set; } // 2 bytes
        public byte[] CONF_B { get; set; } // 2 bytes
        public byte[] CONF_Z { get; set; } // 4 bytes, ЦУП
        public byte[] CONF_P { get; set; } // 10 bytes, ЦУП
        public byte[] CONF_M { get; set; } // 10 bytes, ЦУП
        public byte[] CONF_F { get; set; } // 14 bytes, ЦУП

        public byte[] RCONF_C { get; set; } // 2 bytes
        public byte[] RCONF_B { get; set; } // 2 bytes
        public byte[] RCONF_Z { get; set; } // 4 bytes, ЦУП
        public byte[] RCONF_P { get; set; } // 10 bytes, ЦУП
        public byte[] RCONF_M { get; set; } // 10 bytes, ЦУП
        public byte[] RCONF_F { get; set; } // 14 bytes, ЦУП

        public byte CodTm { get; set; } // 4 bit
        public byte RegTM { get; set; } // 2 bit
        public int TypeTm { get; set; } // 2 bit
        public double[] Delta_Pasp { get; set; } // length 6
        public int Delta_Autotune { get; set; } // 2 bytes
        public byte[] TitleRes { get; set; } // 114 bytes

        /// <summary>
        /// Заполняет все поля заголовка, кроме: 
        /// NPZ,
        /// CONF_RLCI(выбор антенны(D3) и состояние сврл(D5)),
        /// Ntask,
        /// PWR_ON,
        /// Session_key_ON,
        /// Autotune_ON,
        /// ton,
        /// Ttask,
        /// Tvideo.
        /// </summary>
        /// <param name="hasPK">Есть ли съемка ПК</param>
        /// <param name="hasMK">Есть ли съемка МК</param>
        /// <param name="mainKeyBVIP">CONF_C, 0, D4</param>
        /// <param name="mainKeyBBZU">CONF_B, 0, D0</param>
        /// <param name="mainKeyVIP1">CONF_B, 0, D1</param>
        /// <param name="useYKZU1_1">CONF_B, 0, D2</param>
        /// <param name="useYKZU2_1">CONF_B, 0, D4</param>
        /// <param name="mainKeyYKPD1">CONF_B, 1, D1</param>
        /// <param name="mainKeyYKPD2">CONF_B, 1, D3</param>
        public HeaderMPZ(bool hasPK, bool hasMK, 
            bool mainKeyBVIP, bool mainKeyBBZU, bool mainKeyVIP1, 
            bool useYKZU1_1, bool useYKZU2_1, bool mainKeyYKPD1, bool mainKeyYKPD2)
        {
            /* ---------- NPZ -----------*/
            // to be filled in MPZ

            /* ---------- NKA -----------*/
            Nka = HeaderMPZ.NKA;

            /* ---------- CONF_RLCI -----------*/
            byte tmp = 0;
            ByteRoutines.SetBitOne(ref tmp, 0); // 1 подканал выбран
            ByteRoutines.SetBitZero(ref tmp, 1); // 2 подканал не выбран
            ByteRoutines.SetBitZero(ref tmp, 2); // СНКПОИ
            // ВЫБОР АНТЕННЫ В КОНСТРУКТОРЕ МПЗ
            ByteRoutines.SetBitZero(ref tmp, 4); // основной канал ЦИ СОЭН
            // СОСТОЯНИЕ СВРЛ В КОНСТРУКТОРЕ МПЗ 
            CONF_RLCI = tmp;


            /* ---------- Ntask -----------*/
            // to be filled in MPZ

            /* ---------- PWR_ON -----------*/
            PWR_ON = 0; // default, В КОНСТРУКТОРЕ МПЗ

            /* ---------- Session_key_ON -----------*/
            Session_key_ON = 0; // default, В КОНСТРУКТОРЕ МПЗ

            /* ---------- Autotune_ON -----------*/
            Autotune_ON = 0; // default, В КОНСТРУКТОРЕ МПЗ

            /* ---------- Autotune_R1 -----------*/
            Autotune_R1 = 0; // default, ЦУП

            /* ---------- Autotune_R2 -----------*/
            Autotune_R2 = 0; // default, ЦУП

            /* ---------- CONF_Test_Off -----------*/
            CONF_Test_Off = 0; // default, ЦУП

            /* ---------- ton -----------*/
            //ton to be filled in MPZ

            /* ---------- Ttask -----------*/
            //ttask to be filled in MPZ

            /* ---------- Tvideo -----------*/
            //tvideo to be filled in MPZ


            /* ---------- CONF_C / RCONF_C -----------*/
            CONF_C = new byte[2] { 0, 0 };
            RCONF_C = new byte[2] { 0, 0 };

            ByteRoutines.SetBitZero(CONF_C, 0); // разрешить включить ББЗУ
            ByteRoutines.SetBitZero(RCONF_C, 0); // разрешить включить ББЗУ

            ByteRoutines.SetOneIfTrue(CONF_C, 1, !(hasMK || hasPK));
            ByteRoutines.SetOneIfTrue(RCONF_C, 1, !(hasMK || hasPK));

            ByteRoutines.SetOneIfTrue(CONF_C, 2, !hasPK);
            ByteRoutines.SetOneIfTrue(RCONF_C, 2, !hasPK);

            ByteRoutines.SetOneIfTrue(CONF_C, 3, !hasMK);
            ByteRoutines.SetOneIfTrue(RCONF_C, 3, !hasMK);

            ByteRoutines.SetOneIfTrue(CONF_C, 4, !mainKeyBVIP);
            ByteRoutines.SetOneIfTrue(RCONF_C, 4, mainKeyBVIP); // != CONF_C

            ByteRoutines.SetBitOne(CONF_C, 5);
            ByteRoutines.SetBitOne(RCONF_C, 5);
            ByteRoutines.SetBitOne(CONF_C, 6);
            ByteRoutines.SetBitOne(RCONF_C, 6);
            ByteRoutines.SetBitOne(CONF_C, 7);
            ByteRoutines.SetBitOne(RCONF_C, 7);
            // 8, 9, 10, 11 - zero by default
            ByteRoutines.SetBitOne(CONF_C, 12);
            ByteRoutines.SetBitOne(RCONF_C, 12);
            ByteRoutines.SetBitOne(CONF_C, 13);
            ByteRoutines.SetBitOne(RCONF_C, 13);
            ByteRoutines.SetBitOne(CONF_C, 14);
            ByteRoutines.SetBitOne(RCONF_C, 14);
            ByteRoutines.SetBitOne(CONF_C, 15);
            ByteRoutines.SetBitOne(RCONF_C, 15);

            /* ---------- CONF_B / RCONF_B -----------*/
            CONF_B = new byte[2] { 0, 0 };
            RCONF_B = new byte[2] { 0, 0 };

            ByteRoutines.SetOneIfTrue(CONF_B, 0, !mainKeyBBZU);
            ByteRoutines.SetOneIfTrue(RCONF_B, 0, mainKeyBBZU); // != CONF_B

            ByteRoutines.SetOneIfTrue(CONF_B, 1, !mainKeyVIP1);
            ByteRoutines.SetOneIfTrue(RCONF_B, 1, mainKeyVIP1); // != CONF_B

            ByteRoutines.SetOneIfTrue(CONF_B, 2, useYKZU1_1);
            ByteRoutines.SetOneIfTrue(RCONF_B, 2, useYKZU1_1);

            ByteRoutines.SetOneIfTrue(CONF_B, 3, useYKZU1_1); // == D2
            ByteRoutines.SetOneIfTrue(RCONF_B, 3, useYKZU1_1); // == D2

            ByteRoutines.SetOneIfTrue(CONF_B, 4, useYKZU2_1);
            ByteRoutines.SetOneIfTrue(RCONF_B, 4, useYKZU2_1);

            ByteRoutines.SetOneIfTrue(CONF_B, 5, !useYKZU1_1); // != D2
            ByteRoutines.SetOneIfTrue(RCONF_B, 5, !useYKZU1_1); // != D2

            ByteRoutines.SetOneIfTrue(CONF_B, 6, useYKZU2_1); // == D4
            ByteRoutines.SetOneIfTrue(RCONF_B, 6, useYKZU2_1); // == D4

            ByteRoutines.SetOneIfTrue(CONF_B, 7, !useYKZU1_1); // == D5
            ByteRoutines.SetOneIfTrue(RCONF_B, 7, !useYKZU1_1); // == D5

            ByteRoutines.SetBitOne(CONF_B, 8);
            ByteRoutines.SetBitOne(RCONF_B, 8);

            ByteRoutines.SetOneIfTrue(CONF_B, 9, !mainKeyYKPD1);
            ByteRoutines.SetOneIfTrue(RCONF_B, 9, mainKeyYKPD1); // != CONF_B

            ByteRoutines.SetBitOne(CONF_B, 10);
            ByteRoutines.SetBitOne(RCONF_B, 10);

            ByteRoutines.SetOneIfTrue(CONF_B, 11, !mainKeyYKPD2);
            ByteRoutines.SetOneIfTrue(RCONF_B, 11, mainKeyYKPD2); // != CONF_B

            // 12, 13, 14, 15 -- zero by default

            /* ---------- CONF_Z / RCONF_Z -----------*/
            CONF_Z = new byte[4] { 0, 0, 0, 0 };
            RCONF_Z = new byte[4] { 0, 0, 0, 0 };
            // ПО ДЕФОЛТУ, СТАВИТ ЦУП
            ByteRoutines.SetBitOne(CONF_Z, 6);
            ByteRoutines.SetBitOne(CONF_Z, 7);
            ByteRoutines.SetBitOne(CONF_Z, 16);
            ByteRoutines.SetBitOne(CONF_Z, 17);
            ByteRoutines.SetBitOne(CONF_Z, 18);
            ByteRoutines.SetBitOne(CONF_Z, 19);
            ByteRoutines.SetBitOne(CONF_Z, 20);
            ByteRoutines.SetBitOne(CONF_Z, 21);
            ByteRoutines.SetBitOne(RCONF_Z, 6);
            ByteRoutines.SetBitOne(RCONF_Z, 7);
            ByteRoutines.SetBitOne(RCONF_Z, 16);
            ByteRoutines.SetBitOne(RCONF_Z, 17);
            ByteRoutines.SetBitOne(RCONF_Z, 18);
            ByteRoutines.SetBitOne(RCONF_Z, 19);
            ByteRoutines.SetBitOne(RCONF_Z, 20);
            ByteRoutines.SetBitOne(RCONF_Z, 21);

            /* ---------- CONF_P / RCONF_P -----------*/
            CONF_P = new byte[10] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            RCONF_P = new byte[10] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            // ПО ДЕФОЛТУ, СТАВИТ ЦУП
            ByteRoutines.SetBitOne(CONF_P, 0);
            ByteRoutines.SetBitOne(CONF_P, 1);
            ByteRoutines.SetBitOne(CONF_P, 2);
            ByteRoutines.SetBitOne(CONF_P, 4);
            ByteRoutines.SetBitOne(CONF_P, 6);
            ByteRoutines.SetBitOne(CONF_P, 7);
            CONF_P[8] = 56;
            CONF_P[9] = 56;
            ByteRoutines.SetBitOne(RCONF_P, 0);
            ByteRoutines.SetBitOne(RCONF_P, 1);
            ByteRoutines.SetBitOne(RCONF_P, 2);
            ByteRoutines.SetBitOne(RCONF_P, 4);
            ByteRoutines.SetBitOne(RCONF_P, 6);
            ByteRoutines.SetBitOne(RCONF_P, 7);
            RCONF_P[8] = 56;
            RCONF_P[9] = 56;

            /* ---------- CONF_M / RCONF_M -----------*/
            CONF_M = new byte[10] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            RCONF_M = new byte[10] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            ByteRoutines.SetBitOne(CONF_M, 0);
            ByteRoutines.SetBitOne(CONF_M, 1);
            ByteRoutines.SetBitOne(CONF_M, 2);
            ByteRoutines.SetBitOne(CONF_M, 4);
            ByteRoutines.SetBitOne(CONF_M, 6);
            ByteRoutines.SetBitOne(CONF_M, 7);
            CONF_M[8] = 56;
            CONF_M[9] = 56;
            ByteRoutines.SetBitOne(RCONF_M, 0);
            ByteRoutines.SetBitOne(RCONF_M, 1);
            ByteRoutines.SetBitOne(RCONF_M, 2);
            ByteRoutines.SetBitOne(RCONF_M, 4);
            ByteRoutines.SetBitOne(RCONF_M, 6);
            ByteRoutines.SetBitOne(RCONF_M, 7);
            RCONF_M[8] = 56;
            RCONF_M[9] = 56;

            /* ---------- CONF_F / RCONF_F -----------*/
            CONF_F = new byte[14] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            RCONF_F = new byte[14] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

            /* ---------- CodTm -----------*/
            CodTm = 7; // default, ЦУП

            /* ---------- RegTM -----------*/
            RegTM = 0; // default, ЦУП

            /* ---------- TypeTm -----------*/
            TypeTm = 0; // default, ЦУП

            /* ---------- Delta_Pasp -----------*/
            Delta_Pasp = new double[6] { 0, 0, 0, 0, 0, 0 }; // default, ЦУП

            /* ---------- Delta_Autotune -----------*/
            Delta_Autotune = 0; //default, ЦУП

            /* ---------- TitleRes -----------*/
            TitleRes = new byte[114];
            for (int i = 0; i < TitleRes.Length; ++i)
                TitleRes[i] = 0;
        }
    }

    public class RouteMPZ
    {
        public RegimeTypes RegimeType { get; set; }
        private int groupNumber;
        public DateTime startTime { get; set; }


        public int NPZ { get; set; } // 24 bit
        public int Nroute { get; set; } // 4 bit
        public byte REGka { get; set; } // 2 bit
        public Coord InitCoord { get; set; }
        public byte N_PK { get; set; } // 3 bit
        public byte Z { get; set; } // 5 bit
        public byte N_MK { get; set; } // 8 bit
        public int Ts { get; set; } // 16bit
        public int Troute { get; set; } // 16 bit
        public byte[] REGta { get; set; } // 16 bit
        public byte[] REGta_Param { get; set; } // 16 bit
        public IdFile IDFile { get; set; }
        public byte Delta_T { get; set; } // 1 byte
        public byte Hroute { get; set; } // 1 byte
        public int Target_Rate { get; set; } // 2 byte
        public byte[] K00 { get; set; } // 16 byte
        public byte[] Tune_Param { get; set; } // 64 byte
        public double W_D_MpZ { get; set; } // 4 byte
        public int Coef_tang { get; set; } // 2 byte
        public int Target_RatePK { get; set; } // 2 byte
        public int Target_RateMK { get; set; } // 2 byte
        public int Quant_InitValuePK { get; set; } // 14 bit
        public int Quant_InitValueMK { get; set; } // 14 bit
        public PolinomCoef Polinomial_Coeff { get; set; }
        public byte[] K01 { get; set; } // 16 byte
        public byte[] K10 { get; set; } // 8 byte
        public byte[] K11 { get; set; } // 8 byte
        public byte[] R00 { get; set; } // 4 byte
        public byte[] TaskRes { get; set; } // 38 byte

        private OptimalChain.RouteParams parameters;
        public OptimalChain.RouteParams Parameters { get { return parameters; } }

        public RouteMPZ(OptimalChain.RouteParams inpParameters)
            : this(IntToType(inpParameters.type))
        {
            parameters = inpParameters;
            startTime = parameters.start;

            DIOS.Common.SqlManager DBmanager = new DIOS.Common.SqlManager("Server=188.44.42.188;Database=MCCDB;user=CuksTest;password=qwer1234QWER");
            DBTables.DataFetcher fetcher = new DBTables.DataFetcher(DBmanager);

            Astronomy.TrajectoryPoint? KAbegin_ = fetcher.GetPositionSat(parameters.start);
            if (KAbegin_ == null)
            {
                throw new Exception("No trajectory data.");
            }
            Astronomy.TrajectoryPoint KAbegin = KAbegin_.Value;
            Common.GeoPoint geoBegin = SphericalGeom.Routines.IntersectOpticalAxisAndEarth(
                KAbegin, parameters.ShootingConf.roll, parameters.ShootingConf.pitch);

            /* ---------- InitCoord -----------*/
            if (REGka == 0)
            {
                InitCoord = new Coord { Bc = AstronomyMath.ToRad(geoBegin.Latitude), Lc = AstronomyMath.ToRad(geoBegin.Longitude), Hc = 0 }; // VERIFY HC

            /* ---------- Polinomial_coeff -----------*/
                if (parameters.shooting_type == 2) // коридорная
                {
                    double l1, l2, b1, b2, s1, s2, s3, duration;
                    SatelliteTrajectory.TrajectoryRoutines.GetCoridorParams(fetcher,
                        parameters.start, parameters.coridorAzimuth, parameters.coridorLength,
                        parameters.ShootingConf.roll, parameters.ShootingConf.pitch,
                        out b1, out b2, out l1, out l2, out s1, out s2, out s3, out duration);
                    Polinomial_Coeff = new PolinomCoef { L1 = l1, L2 = l2, B1 = b1, B2 = b2, S1 = s1, S2 = s2, S3 = s3, WD_K = 0 };
                    parameters.duration = duration * 1e3;
                    parameters.end = parameters.start.AddMilliseconds(parameters.duration);
                }
                else
                    Polinomial_Coeff = new PolinomCoef { L1 = 0, L2 = 0, B1 = 0, B2 = 0, S1 = 0, S2 = 0, S3 = 0, WD_K = 0 };
            }

            /* ---------- N_PK -----------*/
            double sunHeight = SatelliteTrajectory.TrajectoryRoutines.getSunHeight(fetcher, geoBegin, startTime);
            N_PK = GetNpk(RegimeType, sunHeight, parameters.albedo, parameters.ShootingConf.roll, parameters.ShootingConf.pitch);

            /* ---------- Z -----------*/
            switch(parameters.shooting_channel)
            {
                case "pk":
                    Z = 16;
                    break;
                case "mk":
                    Z = 15;
                    break;
                default:
                    Z = 31;
                    break;
            }

            /* ---------- Troute -----------*/
            switch (RegimeType)
            {
                case RegimeTypes.SI:
                    int seconds = 5;
                    Troute = seconds * 5;
                    parameters.end = parameters.start.AddSeconds(seconds);
                    parameters.duration = seconds * 1e3;
                    break;
                case RegimeTypes.ZI:
                    Troute = (int)((parameters.end - parameters.start).TotalSeconds * 5);
                    parameters.File_Size = (int)Math.Ceiling(ComputeFileSize());
                    break;
                case RegimeTypes.VI:
                    seconds = (int)(parameters.File_Size / 1024.0 * 8 + 1);
                    Troute =  seconds * 5;
                    parameters.end = parameters.start.AddSeconds(seconds);
                    parameters.duration = seconds * 1e3;
                    break;
                case RegimeTypes.NP:
                    Troute = (int)((parameters.end - parameters.start).TotalSeconds * 5);
                    parameters.File_Size = (int)Math.Ceiling(ComputeFileSize());
                    break;
                default:
                    break;
                    //throw new Exception(String.Format("Invalid route type {0}", inpParameters.type));
            }

            /* ---------- REGta -----------*/
            if (REGka == 0)
            {
                switch (parameters.shooting_type)
                {
                    case 0:
                        ByteRoutines.SetBitOne(REGta, 0); // кадровая
                        break;
                    case 1:
                        ByteRoutines.SetBitOne(REGta, 1); // стереотриплет
                        ByteRoutines.SetBitOne(REGta, 2);
                        break;
                    case 2:
                        ByteRoutines.SetBitOne(REGta, 2); // коридорная
                        break;
                    default:
                        ByteRoutines.SetBitOne(REGta, 0); // кадровая
                        break;
                }
            }

            /* ---------- REGta_Param -----------*/
            switch (groupNumber)
            {
                case 0:
                    switch (parameters.zipPK)
                    {
                        case 0: // без потерь
                            ByteRoutines.SetBitZero(REGta_Param, 0);
                            ByteRoutines.SetBitOne(REGta_Param, 1);
                            ByteRoutines.SetBitZero(REGta_Param, 2);
                            break;
                        case 1: // без сжатия
                            ByteRoutines.SetBitOne(REGta_Param, 0);
                            ByteRoutines.SetBitOne(REGta_Param, 1);
                            ByteRoutines.SetBitZero(REGta_Param, 2);
                            break;
                        default: // сжатие с потерями
                            ByteRoutines.SetBitZero(REGta_Param, 0);
                            ByteRoutines.SetBitOne(REGta_Param, 1);
                            ByteRoutines.SetBitOne(REGta_Param, 2);
                            break;
                    }
                    ByteRoutines.SetBitZero(REGta_Param, 3);
                    switch (parameters.zipMK)
                    {
                        case 0: // без потерь
                            ByteRoutines.SetBitZero(REGta_Param, 4);
                            ByteRoutines.SetBitOne(REGta_Param, 5);
                            ByteRoutines.SetBitZero(REGta_Param, 6);
                            break;
                        case 1: // без сжатия
                            ByteRoutines.SetBitOne(REGta_Param, 4);
                            ByteRoutines.SetBitOne(REGta_Param, 5);
                            ByteRoutines.SetBitZero(REGta_Param, 6);
                            break;
                        default: // сжатие с потерями
                            ByteRoutines.SetBitZero(REGta_Param, 4);
                            ByteRoutines.SetBitOne(REGta_Param, 5);
                            ByteRoutines.SetBitOne(REGta_Param, 6);
                            break;
                    }
                    // 7-9 -- нули по умолчанию


                    //switch (parameters.memoryCellMZU1)
                    //{
                    //    case 0:
                    //        ByteRoutines.SetBitZero(REGta_Param, 10);
                    //        ByteRoutines.SetBitZero(REGta_Param, 11);
                    //        break;
                    //    case 1:
                    //        ByteRoutines.SetBitOne(REGta_Param, 10);
                    //        ByteRoutines.SetBitZero(REGta_Param, 11);
                    //        break;
                    //    case 2:
                    //        ByteRoutines.SetBitZero(REGta_Param, 10);
                    //        ByteRoutines.SetBitOne(REGta_Param, 11);
                    //        break;
                    //    case 3:
                    //        ByteRoutines.SetBitOne(REGta_Param, 10);
                    //        ByteRoutines.SetBitOne(REGta_Param, 11);
                    //        break;
                    //}
                    //switch (parameters.memoryCellMZU2)
                    //{
                    //    case 0:
                    //        ByteRoutines.SetBitZero(REGta_Param, 12);
                    //        ByteRoutines.SetBitZero(REGta_Param, 13);
                    //        break;
                    //    case 1:
                    //        ByteRoutines.SetBitOne(REGta_Param, 12);
                    //        ByteRoutines.SetBitZero(REGta_Param, 13);
                    //        break;
                    //    case 2:
                    //        ByteRoutines.SetBitZero(REGta_Param, 12);
                    //        ByteRoutines.SetBitOne(REGta_Param, 13);
                    //        break;
                    //    case 3:
                    //        ByteRoutines.SetBitOne(REGta_Param, 12);
                    //        ByteRoutines.SetBitOne(REGta_Param, 13);
                    //        break;
                    //}

                    // 10-13 -- БУДУТ ЗАПОЛНЕНЫ В МПЗ НА ОСНОВЕ CONF_B
                    // 14-15 -- нули по умолчанию
                    break;
                default:
                    break;
            }

            /* ---------- IDFile -----------*/
            if (REGka == 1)
            {
                IDFile = new IdFile
                {
                    TNPZ = parameters.binded_route.Item1,
                    TNroute = parameters.binded_route.Item2 == -1 ? 15 : parameters.binded_route.Item2,
                    TNPos = 0 //parameters.TNPos == null ? 0 : parameters.TNPos
                };
            }

            /* ---------- Delta_T -----------*/
            Delta_T = 0; //(byte)(parameters.Delta_T == null ? 0 : parameters.Delta_T);

            /* ---------- Target_RatePK -----------*/
            if (parameters.zipPK >= 2)
            {
                Target_RatePK = (int)(48 * 48 * 12.0 / parameters.zipPK);
            }

            /* ---------- Target_RateMK -----------*/
            if (parameters.zipMK >= 2)
            {
                Target_RateMK = (int)(24 * 24 * 12.0 / parameters.zipMK);
            }

            /* ---------- Quant_InitValuePK -----------*/
            switch (parameters.zipPK)
            {
                case 2:
                    Quant_InitValuePK = 4000;
                    break;
                case 3:
                    Quant_InitValuePK = 2000;
                    break;
                case 4:
                case 5:
                    Quant_InitValuePK = 700;
                    break;
                case 6:
                case 7:
                    Quant_InitValuePK = 500;
                    break;
                case 8:
                case 9:
                case 10:
                    Quant_InitValuePK = 300;
                    break;
                default:
                    Quant_InitValuePK = 0;
                    break;
            }

            /* ---------- Quant_InitValueMK -----------*/
            switch (parameters.zipMK)
            {
                case 2:
                    Quant_InitValueMK = 4000;
                    break;
                case 3:
                    Quant_InitValueMK = 2000;
                    break;
                case 4:
                case 5:
                    Quant_InitValueMK = 700;
                    break;
                case 6:
                case 7:
                    Quant_InitValueMK = 500;
                    break;
                case 8:
                case 9:
                case 10:
                    Quant_InitValueMK = 300;
                    break;
                default:
                    Quant_InitValueMK = 0;
                    break;
            }
        }

        /// <summary>
        /// Sets default values.
        /// </summary>
        /// <param name="regimeType"></param>
        public RouteMPZ(RegimeTypes regimeType)
        {
            RegimeType = regimeType;

            /* ---------- NPZ -----------*/
            NPZ = -1; // to be filled in the MPZ constructor

            /* ---------- Nroute -----------*/
            // to be filled in the MPZ constructor

            /* ---------- REGka -----------*/
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

            /* ---------- InitCoord -----------*/
            switch (REGka)
            {
                case 0:
                    //InitCoord = new Coord{Bc, Lc, Hc}
                    //To be filled separately
                    break;
                case 1:
                    InitCoord = new Coord { Bc = 0, Lc = 0, Hc = 0 };
                    break;
            }

            /* ---------- N_PK -----------*/
            // отдельно

            /* ---------- Z -----------*/
            Z = 31;

            /* ---------- N_MK -----------*/
            N_MK = 0; // поле не используется, всегда 0

            /* ---------- TS -----------*/
            // в МПЗ

            /* ---------- Troute -----------*/
            // отдельно

            /* ---------- REGta -----------*/
            #region set REGta
            REGta = new byte[2] { 0, 0 };
            ByteRoutines.SetBitZero(REGta, 0); //
            ByteRoutines.SetBitZero(REGta, 1); // Штатный ЗИ и НП -- отдельно 
            ByteRoutines.SetBitZero(REGta, 2); //
            ByteRoutines.SetBitOne(REGta, 3);
            ByteRoutines.SetBitZero(REGta, 4);
            ByteRoutines.SetBitZero(REGta, 5);
            ByteRoutines.SetBitZero(REGta, 6);
            ByteRoutines.SetBitOne(REGta, 7);
            ByteRoutines.SetBitZero(REGta, 8);
            ByteRoutines.SetBitOne(REGta, 9);
            // ByteRoutines.SetBitOne(REGta, 10); задается отдельно в МПЗ, SUSPEND_MODE
            ByteRoutines.SetBitZero(REGta, 11);
            int typeRegime = 0;
            switch (regimeType)
            {
                case RegimeTypes.SI:
                case RegimeTypes.VI:
                case RegimeTypes.ZI_fok_yust:
                case RegimeTypes.KPI_unload:
                case RegimeTypes.BBZU_control:
                    ByteRoutines.SetBitZero(REGta, 12);
                    break;
                default:
                    ByteRoutines.SetBitOne(REGta, 12);
                    typeRegime += 1;
                    break;
            }
            switch (regimeType)
            {
                case RegimeTypes.VI:
                case RegimeTypes.NP:
                case RegimeTypes.ZI_fok_yust:
                case RegimeTypes.NP_fok_yust:
                case RegimeTypes.KPI_unload:
                case RegimeTypes.PUF_control:
                    ByteRoutines.SetBitOne(REGta, 13);
                    typeRegime += 2;
                    break;
                default:
                    ByteRoutines.SetBitZero(REGta, 13);
                    break;
            }
            
            switch (regimeType)
            {
                case RegimeTypes.SI:
                case RegimeTypes.ZI:
                case RegimeTypes.VI:
                case RegimeTypes.NP:
                    ByteRoutines.SetBitZero(REGta, 14);
                    ByteRoutines.SetBitZero(REGta, 15);
                    break;
                case RegimeTypes.ZI_cal:
                case RegimeTypes.ZI_fok_yust:
                case RegimeTypes.NP_fok_yust:
                    ByteRoutines.SetBitOne(REGta, 14);
                    ByteRoutines.SetBitZero(REGta, 15);
                    groupNumber = 1;
                    break;
                case RegimeTypes.KPI_load:
                case RegimeTypes.KPI_unload:
                case RegimeTypes.PUF_control:
                    ByteRoutines.SetBitZero(REGta, 14);
                    ByteRoutines.SetBitOne(REGta, 15);
                    groupNumber = 2;
                    break;
                default:
                    ByteRoutines.SetBitOne(REGta, 14);
                    ByteRoutines.SetBitOne(REGta, 15);
                    groupNumber = 3;
                    break;
            }
            #endregion

            /* ---------- REGta_Param -----------*/
            REGta_Param = new byte[2] { 0, 0 };
            
            /* ---------- IDFile -----------*/
            IDFile = new IdFile { TNPZ = 0, TNroute = 0, TNPos = 0 }; // задается отдельно

            /* ---------- Delta_T -----------*/
            Delta_T = 0; // default

            /* ---------- Hroute -----------*/
            Hroute = 200; // БОЛВАНКА -- ТРЕБУЕТ ВНИМАНИЯ!!!

            /* ---------- Target_Rate -----------*/
            Target_Rate = 0; // default

            /* ---------- K00 -----------*/
            K00 = new byte[16]; // ЦУП
            for (int i = 0; i < K00.Length; ++i)
                K00[i] = 0;

            /* ---------- Tune_Param -----------*/
            Tune_Param = new byte[64]; // болванка
            for (int i = 0; i < Tune_Param.Length; ++i)
                Tune_Param[i] = 0;

            /* ---------- W/D_MPZ -----------*/
            W_D_MpZ = 0; // default, ЦУП

            /* ---------- Coef_tang -----------*/
            Coef_tang = 0; // default

            /* ---------- Target_RatePK -----------*/
            Target_RatePK = 0; // default

            /* ---------- Target_RateMK -----------*/
            Target_RateMK = 0; // default

            /* ---------- Quant_InitValuePK -----------*/
            Quant_InitValuePK = 0; // default

            /* ---------- Quant_InitValueMK -----------*/
            Quant_InitValueMK = 0; // default

            /* ---------- Polinomial_coeff -----------*/
            // отдельно

            /* ---------- K01 -----------*/
            K01 = new byte[16] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

            /* ---------- K10 -----------*/
            K10 = new byte[8] { 0, 0, 0, 0, 0, 0, 0, 0 };

            /* ---------- K11 -----------*/
            K11 = new byte[8] { 0, 0, 0, 0, 0, 0, 0, 0 };

            /* ---------- R00 -----------*/
            R00 = new byte[4] { 0, 0, 0, 0 };

            /* ---------- TaskRes -----------*/
            TaskRes = new byte[38];
            for (int i = 0; i < TaskRes.Length; ++i)
                TaskRes[i] = 0;
        }

        private static RegimeTypes IntToType(int type)
        {
            switch (type)
            {
                case 0:
                    return RegimeTypes.ZI;
                    // return RegimeTypes.SI; так написано в OptimalChain.MPZ, но просят поменять
                case 1:
                    return RegimeTypes.VI;
                    // return RegimeTypes.ZI; так написано в OptimalChain.MPZ, но просят поменять
                case 2:
                    return RegimeTypes.SI;
                    // return RegimeTypes.VI; так написано в OptimalChain.MPZ, но просят поменять
                case 3:
                    return RegimeTypes.NP;
                    // return RegimeTypes.NP; так написано в OptimalChain.MPZ, но просят поменять
                default:
                    throw new Exception(String.Format("Invalid route type {0}", type));
            }
        }

        /// <summary>
        /// В Мб.
        /// </summary>
        /// <param name="routeParams"></param>
        /// <returns></returns>
        private double ComputeFileSize()
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

            // Чтобы не делить на 0.
            int zipmk = parameters.zipMK > 0 ? parameters.zipMK : 1;
            int zippk = parameters.zipPK > 0 ? parameters.zipPK : 1;
            return OptimalChain.RouteParams.InformationFluxInBits(
                parameters.ShootingConf.roll, parameters.ShootingConf.pitch,
                Hroute, CodVznCalibr, Nm, zipmk, Np, zippk) * (Troute * 0.2) / (1 << 23);
        }

        /// <summary>
        /// Вычисляет параметр N_PK (число шагов ВЗН).
        /// </summary>
        /// <param name="type">Режим съемки</param>
        /// <param name="sunHeight">Высота Солнца в радианах</param>
        /// <param name="albedo">Альбедо</param>
        /// <param name="roll">Крен в радианах</param>
        /// <param name="pitch">Тангаж в радианах</param>
        /// <returns></returns>
        private byte GetNpk(RegimeTypes type, double sunHeight, double albedo, double roll = 0, double pitch = 0)
        {
            if (type == RegimeTypes.SI || type == RegimeTypes.VI)
            { 
                // Согласно таблиц
                double sunDeg = AstronomyMath.ToDegrees(sunHeight);

                if (albedo < 0.2)
                {
                    return 4;
                }
                else if (albedo < 0.4)
                {
                    if (sunDeg < 40)
                        return 4;
                    else
                        return 3;
                }
                else if (albedo < 0.6)
                {
                    if (sunDeg < 30)
                        return 4;
                    else if (sunDeg < 50)
                        return 3;
                    else
                        return 2;
                }
                else if (albedo < 0.8)
                {
                    if (sunDeg < 20)
                        return 4;
                    else if (sunDeg < 40)
                        return 3;
                    else
                        return 2;
                }
                else if (albedo <= 1.0)
                {
                    if (sunDeg < 20)
                        return 4;
                    else if (sunDeg < 30)
                        return 3;
                    else if (sunDeg < 70)
                        return 2;
                    else
                        return 1;
                }
                else
                    throw new ArgumentException(String.Format("Bad parameters sunHeight = {0}, albedo = {1}", sunHeight, albedo));
            } 
            else
            {
                // По формулам из ИД.
                double vu = Math.Sin(sunHeight) * albedo / OptimalChain.RouteParams.WD(roll, pitch);
                if (vu <= 22)
                    return 4;
                else if (vu < 45)
                    return 3;
                else if (vu <= 87)
                    return 2;
                else if (vu < 178)
                    return 1;
                else
                    return 0;
            }
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

    public class PolinomCoef
    {
        public double L1 { get; set; }
        public double L2 { get; set; }
        public double B1 { get; set; }
        public double B2 { get; set; }
        public double WD_K { get; set; }
        public double S1 { get; set; }
        public double S2 { get; set; }
        public double S3 { get; set; }
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

        public static void SetOneIfTrue(byte[] data, int index, bool flag)
        {
            if (flag)
                SetBitOne(data, index);
            else
                SetBitZero(data, index);
        }

        public static void SetBitOne(byte[] data, int index)
        {
            SetBitOne(ref data[index / 8], index % 8);
        }

        public static void SetBitZero(byte[] data, int index)
        {
            SetBitZero(ref data[index / 8], index % 8);
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
