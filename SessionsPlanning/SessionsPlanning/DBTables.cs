using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using System.Data;

using Common;
using Astronomy;
using DIOS.Common;

namespace DBTables
{
    public class DataFetcher
    {
        private static GeoPoint snkpoi = new GeoPoint(30.185089, 31.690301);
        private SqlManager manager;

        public DataFetcher(SqlManager _manager)
        {
            manager = _manager;
        }

        /// <summary>
        /// Fetches space-time position of the Sun (NOT normalized to the Earth radius).
        /// </summary>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <returns></returns>
        public List<SpaceTime> GetPositionSun(/*DateTime from, DateTime to*/)
        {
            DataTable sunPositionTable = manager.GetSqlObject(
                SunTable.Name, ""
                /*String.Format("where {0} between #{1}# and #{2}#", SunTable.Time, from.ToShortDateString(), to.ToShortDateString())*/);

            List<SpaceTime> res = new List<SpaceTime>();
            DataRow[] rows = sunPositionTable.Select();
            foreach (DataRow row in rows)
                res.Add(new SpaceTime { Position = SunTable.GetPosition(row), Time = SunTable.GetTime(row) });

            return res;
        }

        /// <summary>
        /// In units of Earth radius.
        /// </summary>
        /// <returns></returns>
        public Vector3D GetPositionUnitSNKPOI()
        {
            return GeoPoint.ToCartesian(snkpoi, 1);
        }
        public Vector3D GetPositionSNKPOI()
        {
            return GeoPoint.ToCartesian(snkpoi, Astronomy.Constants.EarthRadius);
        }
        /// <summary>
        /// As lat-lon.
        /// </summary>
        /// <returns></returns>
        public GeoPoint GetPositionGeoSNKPOI()
        {
            return snkpoi;
        }

        public List<PositionMNKPOI> GetPositionMNKPOI()
        {
            DataTable mnkpoiPositionTable = manager.GetSqlObject(
                MnkpoiTable.Name, "");

            List<PositionMNKPOI> res = new List<PositionMNKPOI>();
            DataRow[] rows = mnkpoiPositionTable.Select();
            foreach (DataRow row in rows)
                res.Add(new PositionMNKPOI
                {
                    Number = MnkpoiTable.GetNum(row),
                    Position = MnkpoiTable.GetPositionGeo(row),
                    Altitude = MnkpoiTable.GetAlt(row),
                    TimeBeg = MnkpoiTable.GetTimeFrom(row),
                    TimeEnd = MnkpoiTable.GetTimeTo(row)
                });

            return res;
        }

        /// <summary>
        /// Fetches space-time position of the satellite (NOT normalized to the Earth radius).
        /// </summary>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <returns></returns>
        public List<SpaceTime> GetPositionSat(DateTime from, DateTime to)
        {
            DataTable satPositionTable = manager.GetSqlObject(
                SunTable.Name, 
                String.Format("where {0} between #{1}# and #{2}#", SunTable.Time, from.ToShortDateString(), to.ToShortDateString()));

            List<SpaceTime> res = new List<SpaceTime>();
            DataRow[] rows = satPositionTable.Select();
            foreach (DataRow row in rows)
                res.Add(new SpaceTime { Position = SatTable.GetPosition(row), Time = SatTable.GetTime(row) });

            return res;
        }
    }

    public static class SunTable
    {
        public const string Name = "TblSunPosition";
        public const string Id = "ID";
        public const string Time = "TM";
        public const string Lat = "F_SUN";
        public const string Lon = "L_SUN";
        public const string Rad = "R_SUN";
        public const string WriteTime = "WRT_TM";

        public static int GetId(DataRow row)
        {
            return (int)row[Id];
        }

        public static DateTime GetTime(DataRow row)
        {
            return (DateTime)row[Time];
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLat(DataRow row)
        {
            return (double)row[Lat];
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLon(DataRow row)
        {
            return (double)row[Lon];
        }

        public static double GetRad(DataRow row)
        {
            return (double)row[Rad];
        }

        public static GeoPoint GetPositionGeo(DataRow row)
        {
            return new GeoPoint(GetLat(row), GetLon(row));
        }

        public static Vector3D GetPosition(DataRow row)
        {
            return GeoPoint.ToCartesian(GetPositionGeo(row), GetRad(row));
        }

        public static Vector3D GetPositionUnitEarth(DataRow row)
        {
            return GeoPoint.ToCartesian(GetPositionGeo(row), GetRad(row) / Astronomy.Constants.EarthRadius);
        }

        public static DateTime GetWriteTime(DataRow row)
        {
            return (DateTime)row[WriteTime];
        }
    }

    public static class SatTable
    {
        public const string Name = "TblKAPosition";
        public const string Id = "ID";
        public const string Num = "N_KA";
        public const string Time = "TM";
        public const string Lat = "FI_SSPOINT";
        public const string Lon = "LAM_SSPOINT";
        public const string Rad = "R";
        public const string WriteTime = "WRT_TM";

        public static int GetId(DataRow row)
        {
            return (int)row[Id];
        }

        public static int GetNum(DataRow row)
        {
            return (int)row[Num];
        }

        public static DateTime GetTime(DataRow row)
        {
            return (DateTime)row[Time];
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLat(DataRow row)
        {
            return (double)row[Lat];
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLon(DataRow row)
        {
            return (double)row[Lon];
        }

        public static double GetRad(DataRow row)
        {
            return (double)row[Rad];
        }

        public static GeoPoint GetPositionGeo(DataRow row)
        {
            return new GeoPoint(GetLat(row), GetLon(row));
        }

        public static Vector3D GetPosition(DataRow row)
        {
            return GeoPoint.ToCartesian(GetPositionGeo(row), GetRad(row));
        }

        public static Vector3D GetPositionUnitEarth(DataRow row)
        {
            return GeoPoint.ToCartesian(GetPositionGeo(row), GetRad(row) / Astronomy.Constants.EarthRadius);
        }

        public static DateTime GetWriteTime(DataRow row)
        {
            return (DateTime)row[WriteTime];
        }
    }

    public static class OrbitTable
    {
        public const string Name = "TblOrbit";
        public const string Id = "ID";
        public const string Num = "N_KA";
        public const string NumTurn = "N_VIT";
        public const string NumTurnDaily = "N_VIT_S";
        public const string TimeEquator = "T_EQUATOR";
        public const string TimeEquatorSec = "T_EQUATOR_SEC";
        public const string AscendingNodeLon = "LONGITUDE";
        public const string SunAngle = "SUN_ANGLE";
        public const string MajorSemiaxis = "MAJOR_SEMIAXIS";
        public const string InertialLon = "INRT_LONGITUDE";
        public const string Inclination = "INCLINATION";
        public const string Eccentricity = "ECCENTRICITY";
        public const string PericentreArg = "PRM_PERICENTRE";
        public const string LatitudeArg = "PRM_LATITUDE";
        public const string WriteTime = "WRT_TM";

        public static int GetId(DataRow row)
        {
            return (int)row[Id];
        }

        public static int GetNum(DataRow row)
        {
            return (int)row[Num];
        }

        public static int GetNumTurn(DataRow row)
        {
            return (int)row[NumTurn];
        }

        public static int GetNumTurnDaily(DataRow row)
        {
            return (int)row[NumTurnDaily];
        }

        public static DateTime GetTimeEquator(DataRow row)
        {
            return (DateTime)row[TimeEquator];
        }

        public static double GetTimeEquatorSec(DataRow row)
        {
            return (double)row[TimeEquatorSec];
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetAscendingNodeLon(DataRow row)
        {
            return (double)row[AscendingNodeLon];
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetSunAngle(DataRow row)
        {
            return (double)row[SunAngle];
        }

        public static double GetMajorSemiaxis(DataRow row)
        {
            return (double)row[MajorSemiaxis];
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetInertialLon(DataRow row)
        {
            return (double)row[InertialLon];
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetInclination(DataRow row)
        {
            return (double)row[Inclination];
        }

        public static double GetEccentricity(DataRow row)
        {
            return (double)row[Eccentricity];
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetPericentreArg(DataRow row)
        {
            return (double)row[PericentreArg];
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLatitudeArg(DataRow row)
        {
            return (double)row[LatitudeArg];
        }

        public static DateTime GetWriteTime(DataRow row)
        {
            return (DateTime)row[WriteTime];
        }
    }

    public static class CoverageTable
    {
        public const string Name = "TblCoverage";
        public const string Id = "ID";
        public const string Num = "N_KA";
        public const string NadirTime = "TM";
        public const string NadirLat = "FI_SSPOINT";
        public const string NadirLon = "LAM_SSPOINT";
        public const string LeftLat = "FI_LSSP";
        public const string LeftLon = "LAM_LSSP";
        public const string RightLat = "FI_RSSP";
        public const string RightLon = "LAM_RSSP";
        public const string WriteTime = "WRT_TM";

        public static int GetId(DataRow row)
        {
            return (int)row[Id];
        }

        public static int GetNum(DataRow row)
        {
            return (int)row[Num];
        }

        public static DateTime GetNadirTime(DataRow row)
        {
            return (DateTime)row[NadirTime];
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetNadirLat(DataRow row)
        {
            return (double)row[NadirLat];
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetNadirLon(DataRow row)
        {
            return (double)row[NadirLon];
        }

        public static GeoPoint GetNadirGeo(DataRow row)
        {
            return new GeoPoint(GetNadirLat(row), GetNadirLon(row));
        }

        public static Vector3D GetNadirUnit(DataRow row)
        {
            return GeoPoint.ToCartesian(GetNadirGeo(row), 1);
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLeftLat(DataRow row)
        {
            return (double)row[LeftLat];
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLeftLon(DataRow row)
        {
            return (double)row[LeftLon];
        }

        public static GeoPoint GetLeftGeo(DataRow row)
        {
            return new GeoPoint(GetLeftLat(row), GetLeftLon(row));
        }

        public static Vector3D GetLeftUnit(DataRow row)
        {
            return GeoPoint.ToCartesian(GetLeftGeo(row), 1);
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetRightLat(DataRow row)
        {
            return (double)row[RightLat];
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetRightLon(DataRow row)
        {
            return (double)row[RightLon];
        }

        public static GeoPoint GetRightGeo(DataRow row)
        {
            return new GeoPoint(GetRightLat(row), GetRightLon(row));
        }

        public static Vector3D GetRightUnit(DataRow row)
        {
            return GeoPoint.ToCartesian(GetRightGeo(row), 1);
        }

        public static SatelliteTrajectory.LanePos GetLanePos(DataRow row)
        {
            return new SatelliteTrajectory.LanePos(GetLeftUnit(row), GetNadirUnit(row), GetRightUnit(row), GetNadirTime(row));
        }
    }

    public static class MnkpoiTable
    {
        public const string Name = "BNO_MNKPOI";
        public const string Id = "ID";
        public const string Num = "NUM";
        public const string Lat = "LATITUDE";
        public const string Lon = "LONGITUDE";
        public const string Alt = "ALTITUDE";
        public const string TimeFrom = "TMBEG";
        public const string TimeTo = "TMEND";
        public const string WriteTime = "WRT_TM";
        public const string Sent = "SENT";

        public static int GetId(DataRow row)
        {
            return (int)row[Id];
        }

        public static int GetNum(DataRow row)
        {
            return (int)row[Num];
        }

        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLat(DataRow row)
        {
            return (double)row[Lat];
        }
        /// <summary>
        /// In degrees.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLon(DataRow row)
        {
            return (double)row[Lon];
        }

        public static double GetAlt(DataRow row)
        {
            return (double)row[Alt];
        }

        public static GeoPoint GetPositionGeo(DataRow row)
        {
            return new GeoPoint(GetLat(row), GetLon(row));
        }

        public static DateTime GetTimeFrom(DataRow row)
        {
            return (DateTime)row[TimeFrom];
        }

        public static DateTime GetTimeTo(DataRow row)
        {
            return (DateTime)row[TimeTo];
        }

        public static DateTime GetWriteTime(DataRow row)
        {
            return (DateTime)row[WriteTime];
        }

        public static int GetSent(DataRow row)
        {
            return (int)row[Sent];
        }
    }

    public class SpaceTime
    {
        public Vector3D Position { get; set; }
        public DateTime Time { get; set; }
    }

    public class PositionMNKPOI
    {
        public int Number { get; set; }
        public GeoPoint Position { get; set; }
        /// <summary>
        /// Altitude in metres.
        /// </summary>
        public double Altitude { get; set; }
        public DateTime TimeBeg { get; set; }
        public DateTime TimeEnd { get; set; }
    }
}
