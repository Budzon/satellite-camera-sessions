using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using System.Data;

using Common;
using Astronomy;

namespace DBTables
{
    public class SunTable
    {
        public static const string Name = "TblSunPosition";
        public static const string Id = "ID";
        public static const string Time = "TM";
        public static const string Lat = "F_SUN";
        public static const string Lon = "L_SUN";
        public static const string Rad = "R_SUN";
        public static const string WriteTime = "WRT_TM";

        public static int GetId(DataRow row)
        {
            return (int)row[Id];
        }

        public static DateTime GetTime(DataRow row)
        {
            return (DateTime)row[Time];
        }
        /// <summary>
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLat(DataRow row)
        {
            return Astronomy.AstronomyMath.ToRad((double)row[Lat]);
        }
        /// <summary>
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLon(DataRow row)
        {
            return Astronomy.AstronomyMath.ToRad((double)row[Lon]);
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

    public class SatTable
    {
        public static const string Name = "TblKAPosition";
        public static const string Id = "ID";
        public static const string Num = "N_KA";
        public static const string Time = "TM";
        public static const string Lat = "FI_SSPOINT";
        public static const string Lon = "LAM_SSPOINT";
        public static const string Rad = "R";
        public static const string WriteTime = "WRT_TM";

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
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLat(DataRow row)
        {
            return Astronomy.AstronomyMath.ToRad((double)row[Lat]);
        }
        /// <summary>
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLon(DataRow row)
        {
            return Astronomy.AstronomyMath.ToRad((double)row[Lon]);
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

    public class OrbitTable
    {
        public static const string Name = "TblOrbit";
        public static const string Id = "ID";
        public static const string Num = "N_KA";
        public static const string NumTurn = "N_VIT";
        public static const string NumTurnDaily = "N_VIT_S";
        public static const string TimeEquator = "T_EQUATOR";
        public static const string TimeEquatorSec = "T_EQUATOR_SEC";
        public static const string AscendingNodeLon = "LONGITUDE";
        public static const string SunAngle = "SUN_ANGLE";
        public static const string MajorSemiaxis = "MAJOR_SEMIAXIS";
        public static const string InertialLon = "INRT_LONGITUDE";
        public static const string Inclination = "INCLINATION";
        public static const string Eccentricity = "ECCENTRICITY";
        public static const string PericentreArg = "PRM_PERICENTRE";
        public static const string LatitudeArg = "PRM_LATITUDE";
        public static const string WriteTime = "WRT_TM";

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
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetAscendingNodeLon(DataRow row)
        {
            return AstronomyMath.ToRad((double)row[AscendingNodeLon]);
        }
        /// <summary>
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetSunAngle(DataRow row)
        {
            return AstronomyMath.ToRad((double)row[SunAngle]);
        }

        public static double GetMajorSemiaxis(DataRow row)
        {
            return (double)row[MajorSemiaxis];
        }
        /// <summary>
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetInertialLon(DataRow row)
        {
            return AstronomyMath.ToRad((double)row[InertialLon]);
        }
        /// <summary>
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetInclination(DataRow row)
        {
            return AstronomyMath.ToRad((double)row[Inclination]);
        }

        public static double GetEccentricity(DataRow row)
        {
            return (double)row[Eccentricity];
        }
        /// <summary>
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetPericentreArg(DataRow row)
        {
            return AstronomyMath.ToRad((double)row[PericentreArg]);
        }
        /// <summary>
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLatitudeArg(DataRow row)
        {
            return AstronomyMath.ToRad((double)row[LatitudeArg]);
        }

        public static DateTime GetWriteTime(DataRow row)
        {
            return (DateTime)row[WriteTime];
        }
    }

    public class CoverageTable
    {
        public static const string Name = "TblCoverage";
        public static const string Id = "ID";
        public static const string Num = "N_KA";
        public static const string NadirTime = "TM";
        public static const string NadirLat = "FI_SSPOINT";
        public static const string NadirLon = "LAM_SSPOINT";
        public static const string LeftLat = "FI_LSSP";
        public static const string LeftLon = "LAM_LSSP";
        public static const string RightLat = "FI_RSSP";
        public static const string RightLon = "LAM_RSSP";
        public static const string WriteTime = "WRT_TM";

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
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetNadirLat(DataRow row)
        {
            return AstronomyMath.ToRad((double)row[NadirLat]);
        }
        /// <summary>
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetNadirLon(DataRow row)
        {
            return AstronomyMath.ToRad((double)row[NadirLon]);
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
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLeftLat(DataRow row)
        {
            return AstronomyMath.ToRad((double)row[LeftLat]);
        }
        /// <summary>
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetLeftLon(DataRow row)
        {
            return AstronomyMath.ToRad((double)row[LeftLon]);
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
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetRightLat(DataRow row)
        {
            return AstronomyMath.ToRad((double)row[RightLat]);
        }
        /// <summary>
        /// In radians.
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public static double GetRightLon(DataRow row)
        {
            return AstronomyMath.ToRad((double)row[RightLon]);
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
}
