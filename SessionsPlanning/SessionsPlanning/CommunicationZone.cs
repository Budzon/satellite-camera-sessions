using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using Astronomy;
using Common;
using DBTables;

namespace SatelliteSessions
{

    public abstract class CommunicationZone
    {
        /// <summary>
        /// Id
        /// </summary>
        public virtual int IdNumber { get; set; }
        public abstract List<SessionsPlanning.CommunicationSessionStation> Stations { get; }
        /// <summary>
        /// In degrees.
        /// </summary>
        public double CentreLat;
        /// <summary>
        /// In degrees.
        /// </summary>
        public double CentreLon;
        /// <summary>
        /// In km.
        /// </summary>
        public double Radius5;
        /// <summary>
        /// In km.
        /// </summary>
        public double Radius7;

        public bool isPointInZone(Point3D checkPoint, double zoneRadius)
        {
            double angleZone = Math.Asin(zoneRadius / (OptimalChain.Constants.orbit_height + Astronomy.Constants.EarthRadius));
            Vector3D centre = GeoPoint.ToCartesian(new GeoPoint(CentreLat, CentreLon), 1);
            double angleKA = AstronomyMath.ToRad(Vector3D.AngleBetween(checkPoint.ToVector(), centre));
            return angleKA <= angleZone;
        }


        /// <summary>
        /// получить радиус зоны в километрах
        /// </summary>
        /// <param name="R">Planet radius [km]</param>
        /// <param name="h">Orbit height [km]</param>
        /// <param name="d">Altitude [km]</param>
        /// <param name="a">Zone angle [deg]</param>
        /// <returns>радиус зоны в километрах</returns>
        protected static double ZoneRadius(double R, double h, double d, double a)
        {
            double c = Math.Cos(AstronomyMath.ToRad(a));
            double s = Math.Sin(AstronomyMath.ToRad(a));
            return -(R + d) * c * s + c * Math.Sqrt((R + d) * (R + d) * s * s + (2 * R + h + d) * (h - d));
        }


        /// <summary>
        /// Вычисление зон связи МНКПОИ за заданный промежуток времени.
        /// </summary>
        /// <param name="DBManager">Параметры подключения к БД</param>
        /// <param name="timeFrom">Начало временного промежутка</param>
        /// <param name="timeTo">Конец временного промежутка</param>
        /// <param name="zones">Зоны связи</param>
        public static void getMNKPOICommunicationZones(DIOS.Common.SqlManager DBManager, DateTime timeFrom, DateTime timeTo, out List<CommunicationZoneMNKPOI> zones)
        {
            DataFetcher fetcher = new DataFetcher(DBManager);
            List<PositionMNKPOI> positions = new List<PositionMNKPOI>();

            // Check if timeFrom is covered by a work interval that started strictly before.
            System.Data.DataRow[] prePosRow = fetcher.GetDataBeforeDate(MnkpoiTable.Name, MnkpoiTable.TimeFrom, timeFrom, 1);
            if (prePosRow.Length > 0)
            {
                PositionMNKPOI prePos = MnkpoiTable.GetDataMNKPOI(prePosRow[0]);
                if (prePos.TimeEnd > timeFrom)
                    positions.Add(prePos);
            }
            // Add work intervals such that timeFrom <= timeBegin < timeTo.
            positions.AddRange(fetcher.GetPositionMNKPOI(timeFrom, timeTo));

            double R = Astronomy.Constants.EarthRadius;
            double h = fetcher.GetTrajectorySat(timeFrom, timeTo).Select(point => point.Position.ToVector().Length - R).Average();

            zones = new List<CommunicationZoneMNKPOI>();
            foreach (PositionMNKPOI pos in positions)
            {
                double d = pos.Altitude / 1000; // altitude
                zones.Add(new CommunicationZoneMNKPOI
                {
                    CentreLat = pos.Position.Latitude,
                    CentreLon = pos.Position.Longitude,
                    IdNumber = pos.Number,
                    Radius5 = ZoneRadius(R, h, d, 5),
                    Radius7 = ZoneRadius(R, h, d, 7),
                    From = timeFrom < pos.TimeBeg ? pos.TimeBeg : timeFrom,
                    To = timeTo < pos.TimeEnd ? timeTo : pos.TimeEnd
                });
            }
        }


        /// <summary>
        /// Вычисление зоны связи СНКПОИ.
        /// </summary>
        /// <param name="DBManager">Параметры подключения к БД</param>
        /// <param name="zone">Зона связи</param>
        public static void getSNKPOICommunicationZones(DIOS.Common.SqlManager DBManager, out CommunicationZoneSNKPOI zone)
        {
            DataFetcher fetcher = new DataFetcher(DBManager);

            double R = Astronomy.Constants.EarthRadius;
            double h = OptimalChain.Constants.orbit_height; // okay to take as a constant?
            double d = 0; // altitude ?

            GeoPoint snkpoi = fetcher.GetPositionGeoSNKPOI();

            zone = new CommunicationZoneSNKPOI
            {
                CentreLat = snkpoi.Latitude,
                CentreLon = snkpoi.Longitude,
                Radius5 = ZoneRadius(R, h, d, 5),
                Radius7 = ZoneRadius(R, h, d, 7)
            };
        }

        /// <summary>
        /// Вычисление зон связи СНКПОИ и МНКПОИ в заданный момент времени.
        /// </summary>
        /// <param name="DBManager">Параметры подключения к БД</param>
        /// <param name="time">Интересующий момент времени</param>
        /// <param name="snkpoi">Зона связи СНКПОИ</param>
        /// <param name="mnkpoi">Зона связи МНКПОИ</param>
        public static void getCommunicationZones(string connectStr, DateTime time, out CommunicationZoneSNKPOI snkpoi, out CommunicationZoneMNKPOI mnkpoi)
        {
            DIOS.Common.SqlManager DBManager = new DIOS.Common.SqlManager(connectStr);
            DataFetcher fetcher = new DataFetcher(DBManager);
            getSNKPOICommunicationZones(DBManager, out snkpoi);

            PositionMNKPOI mnkpoiPos = fetcher.GetPositionMNKPOI(time);
            if (mnkpoiPos == null)
            {
                mnkpoi = null;
            }
            else
            {
                double R = Astronomy.Constants.EarthRadius;
                double h = OptimalChain.Constants.orbit_height;
                double d = mnkpoiPos.Altitude / 1000;
                mnkpoi = new CommunicationZoneMNKPOI
                {
                    CentreLat = mnkpoiPos.Position.Latitude,
                    CentreLon = mnkpoiPos.Position.Longitude,
                    IdNumber = mnkpoiPos.Number,
                    Radius5 = ZoneRadius(R, h, d, 5),
                    Radius7 = ZoneRadius(R, h, d, 7),
                    From = time,
                    To = time
                };
            }
        }

    }


    public class CommunicationZoneMNKPOI : CommunicationZone
    {
        public DateTime From;
        public DateTime To;
        public override List<SessionsPlanning.CommunicationSessionStation> Stations
        {
            get
            {
                return new List<SessionsPlanning.CommunicationSessionStation>() 
                {
                    SessionsPlanning.CommunicationSessionStation.MIGS
                };
            }
        }
    }

    public class CommunicationZoneSNKPOI : CommunicationZone
    {
        public override int IdNumber { get { return -1; } }

        public override List<SessionsPlanning.CommunicationSessionStation> Stations 
        {
            get 
            {
                return new List<SessionsPlanning.CommunicationSessionStation>() 
                {
                    SessionsPlanning.CommunicationSessionStation.FIGS_Main,
                    SessionsPlanning.CommunicationSessionStation.FIGS_Backup
                }; 
            } 
        }
    }

}
