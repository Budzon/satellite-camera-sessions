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
        private const string datePattern = "MM.dd.yyyy HH:mm:ss";
        //private static TimeSpan sunStep = new TimeSpan(0, 1, 1);
        //private static TimeSpan oneTurn = new TimeSpan(0, 100, 0);
        private static TimeSpan chunkLength = new TimeSpan(3, 0, 0, 0);
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
        public List<SpaceTime> GetPositionSun(DateTime from, DateTime to)
        {
            List<SpaceTime> res = new List<SpaceTime>();
            foreach (DataChunk chunk in GetDataBetweenDatesInChunks(SunTable.Name, SunTable.Time, from, to, true))
                foreach (DataRow row in chunk.Rows)
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

        /// <summary>
        /// Fetches MNKPOI position such that <paramref name="time"/> lies in [timeBegin, timeEnd).
        /// </summary>
        /// <param name="time"></param>
        /// <returns></returns>
        public PositionMNKPOI GetPositionMNKPOI(DateTime time)
        {
            System.Data.DataRow[] prePosRow = GetDataBeforeEqualDate(MnkpoiTable.Name, MnkpoiTable.TimeFrom, time, 1);
            if (prePosRow.Length > 0)
            {
                PositionMNKPOI prePos = MnkpoiTable.GetDataMNKPOI(prePosRow[0]);
                if (prePos.TimeEnd > time)
                    return prePos;
            }
            return null;
        }

        /// <summary>
        /// Fetches MNKPOI positions whose timeBegin lies in the interval [from, to).
        /// </summary>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <returns></returns>
        public List<PositionMNKPOI> GetPositionMNKPOI(DateTime from, DateTime to)
        {
            List<PositionMNKPOI> res = new List<PositionMNKPOI>();
            foreach (DataChunk chunk in GetDataBetweenDatesInChunks(MnkpoiTable.Name, MnkpoiTable.TimeFrom, from, to, false))
                foreach (DataRow row in chunk.Rows)
                    res.Add(MnkpoiTable.GetDataMNKPOI(row));

            return res;
        }

        /// <summary>
        /// Fetches space-time position of the satellite (NOT normalized to the Earth radius).
        /// </summary>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <returns></returns>
        public List<SpaceTime> GetPositions<TableClass>(DateTime from, DateTime to) where TableClass : BallisticTableFacade, new()
        {
            TableClass table = new TableClass();
            List<SpaceTime> res = new List<SpaceTime>();
            foreach (DataChunk chunk in GetDataBetweenDatesInChunks(table.Name, table.Time, from, to, true))
                foreach (DataRow row in chunk.Rows)
                    res.Add(new SpaceTime { Position = table.GetPosition(row), Time = table.GetTime(row) });

            if (res.Count == 0)
            {
                if ((to - from).TotalSeconds > OptimalChain.Constants.minTrajectoryPassInterval)
                    throw new ArgumentException("Not enough ballistic data from " + from.ToString() + " to " + to.ToString());
                else
                    return res;
            }

            double maxDist = 0;
            for (int i = 0; i < res.Count - 1; i++)
                maxDist = Math.Max(maxDist, (res[i + 1].Time - res[i].Time).TotalSeconds);
            maxDist = Math.Max(maxDist, (res.First().Time - from).TotalSeconds);
            maxDist = Math.Max(maxDist, (to - res.Last().Time).TotalSeconds);

            if (maxDist > OptimalChain.Constants.minTrajectoryPassInterval)
                throw new ArgumentException("Not enough ballistic data from " + from.ToString() + " to " + to.ToString());

            return res;
        }

        // Интерполяция минимально достаточного кол-ва точкек на даты  fromDT toDT, нужно когда на эти даты точек слишком мало.
        private List<SpaceTime> GetMinimumPointsArray<TableClass>(DateTime fromDT, DateTime toDT) where TableClass : BallisticTableFacade, new()
        {
            TableClass table = new TableClass();
            int minNumPoints = Trajectory.minNumPoints;
            string fromDTStr = fromDT.ToString(datePattern);
            string toDTStr = toDT.ToString(datePattern);

            List<SpaceTime> positions;

            positions = GetPositions<TableClass>(fromDT, toDT);

            int halfMissNum = (minNumPoints - positions.Count + 1) / 2;

            DataRow[] beforePos = GetDataBeforeDate(table.Name, table.Time, fromDT, halfMissNum);
            DataRow[] afterPos;

            afterPos = GetDataAfterDate(table.Name, table.Time, toDT, halfMissNum);

            if (beforePos.Length < halfMissNum)
                afterPos = GetDataAfterDate(table.Name, table.Time, toDT, minNumPoints - positions.Count - beforePos.Length);

            if (afterPos.Length < halfMissNum)
                beforePos = GetDataBeforeDate(table.Name, table.Time, fromDT, minNumPoints - positions.Count - afterPos.Length);

            List<SpaceTime> beforePosList = beforePos.Reverse().Select(row => new SpaceTime { Position = table.GetPosition(row), Time = table.GetTime(row) }).ToList();
            List<SpaceTime> afterPosList = afterPos.Select(row => new SpaceTime { Position = table.GetPosition(row), Time = table.GetTime(row) }).ToList();

            // проверим, что диапазоны справа и слева (если они есть) не отстоят от запрашиваемой даты слишком далеко.
            if (beforePosList.Count != 0)
            {
                DateTime nearestBeforeTime = beforePosList.Last().Time;
                if ((fromDT - nearestBeforeTime).TotalSeconds > OptimalChain.Constants.minTrajectoryPassInterval)
                    throw new ArgumentException("Not enough ballistic data for a given time period");
            }

            if (afterPosList.Count != 0)
            {
                DateTime nearestAfterTime = afterPosList.First().Time;
                if ((nearestAfterTime - toDT).TotalSeconds > OptimalChain.Constants.minTrajectoryPassInterval)
                    throw new ArgumentException("Not enough ballistic data for a given time period");
            }

            positions.InsertRange(0, beforePosList);
            positions.AddRange(afterPosList);

            if (positions.Count < minNumPoints)
                throw new ArgumentException("Not enough ballistic data for a given time period");

            // найдем самый большой перерыв по времени между точками траектории. 
            double maxDist = 0;
            for (int i = 0; i < positions.Count - 1; i++)
                maxDist = Math.Max(maxDist, (positions[i + 1].Time - positions[i].Time).TotalSeconds);

            if (maxDist > OptimalChain.Constants.minTrajectoryPassInterval)
                throw new ArgumentException("Not enough ballistic data for a given time period");

            return positions;
        }

        /// <summary>
        /// Получить из БД массив точек траектории в диапазоне времени, для которого нет нужного колва точек 
        /// </summary>
        /// <param name="fromDT"> начало диапазона </param>
        /// <param name="toDT"> конец диапазона </param>
        /// <returns> список точек</returns>
        private List<SpaceTime> IncreasePointsNumber(DateTime fromDT, DateTime toDT)
        {
            int minNumPoints = Trajectory.minNumPoints;
            List<SpaceTime> positions = GetMinimumPointsArray<SatTableFacade>(fromDT, toDT);
            TrajectoryPoint[] trajectoryPoints = positions.Select(pos => new TrajectoryPoint(pos.Time, pos.Position.ToPoint(), new Vector3D(0, 0, 0))).ToArray();
            Trajectory trajectory = Trajectory.Create(trajectoryPoints);

            long timeStep = (toDT - fromDT).Ticks / (minNumPoints - 1);

            SpaceTime[] resPoints = new SpaceTime[minNumPoints];
            for (int i = 0; i < minNumPoints; i++)
            {
                DateTime dt = fromDT.AddTicks(i * timeStep);
                resPoints[i] = new SpaceTime() { Position = trajectory.GetPosition(dt).ToVector(), Time = dt };
            }
            return resPoints.ToList();
        }


        private TrajectoryPoint? GetSinglePoint<TableClass>(DateTime dtime, double minStep) where TableClass : BallisticTableFacade, new()
        {
            TableClass table = new TableClass();
            string dtimestr = dtime.ToString(datePattern);
            Trajectory trajectory = SpaceTime.createTrajectory(GetMinimumPointsArray<TableClass>(dtime, dtime), minStep);
            return trajectory.GetPoint(dtime);
        }


        /// <summary>
        /// Получить одну точку траектории, интерполировав её на основе соседних значений.
        /// </summary>
        /// <typeparam name="TableClass"></typeparam>
        /// <param name="dtime"></param>
        /// <returns></returns>
        public TrajectoryPoint? GetSingleSatPoint(DateTime dtime, double minStep = OptimalChain.Constants.minTrajectoryStep)
        {
            return GetSinglePoint<SatTableFacade>(dtime, minStep);
        }


        /// <summary>
        /// Получить одну точку траектории, интерполировав её на основе соседних значений.
        /// </summary>
        /// <typeparam name="TableClass"></typeparam>
        /// <param name="dtime"></param>
        /// <returns></returns>
        public TrajectoryPoint? GetSingleSunPoint(DateTime dtime, double minStep = OptimalChain.Constants.minSunTrajectoryStep)
        {
            return GetSinglePoint<SunTableFacade>(dtime, minStep);
        }

        private Trajectory GetTrajectory<TableClass>(
            DateTime from,
            DateTime to,
            double minStep)
        where TableClass : BallisticTableFacade, new()
        {
            List<SpaceTime> preTrajectory = GetPositions<TableClass>(from, to);

            if (preTrajectory.Count < Trajectory.minNumPoints)
                preTrajectory = IncreasePointsNumber(from, to);
            else
            {
                DateTime lastTime = preTrajectory.Last().Time;
                if ((to - lastTime).TotalSeconds > OptimalChain.Constants.minTrajectoryPassInterval)
                    throw new ArgumentException("Not enough ballistic data from " + from.ToString() + " to " + to.ToString());

                DateTime firstTime = preTrajectory.First().Time;
                if ((firstTime - from).TotalSeconds > OptimalChain.Constants.minTrajectoryPassInterval)
                    throw new ArgumentException("Not enough ballistic data from " + from.ToString() + " to " + to.ToString());

                if (preTrajectory[0].Time != from) // если время первой точки не совпадает с from, то получим точку для from интерполяцией
                {
                    TrajectoryPoint? firstPoint = GetSinglePoint<TableClass>(from, minStep);
                    if (firstPoint != null)
                    {
                        TrajectoryPoint trajPoint = (TrajectoryPoint)firstPoint;
                        SpaceTime spt = new SpaceTime() { Position = trajPoint.Position.ToVector(), Time = from };
                        preTrajectory.Insert(0, spt);
                    }
                }

                if (preTrajectory.Last().Time != to)  // если время последней точки не совпадает с to, то получим точку для to интерполяцией
                {
                    TrajectoryPoint? secondPoint = GetSinglePoint<TableClass>(to, minStep);
                    if (secondPoint != null)
                    {
                        TrajectoryPoint trajPoint = (TrajectoryPoint)secondPoint;
                        SpaceTime spt = new SpaceTime() { Position = trajPoint.Position.ToVector(), Time = to };
                        preTrajectory.Add(spt);
                    }
                }
            }

            return SpaceTime.createTrajectory(preTrajectory, minStep);
        }


        private Trajectory GetTrajectorySimple(
           DateTime from,
           DateTime to,
           double minStep)
        {
            List<SpaceTime> preTrajectory = GetPositions<SatTableFacade>(from, to);

            DateTime lastTime = preTrajectory.Last().Time;
            if ((to - lastTime).TotalSeconds > OptimalChain.Constants.minTrajectoryPassInterval)
                throw new ArgumentException("Not enough ballistic data from " + from.ToString() + " to " + to.ToString());

            DateTime firstTime = preTrajectory.First().Time;
            if ((firstTime - from).TotalSeconds > OptimalChain.Constants.minTrajectoryPassInterval)
                throw new ArgumentException("Not enough ballistic data from " + from.ToString() + " to " + to.ToString());

            if (preTrajectory[0].Time != from) // если время первой точки не совпадает с from, то получим точку для from интерполяцией
            {
                TrajectoryPoint? firstPoint = GetSinglePoint<SatTableFacade>(from, minStep);
                if (firstPoint != null)
                {
                    TrajectoryPoint trajPoint = (TrajectoryPoint)firstPoint;
                    SpaceTime spt = new SpaceTime() { Position = trajPoint.Position.ToVector(), Time = from };
                    preTrajectory.Insert(0, spt);
                }
            }

            if (preTrajectory.Last().Time != to)  // если время последней точки не совпадает с to, то получим точку для to интерполяцией
            {
                TrajectoryPoint? secondPoint = GetSinglePoint<SatTableFacade>(to, minStep);
                if (secondPoint != null)
                {
                    TrajectoryPoint trajPoint = (TrajectoryPoint)secondPoint;
                    SpaceTime spt = new SpaceTime() { Position = trajPoint.Position.ToVector(), Time = to };
                    preTrajectory.Add(spt);
                }
            }
            return SpaceTime.createTrajectory(preTrajectory, minStep);
        }

        public Trajectory GetTrajectorySat(DateTime from, DateTime to, double minStep = OptimalChain.Constants.minTrajectoryStep)
        {
            return GetTrajectory<SatTableFacade>(from, to, minStep);
        }

        public Trajectory GetTrajectorySun(DateTime from, DateTime to, double minStep = OptimalChain.Constants.minSunTrajectoryStep)
        {
            return GetTrajectory<SunTableFacade>(from, to, minStep);
        }

        /// <summary>
        /// Fetches the satellite's view lane from the coverage table.
        /// </summary>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <returns></returns>
        public List<SatelliteTrajectory.LanePos> GetViewLane(DateTime from, DateTime to)
        {
            List<SatelliteTrajectory.LanePos> res = new List<SatelliteTrajectory.LanePos>();
            // Coverage table contains not what we need.
            //foreach (DataChunk chunk in GetDataBetweenDatesInChunks(CoverageTable.Name, CoverageTable.NadirTime, from, to, true))
            //    if (chunk.Rows.Length > 0)
            //    {
            //        foreach (DataRow row in chunk.Rows)
            //            res.Add(CoverageTable.GetLanePos(row));
            //    }
            //    else
            //    {
            //        // db is empty, generate view lane from sat positions
            //    }

            Trajectory traj = GetTrajectorySat(from, to);
            foreach (TrajectoryPoint p in traj.Points)
                res.Add(new SatelliteTrajectory.LanePos(p, 2 * OptimalChain.Constants.max_roll_angle + OptimalChain.Constants.camera_angle, 0));

            return res;
        }

        public List<CloudinessData> GetMeteoData(DateTime from, DateTime to)
        {
            List<CloudinessData> res = new List<CloudinessData>();
            var rows = GetTimePeriodsBetweenDates(MeteoReportTable.Name, MeteoReportTable.DTimeFrom, MeteoReportTable.DTimeTo, from, to);
            foreach (DataRow row in rows)
            {
                res.Add(new CloudinessData(MeteoReportTable.GetCloudness(row),
                    MeteoReportTable.GetPolygonWKT(row),
                    MeteoReportTable.GetTimeFrom(row),
                    MeteoReportTable.GetTimeTo(row)));
            }
            return res;
        }

        public List<Tuple<int, List<SatelliteTrajectory.LanePos>>> GetViewLaneBrokenIntoTurns(DateTime from, DateTime to)
        {
            List<Tuple<int, List<SatelliteTrajectory.LanePos>>> res = new List<Tuple<int, List<SatelliteTrajectory.LanePos>>>();
            List<Tuple<int, DateTime>> times = new List<Tuple<int, DateTime>>();
            DataRow[] preRows;
            var turnsData = GetDataBetweenDatesInChunks(OrbitTable.Name, OrbitTable.TimeEquator, from, to, false);

            foreach (DataChunk chunk in turnsData)
            {
                List<Tuple<int, DateTime>> turns = chunk.Rows.Select(row => Tuple.Create(OrbitTable.GetNumTurn(row), OrbitTable.GetTimeEquator(row))).ToList();

                preRows = GetDataBeforeEqualDate(OrbitTable.Name, OrbitTable.TimeEquator, chunk.Begin, 1);
                if (preRows.Length < 1) // no data about turns that began before the given date
                    throw new ArgumentException("Not enough data about turns from " + from.ToString() + " to " + to.ToString());

                Tuple<int, DateTime> preTurn = Tuple.Create(OrbitTable.GetNumTurn(preRows[0]), OrbitTable.GetTimeEquator(preRows[0]));

                times.Add(Tuple.Create(preTurn.Item1, chunk.Begin));

                int ind = 0;
                if (preTurn.Item2 == chunk.Begin) // == turns[0].Item2
                    ind++;
                while (ind < turns.Count)
                {
                    times.Add(turns[ind]);
                    ind++;
                }
                times.Add(Tuple.Create(-1, chunk.End));
            }

            for (int i = 0; i < times.Count - 1; ++i)
            {
                int cur_turn = times[i].Item1;
                DateTime start = times[i].Item2;

                while (i + 1 < times.Count - 1 && (times[i + 1].Item1 == -1 || times[i + 1].Item1 == cur_turn))
                    ++i;
                res.Add(Tuple.Create(cur_turn, GetViewLane(start, times[i + 1].Item2)));
            }

            if (res.Count() == 0)
                throw new ArgumentException("Not enough turns data from " + from.ToString() + " to " + to.ToString());

            return res;
        }

        /// <summary>
        /// Fetches data from a database that lies within the given dates.
        /// Returns a collection whose elements corresponds to three-day-long time intervals.
        /// </summary>
        /// <param name="tableName"></param>
        /// <param name="dateFieldName"></param>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <param name="margin"></param>
        /// <returns></returns>
        private IEnumerable<DataChunk> GetDataBetweenDatesInChunks(string tableName, string dateFieldName, DateTime from, DateTime to, bool includeEnd)
        {
            if (from > to)
                throw new ArgumentException("Beginning of the time interval cannot be bigger than its end.");

            DateTime start = from;
            DateTime finish = start + chunkLength;

            while (finish < to)
            {
                yield return new DataChunk
                {
                    Rows = GetDataBetweenDates(tableName, dateFieldName, start, finish, false),
                    Begin = start,
                    End = finish
                };
                start += chunkLength;
                finish += chunkLength;
            }

            yield return new DataChunk
            {
                Rows = GetDataBetweenDates(tableName, dateFieldName, start, to, includeEnd),
                Begin = start,
                End = to
            };
        }

        public DataRow[] GetDataBetweenDates(string tableName, string dateFieldName, DateTime from, DateTime to, bool includeEnd)
        {
            string fromStr = from.ToString(datePattern);
            string toStr = to.ToString(datePattern); 
            DataTable table = manager.GetSqlObject(tableName,
                String.Format("where {0} >= '{1}' and {0} <{3} '{2}' ORDER BY {0} ASC", dateFieldName, fromStr, toStr, includeEnd ? "=" : ""));
            return table.Select();
        }

        public DataRow[] GetDataBeforeDate(string tableName, string dateFieldName, DateTime date, int count)
        {
            return manager.GetSqlObject(
                tableName,
                String.Format("where {0} < '{1}' ORDER BY {0} DESC", dateFieldName, date.ToString(datePattern)),
                limit: count).Select();
        }

        public DataRow[] GetDataAfterDate(string tableName, string dateFieldName, DateTime date, int count)
        {
            return manager.GetSqlObject(
                tableName,
                String.Format("where {0} > '{1}' ORDER BY {0} ASC", dateFieldName, date.ToString(datePattern)),
                limit: count).Select();
        }
        public DataRow[] GetDataBeforeEqualDate(string tableName, string dateFieldName, DateTime date, int count)
        {
            return manager.GetSqlObject(
                tableName,
                String.Format("where {0} <= '{1}' ORDER BY {0} DESC", dateFieldName, date.ToString(datePattern)),
                limit: count).Select();
        }

        public DataRow[] GetDataAfterEqualDate(string tableName, string dateFieldName, DateTime date, int count)
        {
            return manager.GetSqlObject(
                tableName,
                String.Format("where {0} >= '{1}' ORDER BY {0} ASC", dateFieldName, date.ToString(datePattern)),
                limit: count).Select();
        }
        
        public DataRow[] GetTimePeriodsBetweenDates(
            string tableName,
            string dateFieldFirst,
            string dateFieldSecond,
            DateTime from,
            DateTime to)
        {
            return manager.GetSqlObject(
                tableName,
                String.Format("where {0} < '{1}' AND {2} > '{3}' ORDER BY {0} ASC",
                dateFieldFirst, to.ToString(datePattern),
                dateFieldSecond, from.ToString(datePattern)
                )).Select();
        }

        public int getNKA()
        {
            DataTable table = manager.GetSqlObject(SatteliteTable.Name,
                String.Format("where {0} = '{1}';", SatteliteTable.Id, 1));
            var res = table.Select();
            if (res.Count() != 1)
                throw new Exception("Can't fetch ka number");
            return SatteliteTable.GetNka(res.First());
        }

        /// <summary>
        /// Формирует список антенн, сортированный по приотритету использования
        /// </summary>
        /// <returns>список антенн, сортированный по приотритету использования</returns>
        public List<SessionsPlanning.CommunicationSessionStation> getSortedStations()
        {
            DataTable table = manager.GetSqlObject(StationTable.Name,"");
            var reqres = table.Select();            
            var res = reqres.Select(row => Tuple.Create( (SessionsPlanning.CommunicationSessionStation)StationTable.getId(row), StationTable.getPriority(row)));
            return res.OrderBy(row => row.Item2).Select(row => row.Item1).ToList();
        }
    }



    internal struct DataChunk
    {
        public DataRow[] Rows;
        public DateTime Begin;
        public DateTime End;
    }


    public abstract class BallisticTableFacade
    {
        public abstract string Name { get; }
        public abstract string Time { get; }

        public abstract Vector3D GetPosition(DataRow row);
        public abstract DateTime GetTime(DataRow row);
    }

    public class SunTableFacade : BallisticTableFacade
    {
        public override string Name { get { return SunTable.Name; } }
        public override string Time { get { return SunTable.Time; } }

        public override Vector3D GetPosition(DataRow row)
        {
            return SunTable.GetPosition(row);
        }
        public override DateTime GetTime(DataRow row)
        {
            return SunTable.GetTime(row);
        }
    }

    public class SatTableFacade : BallisticTableFacade
    {
        public override string Name { get { return SatTable.Name; } }
        public override string Time { get { return SatTable.Time; } }

        public override Vector3D GetPosition(DataRow row)
        {
            return SatTable.GetPosition(row);
        }
        public override DateTime GetTime(DataRow row)
        {
            return SatTable.GetTime(row);
        }
    }

    public static class SatteliteTable
    {
        public const string Name = "SATELLITE";
        public const string Id = "satellite";
        public const string Nka = "N_KA";
        public static int GetNka(DataRow row)
        {
            return (int)row[Nka];
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

        public static long GetId(DataRow row)
        {
            return (long)row[Id];
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

        public static long GetId(DataRow row)
        {
            return (long)row[Id];
        }

        public static byte GetNum(DataRow row)
        {
            return (byte)row[Num];
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

        public static decimal GetId(DataRow row)
        {
            return (decimal)row[Id];
        }

        public static byte GetNum(DataRow row)
        {
            return (byte)row[Num];
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

        public static Single GetTimeEquatorSec(DataRow row)
        {
            return (Single)row[TimeEquatorSec];
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

        public static long GetId(DataRow row)
        {
            return (long)row[Id];
        }

        public static byte GetNum(DataRow row)
        {
            return (byte)row[Num];
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

        public static DateTime GetWriteTime(DataRow row)
        {
            return (DateTime)row[WriteTime];
        }
    }

    public static class StationTable
    {
        public const string Name = "COMMUNICATION_SESSION_STATION";
        public const string Id = "communication_session_station";
        public const string NameCol = "name";
        public const string Priority = "priority";

        public static int getId(DataRow row)
        {
            return (int)row[Id];
        }
        public static string getName(DataRow row)
        {
            return (string)row[NameCol];
        }
        public static int getPriority(DataRow row)
        {
            return (int)row[Priority];
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

        public static decimal GetId(DataRow row)
        {
            return (decimal)row[Id];
        }

        public static Int16 GetNum(DataRow row)
        {
            return (Int16)row[Num];
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

        public static PositionMNKPOI GetDataMNKPOI(DataRow row)
        {
            return new PositionMNKPOI
            {
                Number = MnkpoiTable.GetNum(row),
                Position = MnkpoiTable.GetPositionGeo(row),
                Altitude = MnkpoiTable.GetAlt(row),
                TimeBeg = MnkpoiTable.GetTimeFrom(row),
                TimeEnd = MnkpoiTable.GetTimeTo(row)
            };
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

        public static byte GetSent(DataRow row)
        {
            return (byte)row[Sent];
        }
    }



    public static class MeteoReportTable // эта таблица находится в Базе ЦУКСа, а не ЦУПа, однако в будущем они будут объединены
    {
        public const string Name = "METEO_REPORT";
        public const string Polygon = "polygon_wkt";
        public const string DTimeFrom = "start_date";
        public const string DTimeTo = "end_date";
        public const string Cloudness = "cloudiness_prc";

        public static int GetCloudness(DataRow row)
        {
            return (int)row[Cloudness];
        }

        public static string GetPolygonWKT(DataRow row)
        {
            return (string)row[Polygon];
        }

        public static DateTime GetTimeFrom(DataRow row)
        {
            return (DateTime)row[DTimeFrom];
        }

        public static DateTime GetTimeTo(DataRow row)
        {
            return (DateTime)row[DTimeTo];
        }
    }


    public class SpaceTime
    {
        public Vector3D Position { get; set; }
        public DateTime Time { get; set; }

        /// <summary>
        /// Получить объект Astronomy.Trajectory из набора точек
        /// </summary>
        /// <param name="points">точки траектории</param>
        /// <returns></returns>
        public static Trajectory createTrajectory(List<SpaceTime> points, double minStep)
        {
            int count = points.Count;

            TrajectoryPoint[] trajectoryPoints = new TrajectoryPoint[points.Count];
            for (int i = 0; i < points.Count; i++)
            {
                Vector3D toNeighbor;
                double veloScalar;
                if (i == points.Count - 1) // для последней точки берём вектор на предыдущую точку
                {
                    toNeighbor = points.Last().Position - points[points.Count - 2].Position;
                    double angle = AstronomyMath.ToRad(Vector3D.AngleBetween(points.Last().Position, points[points.Count - 2].Position));
                    double time = (points.Last().Time - points[points.Count - 2].Time).TotalSeconds;
                    veloScalar = angle * points.Last().Position.Length / time;
                }
                else // для остальныйх на следюущую
                {
                    toNeighbor = points[i + 1].Position - points[i].Position;
                    double angle = AstronomyMath.ToRad(Vector3D.AngleBetween( points[i + 1].Position, points[i].Position));
                    double time = (points[i + 1].Time - points[i].Time).TotalSeconds;
                    veloScalar = angle * points[i + 1].Position.Length / time;
                }

                Vector3D prod = Vector3D.CrossProduct(toNeighbor, points[i].Position);
                Vector3D velo = Vector3D.CrossProduct(points[i].Position, prod);
                velo.Normalize();
               
                velo = velo * veloScalar;
                trajectoryPoints[i] = new TrajectoryPoint(points[i].Time, points[i].Position.ToPoint(), velo);
            }

            Trajectory trajectory = Trajectory.Create(trajectoryPoints);
            return Trajectory.changeMaximumTimeStep(trajectory, minStep);
        }
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



    public class CloudinessData : SatelliteSessions.TimePeriod
    {
        public double Cloudiness { get; private set; }
        public SphericalGeom.Polygon Polygon { get; private set; }


        public CloudinessData(int cloudiness_prc, string wkt, DateTime from, DateTime to)
            :base(from,to)
        {
            Cloudiness = (float)(cloudiness_prc) / 100;
            Polygon = new SphericalGeom.Polygon(wkt); 
        }


        public static bool isCapConfInCloud(OptimalChain.CaptureConf conf, CloudinessData cloud)
        {            
            if (conf.orders.Count != 1)
                throw new ArgumentException("Orders number is incorrect");

            OptimalChain.Order order = conf.orders.First();

            if (cloud.Cloudiness < order.request.Cloudiness)
                return false;

            if (!SatelliteSessions.TimePeriod.isPeriodsOverlap(cloud, conf)) // они не пересекаются по времени
                return false;

            var interctions = SphericalGeom.Polygon.Intersect(order.captured, cloud.Polygon);
            if (interctions.Count == 0)
                return false;

            return true;
        }

    }

}
