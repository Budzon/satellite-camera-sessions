using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using Astronomy;
using Common;

namespace SatelliteSessions
{

    public class TimePeriod
    {
        public DateTime dateFrom { get; private set; }
        public DateTime dateTo { get; private set; }

        public TimePeriod(DateTime from, DateTime to)
        {
            dateFrom = from;
            dateTo = to;
        }

        public virtual TimePeriod Unite(TimePeriod p2)
        {
            DateTime newFrom = dateFrom < p2.dateFrom ? dateFrom : p2.dateFrom;
            DateTime newTo = dateTo > p2.dateTo ? dateTo : p2.dateTo;
            return new TimePeriod(newFrom, newTo);
        }

        public virtual bool Equals(TimePeriod obj)
        {
            return (dateFrom.Equals(obj.dateFrom) && dateTo.Equals(obj.dateTo));
        }

        /// <summary>
        /// Объеденим пересекающиеся временные диапазоны 
        /// </summary>
        /// <param name="timePeriods">входные необъединённые временные диапазоны</param>
        /// <param name="accuracy">допуск в секундах (можно объеденить два диапазона, если они близка к друг другу на accuracy) </param>
        /// <returns>объединённые временные диапазоны</returns>
        public static List<T> compressTimePeriods<T>(List<T> timePeriods, double accuracy = 0) where T : TimePeriod
        {
            if (timePeriods.Count == 0)
                return timePeriods;

            List<T> sortedPeriods = new List<T>(timePeriods);
            sortedPeriods.Sort(delegate(T span1, T span2) { return span1.dateFrom.CompareTo(span2.dateFrom); });

            List<T> res = new List<T>();

            T curPeriod = sortedPeriods.First();

            foreach (var period in sortedPeriods.Skip(1))
            {
                if (period.dateFrom <= curPeriod.dateTo.AddSeconds(accuracy))
                {
                    curPeriod = (T)period.Unite(curPeriod);
                }
                else
                {
                    res.Add(curPeriod);
                    curPeriod = period;
                }
            }

            res.Add(curPeriod);

            return res;
        }

        public static List<TimePeriod> getFreeIntervals(List<TimePeriod> compressedOccupiedPeriods, DateTime timeFrom, DateTime timeTo)
        {
            compressedOccupiedPeriods.Sort(delegate(TimePeriod span1, TimePeriod span2) { return span1.dateFrom.CompareTo(span2.dateFrom); });

            List<TimePeriod> res = new List<TimePeriod>();

            DateTime firstDt = timeFrom;

            foreach (var timeSpan in compressedOccupiedPeriods)
            {
                if (timeSpan.dateTo < timeFrom)
                    continue;
                if (timeSpan.dateFrom > timeTo)
                    break;
                if (firstDt < timeSpan.dateFrom)
                    res.Add(new TimePeriod(firstDt, timeSpan.dateFrom));
                firstDt = timeSpan.dateTo;
            }

            if (firstDt < timeTo)
                res.Add(new TimePeriod(firstDt, timeTo));

            return res;
        }

        /// <summary>
        /// является ли период времнеи checkPeriod полностью принадлежащим одному из periods
        /// </summary>
        /// <param name="checkPeriod"></param>
        /// <param name="periods"></param>
        /// <returns></returns>
        public static bool isPeriodInPeriods(TimePeriod checkPeriod, List<TimePeriod> periods)
        {
            foreach (var period in periods)
            {
                if (isPeriodInPeriod(checkPeriod, period))
                    return true;
            }
            return false;
        }

        public static bool isPeriodInPeriod(TimePeriod checkPeriod, TimePeriod period)
        {
            if (period.dateFrom <= checkPeriod.dateFrom && checkPeriod.dateFrom <= period.dateTo
             && period.dateFrom <= checkPeriod.dateTo && checkPeriod.dateTo <= period.dateTo)
                return true;
            else
                return false;
        }
    }
}
