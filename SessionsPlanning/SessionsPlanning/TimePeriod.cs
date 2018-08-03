using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using Astronomy;
using Common;
using OptimalChain;
using SessionsPlanning;

namespace SatelliteSessions
{


    public static class TimePeriodExtentions
    {

        public static void erase(this List<TimePeriod> intervals, List<TimePeriod> intervalsToErase)
        {
            foreach (var intToErase in intervalsToErase)            
                intervals.erase(intToErase);            
        }

        /// <summary>
        /// вырежем интервал intervalToErase из тех интервалов, с которыми у него есть пересечения
        /// </summary>
        /// <param name="intervals"></param>
        /// <param name="intervalToErase"></param>
        public static void erase(this List<TimePeriod> intervals, TimePeriod intervalToErase)
        {
            List<TimePeriod> intersected = intervals.Where(interval => TimePeriod.isPeriodsOverlap(interval, intervalToErase)).ToList();
            List<TimePeriod> newInts = intersected.SelectMany(interval => interval.erase(intervalToErase)).ToList();
            intervals = intervals.Except(intersected).Concat(newInts).ToList();
        }
    }

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

        public bool isDtInPeriod(DateTime dt)
        {
            return dateFrom <= dt && dt <= dateTo;
        }

        public List<TimePeriod> erase(TimePeriod periodToErase)
        {
            List<TimePeriod> res = new List<TimePeriod>();
            if (this.dateFrom < periodToErase.dateFrom)
            {
                res.Add(new TimePeriod(this.dateFrom, TimePeriod.Min(this.dateTo, periodToErase.dateFrom)));
            }

            if (periodToErase.dateTo < this.dateTo)
            {
                res.Add(new TimePeriod(periodToErase.dateTo, TimePeriod.Max(this.dateFrom, periodToErase.dateTo)));
            }
            return res;
        }


        public static DateTime Max(DateTime dt1, DateTime dt2)
        {
            return (dt1 > dt2 ? dt1 : dt2);
        }
        public static DateTime Min(DateTime dt1, DateTime dt2)
        {
            return (dt1 < dt2 ? dt1 : dt2);
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

        public static bool isPeriodsOverlap(TimePeriod p1, TimePeriod p2)
        {
            if (p1.isDtInPeriod(p2.dateFrom) || p1.isDtInPeriod(p2.dateTo))
                return true;

            if (p2.isDtInPeriod(p1.dateFrom) || p2.isDtInPeriod(p1.dateTo))
                return true;

            return false;
        }


        /// <summary>
        /// распределить маршруты по доступным временным интервалам
        /// </summary>
        /// <param name="routesToDrop"></param>
        /// <param name="freeCompressedIntervals"></param>
        /// <param name="station"> антенна, через которую будет осуществлён сброс. Нужно только для сброса</param>
        /// <returns> возвращает словарь *временной интервал / маршруты, помещенные в этот интервал*  </returns>
        public static List<RouteParams> getRoutesParamsInIntervals(
            List<RouteParams> routes,
            List<TimePeriod> freeCompressedIntervals,
            WorkingType workType,
            CommunicationSessionStation station = CommunicationSessionStation.FIGS_Main)
        {  
            List<RouteParams> res = new List<RouteParams>();
            // отсортируем свободные промежутки по продолжительности в порядке возрастания по прищнаку продолжительности
            freeCompressedIntervals.Sort(delegate(TimePeriod span1, TimePeriod span2)
            { return (span1.dateTo - span1.dateFrom).CompareTo(span2.dateTo - span2.dateFrom); });

            foreach (var interval in freeCompressedIntervals.ToArray())
            {                 
                // получим все маршруты, которые могут быть сброшены/удалены в этом промежутке
                List<RouteParams> curRoutes = routes.Where(rout => rout.end < interval.dateTo).ToList();

                if (curRoutes.Count == 0)
                    continue;

                curRoutes.Sort(delegate(RouteParams rout1, RouteParams rout2) { return rout1.end.CompareTo(rout2.end); });                 

                DateTime prevDt = curRoutes[0].end > interval.dateFrom ? curRoutes[0].end : interval.dateFrom;
                foreach (var rmpz in curRoutes.ToArray())
                {
                    if (prevDt < rmpz.end)
                        prevDt = rmpz.end; // если текущее время раньше времени конца работы сбрасываемого/удаляемого маршрута, сдвигаем текущее время

                    double actionTime = 0;

                    if (workType == WorkingType.Downloading)
                        actionTime = rmpz.getDropTime(station).TotalSeconds;
                    else if (workType == WorkingType.Removal)
                        actionTime = OptimalChain.Constants.routeDeleteTime;
                    else
                        throw new ArgumentException("Unsupported working type");

                    DateTime nextDt = prevDt.AddSeconds(actionTime);
                    if (nextDt > interval.dateTo)
                        break; // если вышли за пределы текущего интервала, переходим к следующему интервалу

                    nextDt.AddMilliseconds(OptimalChain.Constants.min_Delta_time); // @todo точно ли эта дельта?

                    RouteParams curParam = new RouteParams(workType, prevDt, nextDt, rmpz.Address);                    
                    curParam.ShootingConf = rmpz.ShootingConf;
                    res.Add(curParam);
                    
                    routes.Remove(rmpz);
                    prevDt = nextDt;
                }
            }

            return res;
        }
    }
}
