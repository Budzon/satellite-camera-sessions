using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace OptimalChain
{
    public static class Constants
    {
        public const int MPZ_ending_Time = 7000;// 2с -- сохранение, 5с -- отключение СОЭН;
        public const int MPZ_ending_Time_PWRON = 7000;// 2с -- сохранение, 5с -- отключение СОЭН;

        public const int MPZ_starting_Time =10000;// минимальное;
        public const int MPZ_starting_Time_RESERVE = 120000;// c учетом перехода на резервную конфигурацию;

        public const int MPZ_init_Time = 47000;// минимальное;
        public const int MPZ_init_Time_RESERVE = 70000;// c учетом перехода на резервную конфигурацию;

        public const int MPZ_max_lasting_time = 1400000;

        public const int MPZ_delta = 0;//2000;

        public const int min_shooting_time = 2000;
        public const int min_Delta_time = 8000;
        public const int minDeltaT = 19000;
         
        // максимальный промежуток между двумя конфигурациями, при котором они объединяются в одну
        public const int maxCConfInterval = 2;

        public const double angle_velocity_max = Math.PI / 60;
        public const double min_degree = Math.PI / 30;
        public const double camera_angle = 0.016616; // ~0.952 градуса - угол обзора камеры
        public const double max_roll_angle = 0.78540; // 45 градусов - максимально возможный угол крена
        public const double max_pitch_angle = 0.52360; // 30 градусов - максимально возможный угол тангажа
        public const double earthRotSpeed = 7.2921158553e-5; // 1/c
        public const double orbital_inclination = 1.7104; // угол наклона орбиты в градусах
        public const double orbit_height = 720; // км (или 650?)
        public const double routeDeleteTime = 15; // время на удаление маршрута
        public const double stereoPitchAngle = 0.52360;
        public const int stripStepPassing = 1; // шаг по колву точек. Значение 1, если используем все точки, без оптимизации
        public const int stripPolygonStep = 10000; // расстояние между точками в метрах
        public const int minTrajectoryStep = 10; // минимальный шаг траектории в секундах. Если в БД лежит траектория с худшим шагом, то интерполируем точки.
        public const double stripOverlap = 0.05; // перекрытие соседних полос.
        public const int compressionDropCapture = 10; // значение коэффициета сжатия, при котором необходимо попробовать съемку со сбросом
        public const int minTrajectoryPassInterval = 120; //[секнуды] максимальный разрыв по времени между двумя точками траектории, при котором мы продолжаем с такой траекторией работать.


        public static int CountMinPause(WorkingType t1, ShootingType st1, ShootingChannel channel1, WorkingType t2, ShootingType st2, ShootingChannel channel2)
        {
            int d = Constants.min_Delta_time;
            if (t1 == WorkingType.eCapture)
            {
                if (channel1 != ShootingChannel.ePK)
                {
                    d += 4;
                }
                if (st1 == ShootingType.eCorridor)
                {
                    d += 4;
                }
            }

            if (t2 == WorkingType.eCapture)
            {
                if (channel2 != ShootingChannel.ePK)
                {
                    d += 4;
                }
                if (st2 == ShootingType.eCorridor)
                {
                    d += 4;
                }
            }

            return d * 1000;
        }
    }
}
