using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SessionsPlanning;

namespace OptimalChain
{
    public static class Constants
    {
        ///Все константы для МПЗ и маршрутов теперь считаем в секундах. Милисекунды оказались не нужны. 
        public const int MPZ_ending_Time = 7;// 2с -- сохранение, 5с -- отключение СОЭН;
        public const int MPZ_ending_Time_PWRON = 0;// Если мы не выключаем СОЭН, то ничего и делать не надо. Пока. Поэтому на всякий случай сделаем такую константу;

        public const int SOEN_turning_on_Time = 150;// 2.5 минуы для любой конфиурации;

        public const int MPZ_max_lasting_time = 1410;// максимальное время работы СОЭН без выключения (т.е. в одном МПЗ или в нескольних со флагом PWR_ON)
        public const int max_shooting_time_per_turn = 800;//максимальное время съемки без выключения СОЭН (т.е. в одном МПЗ или в нескольних со флагом PWR_ON)

        public const int MPZ_delta = 485;//8 минут 5 секун, если перед этим МПЗ не было МПЗ с флагом PWR_ON;

        public const int shooting_route_init_time = 12;//время инициализации маршрута с любой съемкой
        public const int not_shooting_route_init_time = 8;//время инициализации маршрута на сброс/удаление

        public const int shooting_route_end_time = 9;//время окончания маршрута с любой съемкой
        public const int not_shooting_route_end_time = 5;//время окончания маршрута на сброс/удаление

        //это надо изжить
        public const int min_Delta_time = 8;//минимальная пауза между маршрутами, отведенная на инициализацию нового и завершение предыдущего

        public const int minReconfStabilizationT = 19;//минимальное время затрачиваемое на поворот -- это разгон, торможение, стаблилизация. Время на сам поворот считается отдельно

        //***********************************************************************************
                
        public const int minShootingDuration = 3; // [сек] минимальная продолжительность съемки. Если съемка меньше этого значения, мы увеличиваем её продолжительность до этого вручную.
        public const double turnDuration = 90.2;  // продолжительность витка в секундах

        public const double maxPitchTimeDelta = 70; // ПРИБЛИЗИТЕЛЬНОЕ максимально возможное время, которое можно компенсировать тангажом
        // максимальный промежуток (в секундах) между двумя конфигурациями, при котором они объединяются в одну
        public const int maxCConfInterval = 2;
        public const int  max_route_duration = 1000; // секунды
        public const double angle_velocity_max = Math.PI / 60;
        public const double min_degree = Math.PI / 30;
        public const double camera_angle = 0.016616; // ~0.952 градуса - угол обзора камеры
        public const double sunBlindingAngle = 0.17453; // 10 градусов - минимально допустимый угол между направлением взора камеры и направлением вектора  на солнце
        public const double max_roll_angle = 0.78540; // 45 градусов - максимально возможный угол крена
        public const double max_pitch_angle = 0.52360; // 30 градусов - максимально возможный угол тангажа
        public const double roll_correction_epsilon = 0.001; // точность нахождения правки по крену в секундах 
        public const double earthRotSpeed = 7.2921158553e-5; // 1/c скорость вращения земли
        public const double orbital_inclination = 1.7104; // угол наклона орбиты в градусах
        public const double orbit_height = 720; // км (или 650?)
        public const double routeDeleteTime = 15; // время на удаление маршрута
        public const double stereoPitchAngle = 0.52360;
        public const int stripStepPassing = 1; // шаг по колву точек. Значение 1, если используем все точки, без оптимизации
        // public const int stripPolygonStep = 10000; // расстояние между точками в метрах
        public const int minTrajectoryStep = 10; // минимальный шаг траектории в секундах. Если в БД лежит траектория с худшим шагом, то интерполируем точки.
        public const int minSunTrajectoryStep = 60; 
        public const double stripOverlap = 0.05; // перекрытие соседних полос.
        public const int compressionDropCapture = 10; // значение коэффициета сжатия, при котором необходимо попробовать съемку со сбросом
        public const int minTrajectoryPassInterval = 120; //[секнуды] максимальный разрыв по времени между двумя точками траектории, при котором мы продолжаем с такой траекторией работать.

<<<<<<< HEAD
        //@toDo удалить после завершения рефакторинга
=======
        /// <summary>
        /// Вернуть полный путь до файла known_dem_cut.list  
        /// </summary>
        /// <returns> полный путь до файла known_dem_cut.list  </returns>
        static public string demHandlerKnownDemCut()
        {
            return AppDomain.CurrentDomain.BaseDirectory + "/bin/" + "known_dem_cut.list";
        }

        /// <summary>
        /// Вернуть полный путь до файла dem_location.conf
        /// </summary>
        /// <returns> полный путь до файла dem_location.conf</returns>
        static public string demHandlerDemLocation()
        {
            return AppDomain.CurrentDomain.BaseDirectory + "/bin/" + "dem_location.conf";
        }

>>>>>>> prototypes
        public static int CountMinPause(WorkingType t1, ShootingType st1, ShootingChannel channel1, WorkingType t2, ShootingType st2, ShootingChannel channel2)
        {
            int d = Constants.min_Delta_time; // в константах время указано в милисекундах, а тут мы все считаем в секундах
            if (t1 == WorkingType.Shooting)
            {
                if (channel1 != ShootingChannel.pk)
                {
                    d += 4;
                }
                if (st1 == ShootingType.Coridor)
                {
                    d += 4;
                }
            }

            if (t2 == WorkingType.Shooting)
            {
                if (channel2 != ShootingChannel.pk)
                {
                    d += 4;
                }
                if (st2 == ShootingType.Coridor)
                {
                    d += 4;
                }
            }

            return d;
        }
    }
}

