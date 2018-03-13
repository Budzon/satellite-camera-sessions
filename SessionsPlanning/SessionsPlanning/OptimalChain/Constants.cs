﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace OptimalChain
{
    public class Constants
    {
        public const int MPZ_ending_Time = 7000;// 2с -- сохранение, 5с -- отключение СОЭН;
        public const int MPZ_ending_Time_PWRON = 7000;// 2с -- сохранение, 5с -- отключение СОЭН;

        public const int MPZ_starting_Time =10000;// минимальное;
        public const int MPZ_starting_Time_RESERVE = 120000;// c учетом перехода на резервную конфигурацию;

        public const int MPZ_init_Time = 47000;// минимальное;
        public const int MPZ_init_Time_RESERVE = 70000;// c учетом перехода на резервную конфигурацию;

        public const int MPZ_delta = 0;//2000;

        public const int min_shooting_time = 2000;
        public const int min_Delta_time = 8000;
        public const int minDeltaT = 19000;


        public const double angle_velocity_max = Math.PI / 60;
        public const double min_degree = Math.PI / 30;
        public const double camera_angle = 0.016616; // ~0.952 градуса - угол обзора камеры
        public const double earthRotSpeed = Math.PI / (12 * 60 * 60);
        public const int pitchStep = 1; // угол изменения тангажа в градусах
        public const double orbital_inclination = 1.7104; // угол наклона орбиты в градусах
        public const double orbit_height = 720; // км (или 650?)
    }
}
