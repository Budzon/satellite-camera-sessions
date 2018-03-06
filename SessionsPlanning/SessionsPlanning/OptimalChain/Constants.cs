using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace OptimalChain
{
    public class Constants
    {
        public const int MPZ_ending_Time = 0;// 2000;
        public const int MPZ_starting_Time =0;// 2000;
        public const int MPZ_delta = 0;//2000;

        public const int min_shooting_time = 2000;
        public const int min_Delta_time = 12;
        public const int minDeltaT = 19000;
       
        public const double angle_velocity_max = Math.PI / 60;
        public const double min_degree = Math.PI / 30;
        public const double camera_angle = 0.016616; // ~0.952 градуса - угол обзора камеры
        public const double earthRotSpeed = Math.PI / (12 * 60 * 60);
        public const int pitchStep = 1; // в градусах
        public const double orbital_inclination = 1.7104; // угл наклона орбиты в градусах
        public const double orbit_height = 650; // км (или 720?)
    }
}
