using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace OptimalChain
{
    public class Constants
    {
        public const int minDeltaT = 19000;
        public const int min_shooting_time = 30000;
        public const double angle_velocity_max = Math.PI / 60;
        public const double min_degree = Math.PI/30;
        public const double camera_angle = 0.016616; // ~0.952 градуса - угол обзора камеры
    }
}
