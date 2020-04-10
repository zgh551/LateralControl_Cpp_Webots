#include "utils_math.hpp"

namespace math
{
    double NormalizeAngle(double angle)
    {
        double a = fmod(angle + M_PI,2.0 * M_PI);
        if(a < 0.0)
        {
            a += 2.0 * M_PI;
        }
        return a - M_PI;
    }
} // namespace name
