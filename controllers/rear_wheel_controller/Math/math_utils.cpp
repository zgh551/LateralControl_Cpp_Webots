#include "math_utils.h"

namespace math{

/**
 * @brief Normallize Angle to [-PI,PI)
 * @param angle the original value of the angle
 * @return The normalized value of the angle
 */
double NormallizeAngle(const double angle)
{
    double a = fmod(angle + static_cast<double>(M_PI),2.0 * static_cast<double>(M_PI));
    if(a < 0.0)
    {
        a += 2.0 * static_cast<double>(M_PI);
    }
    return  a - static_cast<double>(M_PI);
}



}


