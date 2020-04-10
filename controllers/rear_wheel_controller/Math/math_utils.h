#ifndef MATHUTILS_H
#define MATHUTILS_H

// #include <QMainWindow>
#include "math.h"
#include <stdio.h>
#include <stdlib.h>

// #include "Common/Configure/Configs/system_config.h"

namespace math{
    /**
     * @brief Normallize Angle to [-PI,PI)
     * @param angle the original value of the angle
     * @return The normalized value of the angle
     */
    double NormallizeAngle(const double angle);

    /**
     * @brief Clamp a value between two bounds.
     *        If the value goes beyond the bounds, return one of the bounds,
     *        otherwise, return the original value.
     * @param value The original value to be clamped.
     * @param bound1 One bound to clamp the value.
     * @param bound2 The other bound to clamp the value.
     * @return The clamped value.
     */
    template <typename T>
    T Clamp(const T value, T bound1, T bound2) {
      // if (bound1 > bound2) {
      //   std::swap(bound1, bound2);
      // }

      if (value < bound1) {
        return bound1;
      } else if (value > bound2) {
        return bound2;
      }
      return value;
    }
}

#endif // MATHUTILS_H
