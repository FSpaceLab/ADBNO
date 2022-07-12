#ifndef HELPER_H
#define HELPER_H

#include "utility/imumaths.h"

struct Angles {
    float from_x, from_y, from_z, theta;
};

Angles get_angles_from_quat(imu::Quaternion quat, int8_t x_dir, int8_t y_dir, int8_t z_dir);

#endif