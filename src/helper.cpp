#include "helper.h"


Angles get_angles_from_quat(imu::Quaternion quat, int8_t x_dir, int8_t y_dir, int8_t z_dir) {
    Angles angles;

    angles.from_x = x_dir * 2 * atan2(quat.x(), quat.w()) * (180.0/3.141592653589793238463);
    angles.from_y = y_dir * 2 * atan2(quat.y(), quat.w()) * (180.0/3.141592653589793238463);
    angles.from_z = z_dir * 2 * atan2(quat.z(), quat.w()) * (180.0/3.141592653589793238463);
    // angles.theta  = 2 * atan2(sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z), quat.w) * (180.0/3.141592653589793238463);  

    return angles;
}
