//
// Created by sxt437 on 4/30/16.
//

#include <BaxterArmCommander/BaxterArmCommander.h>

std::vector<double> BaxterArmCommander::quat2euler(geometry_msgs::Quaternion quaternion) {
    double mData[4];
    std::vector<double> euler(3);
    const static double PI_OVER_2 = M_PI * 0.5;
    const static double EPSILON = 1e-10;
    double sqw, sqx, sqy, sqz;

    mData[0] = quaternion.x;
    mData[1] = quaternion.y;
    mData[2] = quaternion.z;
    mData[3] = quaternion.w;
    // quick conversion to Euler angles to give tilt to user
    sqw = mData[3] * mData[3];
    sqx = mData[0] * mData[0];
    sqy = mData[1] * mData[1];
    sqz = mData[2] * mData[2];

    euler[1] = asin(2.0 * (mData[3] * mData[1] - mData[0] * mData[2]));
    if (PI_OVER_2 - fabs(euler[1]) > EPSILON) {
        euler[2] = atan2(2.0 * (mData[0] * mData[1] + mData[3] * mData[2]),
                         sqx - sqy - sqz + sqw);
        euler[0] = atan2(2.0 * (mData[3] * mData[0] + mData[1] * mData[2]),
                         sqw - sqx - sqy + sqz);
    } else {
        // compute heading from local 'down' vector
        euler[2] = atan2(2 * mData[1] * mData[2] - 2 * mData[0] * mData[3],
                         2 * mData[0] * mData[2] + 2 * mData[1] * mData[3]);
        euler[0] = 0.0;

        // If facing down, reverse yaw
        if (euler[1] < 0)
            euler[2] = M_PI - euler[2];
    }
    return euler;
}
