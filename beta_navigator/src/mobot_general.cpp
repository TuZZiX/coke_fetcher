//
// Created by tianshipei on 2/14/16.
//

#include <mobot_general/mobot_general.h>


//signum function: strip off and return the sign of the argument
double sgn(double x) {
    if (x > 0.0) {
        return 1.0;
    } else if (x < 0.0) {
        return -1.0;
    } else {
        return 0.0;
    }
}

//a function to consider periodicity and find min delta angle
double min_spin(double spin_angle) {
    if (spin_angle>M_PI) {
        spin_angle -= 2.0*M_PI;}
    if (spin_angle< -M_PI) {
        spin_angle += 2.0*M_PI;}
    return spin_angle;
}

// a useful conversion function: from quaternion to yaw
double quat2ang(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion ang2quat(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

std::vector<double> quat2euler(geometry_msgs::Quaternion quaternion) {
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

RobotCommander::RobotCommander(ros::NodeHandle &nodehandle) : nh_(nodehandle) {
    sample_dt = 0.001;
    speed = 1.0; // 1m/s speed command
    yaw_rate = 0.5; //0.5 rad/sec yaw rate command
    ros::Publisher twist_commander_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    twist_commander = twist_commander_;
}
void RobotCommander::stop() {
    ros::Rate loop_timer(1 / sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = 0.0;
    for (int i = 0; i < 10; i++) {
        twist_commander.publish(twist_cmd);
        loop_timer.sleep();
        ros::spinOnce();
    }
}

void RobotCommander::turn(double rad) {

    double time = abs(rad) / yaw_rate;
    double timer = 0.0;
    ros::Rate loop_timer(1 / sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate

    // start with all zeros in the command message; should be the case by default, but just to be safe..
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = 0.0;
    if (rad == 0) {

    } else if (rad > 0.0) {
        twist_cmd.angular.z = yaw_rate;
        ROS_INFO("Turn left");
    } else if (rad < 0.0) {
        twist_cmd.angular.z = -1.0 * yaw_rate;
        ROS_INFO("Turn right");
    }
    while (timer < time) {
        twist_commander.publish(twist_cmd);
        timer += sample_dt;
        loop_timer.sleep();
        ros::spinOnce();
    }
    stop();
}

void RobotCommander::spin(int direction) {
    ros::Rate loop_timer(1 / sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = 0.0;
    switch (direction) {
        case NONE:
            break;
        case LEFT:
            twist_cmd.angular.z = yaw_rate;
            break;
        case RIGHT:
            twist_cmd.angular.z = -1.0 * yaw_rate;
            break;
        default:
            break;
    }
    for (int i = 0; i < 10; i++) {
        twist_commander.publish(twist_cmd);
        loop_timer.sleep();
        ros::spinOnce();
    }
}

void RobotCommander::move(int direction, double time) {

    double timer = 0.0;
    ros::Rate loop_timer(1 / sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate

    // start with all zeros in the command message; should be the case by default, but just to be safe..
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = 0.0;
    switch (direction) {
        case NONE:
            break;
        case FORWARD:
            twist_cmd.linear.x = speed;
            break;
        case BACKWARD:
            twist_cmd.linear.x = -1.0 * speed;
            break;
        default:
            break;
    }
    while (timer < time) {
        twist_commander.publish(twist_cmd);
        timer += sample_dt;
        loop_timer.sleep();
        ros::spinOnce();
    }
    stop();
}

void RobotCommander::move(double time) {
    if (time >= 0) {
        move(FORWARD, time);
    } else {
        move(BACKWARD, -1 * time);
    }
}

void RobotCommander::go(int direction) {
    ros::Rate loop_timer(1 / sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate
    // start with all zeros in the command message; should be the case by default, but just to be safe..
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = 0.0;
    switch (direction) {
        case NONE:
            break;
        case FORWARD:
            twist_cmd.linear.x = speed;
            break;
        case BACKWARD:
            twist_cmd.linear.x = -1.0 * speed;
            break;
        default:
            break;
    }
    for (int i = 0; i < 10; i++) {
        twist_commander.publish(twist_cmd);
        loop_timer.sleep();
        ros::spinOnce();
    }
}