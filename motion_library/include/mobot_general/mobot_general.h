//
// Created by tianshipei on 2/14/16.
//

#ifndef MOBOT_SIMULATION_MOBOT_GENERAL_H
#define MOBOT_SIMULATION_MOBOT_GENERAL_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <vector>

#define NONE		0
#define FORWARD		1
#define BACKWARD	2
#define RIGHT		3
#define LEFT		4

double sgn(double x);
double min_spin(double spin_angle);
double quat2ang(geometry_msgs::Quaternion quaternion);
geometry_msgs::Quaternion ang2quat(double phi);
std::vector<double> quat2euler(geometry_msgs::Quaternion quaternion);


class RobotCommander {
private:
    ros::NodeHandle nh_;
    ros::Publisher twist_commander;
    //some "magic numbers"
    double sample_dt;
    double speed; // 1m/s speed command
    double yaw_rate; //0.5 rad/sec yaw rate command
    geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR
public:
    RobotCommander(ros::NodeHandle* nodehandle);
    void stop();

    void turn(double rad);
    void spin(int direction);

    void move(int direction, double time);
    void move(double time);
    void go(int direction);
};

#endif //MOBOT_SIMULATION_MOBOT_GENERAL_H
