//
// Created by sxt437 on 4/30/16.
//

#ifndef COKE_GRABBER_BAXTERGRIPPER_H
#define COKE_GRABBER_BAXTERGRIPPER_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

class BaxterGripper {
public:
    BaxterGripper(ros::NodeHandle &nodehandle);
    void close();
    void open();
    void test();
    void set_mode(const int mode);

    static const int TORQUE = 1;
    static const int POSITION = 2;
    static const int OPEN_CLOSE = 3;

private:
    ros::NodeHandle nh_;
    ros::Publisher open_close_pub;
    ros::Publisher position_pub;
    ros::Publisher mode_pub;
    std_msgs::Int16 position_msg;
    std_msgs::Int16 cmd_msg;
    std_msgs::Bool mode_msg;
    std_msgs::Bool open_close_msg;
    int mode;
};


#endif //COKE_GRABBER_BAXTERGRIPPER_H
