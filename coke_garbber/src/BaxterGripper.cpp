//
// Created by sxt437 on 4/30/16.
//

#include <BaxterGripper/BaxterGripper.h>


BaxterGripper::BaxterGripper(ros::NodeHandle &nodehandle) : nh_(nodehandle) {
    position_pub = nh_.advertise<std_msgs::Int16>("/dynamixel_motor1_cmd", 1);
    mode_pub= nh_.advertise<std_msgs::Bool>("/dynamixel_motor1_mode", 1);
    open_close_pub = nh_.advertise<std_msgs::Bool>("/gripper_open_close", 1);
    set_mode(OPEN_CLOSE);
}

void BaxterGripper::set_mode(const int mode) {
    this->mode = mode;
}

void BaxterGripper::close() {
    switch (mode) {
        case 2:
            position_msg.data = 3999;
            position_pub.publish(position_msg);
            break;
        case 1:
            cmd_msg.data = 80;
            mode_msg.data = 1;
            position_pub.publish(cmd_msg);
            mode_pub.publish(mode_msg);
            break;
        case 3:
            open_close_msg.data = 1;
            open_close_pub.publish (open_close_msg);
            break;
    }
}

void BaxterGripper::open() {
    switch (mode) {
        case 2:
            position_msg.data = 3000;
            position_pub.publish(position_msg);
            break;
        case 1:
            cmd_msg.data = 3200;
            mode_msg.data = 0;
            position_pub.publish(cmd_msg);
            mode_pub.publish(mode_msg);
            break;
        case 3:
            open_close_msg.data = 0;
            open_close_pub.publish (open_close_msg);
            break;
    }
}

void BaxterGripper::test() {
    mode = OPEN_CLOSE;
    open();
    ros::Duration(3).sleep();
    close();
    ros::Duration(3).sleep();
    open();
}