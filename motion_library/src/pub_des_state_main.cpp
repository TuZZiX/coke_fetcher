#include "pub_des_state/pub_des_state.h"
#include <nav_msgs/Odometry.h>
#include <mobot_general/mobot_general.h>

bool is_init_orien = false;
geometry_msgs::Pose g_current_pose;

void odomCallback(const nav_msgs::Odometry& odom_msg) {
    if (!is_init_orien) {
        is_init_orien = true;
        g_current_pose = odom_msg.pose.pose;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_publisher");
    ros::NodeHandle nh;
    ros::Subscriber odom_subscriber = nh.subscribe("odom", 1, odomCallback);
    //instantiate a desired-state publisher object
    DesStatePublisher desStatePublisher(nh);
    //dt is set in header file pub_des_state.h    
    ros::Rate looprate(1 / dt); //timer for fixed publication rate
    ROS_INFO("Waiting for odometery message to start...");
    while (!is_init_orien) {
        ros::spinOnce();    //wait for odom callback
    }
    ROS_INFO("got odometery with x = %f, y = %f, th = %f", g_current_pose.position.x, g_current_pose.position.y, quat2ang(g_current_pose.orientation));
    desStatePublisher.set_init_pose(g_current_pose.position.x, g_current_pose.position.y, quat2ang(g_current_pose.orientation)); //x=0, y=0, psi=0
    //put some points in the path queue--hard coded here
    if (argc > 1 && ( strcmp(argv[1], "jinx") == 0 )) {
    	double gcpX  = g_current_pose.position.x;
	   	double gcpY  = g_current_pose.position.y;
		double gcpTh = quat2ang(g_current_pose.orientation);
        double x = 5.6 + gcpX;
        double y = -12.4 + gcpY;
        double v = 6.1 + 1.8 - 0.3; // 20 tiles + 3yd - 1ft for safety
        desStatePublisher.append_path_queue(x,   gcpY, gcpTh       );
        desStatePublisher.append_path_queue(x,   gcpY, gcpTh-M_PI/2);
        desStatePublisher.append_path_queue(x,   y,    gcpTh-M_PI/2);
        desStatePublisher.append_path_queue(x-v, y,    gcpTh-M_PI  );
    } else if (argc > 1 && ( strcmp(argv[1], "test") == 0 )) {
        desStatePublisher.append_path_queue(0.5,  0.0,  0.0);
    } else if (argc > 1 && ( strcmp(argv[1], "circle_out") == 0 )) {
        double circle_length = 2.0;
/*      clockwise*/
        desStatePublisher.append_path_queue(circle_length,  0.0, 0.0);
        desStatePublisher.append_path_queue(circle_length,  circle_length, -M_PI/2);
        desStatePublisher.append_path_queue(-1*circle_length,  circle_length, -M_PI);
        desStatePublisher.append_path_queue(-1*circle_length,  -1*circle_length, M_PI/2);
        desStatePublisher.append_path_queue(circle_length,  -1*circle_length, 0.0);
        desStatePublisher.append_path_queue(circle_length,  circle_length, -M_PI/2);
        desStatePublisher.append_path_queue(0.0,  circle_length, 0.0);
        desStatePublisher.append_path_queue(0.0,  6.0, -M_PI/2);
        desStatePublisher.append_path_queue(-9.0, 6.0, -M_PI);
        desStatePublisher.append_path_queue(-9.0, 0.0, M_PI/2);
    } else if (argc > 1 && ( strcmp(argv[1], "out") == 0 )) {
        desStatePublisher.append_path_queue(0.0,  7.0, -M_PI/2);
        desStatePublisher.append_path_queue(-9.0, 7.0, -M_PI);
        desStatePublisher.append_path_queue(-9.0, 0.0, M_PI/2);
    } else {
        desStatePublisher.append_path_queue(0.0,  0.0, 0.0);
    }
    // main loop; publish a desired state every iteration
    while (ros::ok()) {
        desStatePublisher.pub_next_state();
        ros::spinOnce();
        looprate.sleep(); //sleep for defined sample period, then do loop again
    }
}

