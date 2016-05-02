#include <pub_des_state/pub_des_state.h>
#include <mobot_general/mobot_general.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_publisher");
    ros::NodeHandle nh;

    RobotCommander robot(nh);
    //instantiate a desired-state publisher object
    DesStatePublisher desStatePublisher(nh);

    ROS_INFO("You have 5 second to move the robot");
    ros::Rate wait(5);
    wait.sleep();
    ROS_INFO("Waiting for good amcl particles");
    robot.turn(M_PI*2);

    //ROS_INFO("Waiting for odometery message to start...");
    geometry_msgs::Pose g_current_pose = desStatePublisher.get_odom().pose.pose;
    ROS_INFO("got odometery with x = %f, y = %f, th = %f", g_current_pose.position.x, g_current_pose.position.y, quat2ang(g_current_pose.orientation));
    desStatePublisher.sync_pose();
    desStatePublisher.enable_replanning();
    double gcpX  = g_current_pose.position.x;
    double gcpY  = g_current_pose.position.y;
    double gcpTh = quat2ang(g_current_pose.orientation);

    if (argc > 1 && ( strcmp(argv[1], "jinx") == 0 )) {
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
        desStatePublisher.append_path_queue(-circle_length,  circle_length, -M_PI);
        desStatePublisher.append_path_queue(-circle_length,  -circle_length, M_PI/2);
        desStatePublisher.append_path_queue(circle_length,  -circle_length, 0.0);
        desStatePublisher.append_path_queue(circle_length,  circle_length, -M_PI/2);
/*
        desStatePublisher.append_path_queue(circle_length,  0.0, -M_PI/2);
        desStatePublisher.append_path_queue(circle_length,  -circle_length, -M_PI);
        desStatePublisher.append_path_queue(-circle_length,  -circle_length, M_PI/2);
        desStatePublisher.append_path_queue(-circle_length,  circle_length, 0.0);
*/
        desStatePublisher.append_path_queue(0.0,  circle_length, M_PI/2);
        /*desStatePublisher.append_path_queue(0.0,  7.0, M_PI);
        desStatePublisher.append_path_queue(-9.0, 7.0, -M_PI/2);
        desStatePublisher.append_path_queue(-9.0, 0.0, -M_PI/2);*/
        desStatePublisher.append_path_queue(0.0,  7.0, -M_PI/2);
        desStatePublisher.append_path_queue(-9.0, 7.0, -M_PI);
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
        desStatePublisher.loop_sleep();
    }
}

