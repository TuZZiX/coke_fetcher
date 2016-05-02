//pub_des_state_path_client:
// illustrates how to send a request to the append_path_queue_service service

#include <ros/ros.h>
#include <beta_navigator/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
using namespace std;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "append_path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<beta_navigator::path>("append_path_queue_service");
    ros::ServiceClient flush = n.serviceClient<std_srvs::Trigger>("flush_path_queue_service");
    ros::ServiceClient estop = n.serviceClient<std_srvs::Trigger>("estop_service");
    ros::ServiceClient clear_estop = n.serviceClient<std_srvs::Trigger>("clear_estop_service");
    geometry_msgs::Quaternion quat;
    std_srvs::Trigger trigger;
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
      ros::spinOnce();
    }
    //ROS_INFO("connected client to service");
    beta_navigator::path path_srv;

    if (argc < 2) return 1;

    if (!strcmp(argv[1], "flush")) {
    	ROS_INFO("flush current path");
        flush.call(trigger);
    } else if (!strcmp(argv[1], "stop")) {
    	ROS_INFO("E-stop engaged");
        estop.call(trigger);
    } else if (!strcmp(argv[1], "clear")) {
    	ROS_INFO("E-stop released");
        clear_estop.call(trigger);
    } else if (!strcmp(argv[1], "append")) {
        if (argc == 2) {
            //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "world";
            geometry_msgs::Pose pose;
            pose.position.x = 5.0; // say desired x-coord is 5
            pose.position.y = 0.0;
            pose.position.z = 0.0; // let's hope so!
            quat = convertPlanarPhi2Quaternion(0);
            pose.orientation = quat;
            pose_stamped.pose = pose;
            path_srv.request.path.poses.push_back(pose_stamped);
         
            pose.position.y = 5.0;
            pose_stamped.pose = pose;
            path_srv.request.path.poses.push_back(pose_stamped);

            pose.position.x = 0.0;
            pose_stamped.pose = pose;
            path_srv.request.path.poses.push_back(pose_stamped);
            
            pose.position.y = 0.0;
            pose_stamped.pose = pose;
            path_srv.request.path.poses.push_back(pose_stamped);
            
            //repeat (x,y) with new heading:
            pose_stamped.pose.orientation = convertPlanarPhi2Quaternion(0); 
            path_srv.request.path.poses.push_back(pose_stamped);

            ROS_INFO("Append goal with %d paths", (int)(path_srv.request.path.poses.size()));
        } else if ((argc - 2) % 3 == 0) {
            int i = 1;
            path_srv.request.path.poses.clear();
            while (i < (argc - 1)) {
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.frame_id = "world";
                geometry_msgs::Pose pose;
                pose.position.x = atof(argv[++i]);
                pose.position.y = atof(argv[++i]);
                pose.position.z = 0;
                quat = convertPlanarPhi2Quaternion(atof(argv[++i]));
                pose.orientation = quat;
                pose_stamped.pose = pose;
                path_srv.request.path.poses.push_back(pose_stamped);
            }
            ROS_INFO("Append goal with %d paths", (int)(path_srv.request.path.poses.size()));
        } else {
            ROS_ERROR("Incorrect number of arguments.  Should be 3n+2, received %d", argc);
        }
        client.call(path_srv);
    } else {
        ROS_ERROR("Command should be 'flush' or 'append' or 'stop' or 'go', received %s", argv[1]);
    }

    return 0;
}
