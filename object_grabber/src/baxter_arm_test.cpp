//
// Created by sxt437 on 5/1/16.
//
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/TableArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>  //  transform listener headers
#include <tf/transform_broadcaster.h>
#include <BaxterArmCommander/BaxterArmCommander.h>

std::string coke_id = "";
double coke_conficent = 0;
geometry_msgs::PoseStamped coke_pose;
bool firstCB = false;

void objectCallback(const object_recognition_msgs::RecognizedObjectArray objects_msg) {
    double confident = 0;
    int id = -1;
    ROS_WARN("Total %d objects found", (int)objects_msg.objects.size());
    ROS_INFO("Frame_id 1: %s", objects_msg.header.frame_id.c_str());

    if (firstCB == false && (int)objects_msg.objects.size() == 1) {
        coke_id.assign(objects_msg.objects[0].type.key.c_str());
        firstCB == true;
    }
    for (int i = 0; i < objects_msg.objects.size(); ++i) {
        ROS_INFO("Frame_id 2: %s", objects_msg.objects[i].header.frame_id.c_str());
        ROS_INFO("Frame_id 3: %s", objects_msg.objects[i].pose.header.frame_id.c_str());
        if (coke_id.compare(objects_msg.objects[i].type.key.c_str()) == 0) {
            if (objects_msg.objects[i].confidence > confident) {
                confident = objects_msg.objects[i].confidence;
                id = i;
            }
        }
    }
    coke_pose.pose = objects_msg.objects[id].pose.pose.pose;
    coke_conficent = objects_msg.objects[id].confidence;
    ROS_INFO("Best Similarity = %f ", objects_msg.objects[id].confidence);
    ROS_INFO("pose x is: %f", objects_msg.objects[id].pose.pose.pose.position.x);
    ROS_INFO("pose y is: %f", objects_msg.objects[id].pose.pose.pose.position.y);
    ROS_INFO("pose z is: %f", objects_msg.objects[id].pose.pose.pose.position.z);
    ROS_INFO("---------------------------------");
}

void planeCallback(const object_recognition_msgs::TableArray plane_msg) {
    ROS_WARN("Total %d planes found", (int)plane_msg.tables.size());
    /*
    for (int i = 0; i < plane_msg.tables.size(); ++i) {
        ROS_INFO("Plane %d: plane pose x is: %f", i+1, plane_msg.tables[i].pose.position.x);
        ROS_INFO("Plane %d: plane pose y is: %f", i+1, plane_msg.tables[i].pose.position.y);
        ROS_INFO("Plane %d: plane pose z is: %f", i+1, plane_msg.tables[i].pose.position.z);
        ROS_INFO("---------------------------------");
    }*/

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "coke_grabber"); // name this node
    ros::NodeHandle nh; //standard ros node handle

    ros::Subscriber object_sub = nh.subscribe("/recognized_object_array", 1, &objectCallback);
    ros::Subscriber plane_sub = nh.subscribe("/table_array", 1, &planeCallback);

    BaxterArmCommander arm(nh);
    geometry_msgs::PoseStamped transed_pose;
    tf::TransformListener tf_listener; //start a transform listener
    ros::Duration loop_timer(3.0);

    coke_pose.header.frame_id = "camera_depth_optical_frame";

    while (ros::ok()) {
        if (coke_conficent > 0.9) {
            //stuff a goal message:
            bool tferr = true;
            while (tferr) {
                tferr = false;
                try {
                    tf_listener.transformPose("torso", coke_pose, transed_pose);
                } catch (tf::TransformException &exception) {
                    ROS_ERROR("%s", exception.what());
                    tferr = true;
                    ros::Duration(0.1).sleep(); // sleep for half a second
                    ros::spinOnce();
                }
            }
            ROS_INFO("transformed coke pose x is: %f", transed_pose.pose.position.x);
            ROS_INFO("transformed coke pose y is: %f", transed_pose.pose.position.y);
            ROS_INFO("transformed coke pose z is: %f", transed_pose.pose.position.z);
            ROS_INFO("Grab the coke!");
            arm.grabCoke(transed_pose.pose);
        }
        ros::spinOnce();
        loop_timer.sleep();
    }

    return 0;
}