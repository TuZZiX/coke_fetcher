//
// Created by sxt437 on 5/1/16.
//

#ifndef COKE_GRABBER_SIMPLECOKEFINDER_H
#define COKE_GRABBER_SIMPLECOKEFINDER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/TableArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>  //  transform listener headers
#include <tf/transform_broadcaster.h>
#include <BaxterArmCommander/BaxterArmCommander.h>

class SimpleCokeFinder {
public:
    SimpleCokeFinder(ros::NodeHandle &nodehandle);

    bool getCokePoseTorso(geometry_msgs::PoseStamped &coke_pose, double &confidence);
    bool getCokePoseKinect(geometry_msgs::PoseStamped &coke_pose, double &confidence);

    bool getTableTorso(geometry_msgs::PoseStamped &table_pose, Eigen::Vector3d &start_cord, Eigen::Vector3d &end_cord);
    bool getTableKinect(geometry_msgs::PoseStamped &table_pose, Eigen::Vector3d &start_cord, Eigen::Vector3d &end_cord);

private:
    ros::NodeHandle nh_;
    std::string coke_id;
    double coke_confidence;
    geometry_msgs::PoseStamped coke_pose;
    geometry_msgs::PoseStamped transed_pose;
    tf::TransformListener tf_listener; //start a transform listener
    bool firstCB;
    ros::Subscriber object_sub;
    ros::Subscriber plane_sub;

    void objectCallback(const object_recognition_msgs::RecognizedObjectArray objects_msg);
    void planeCallback(const object_recognition_msgs::TableArray plane_msg);
};


#endif //COKE_GRABBER_SIMPLECOKEFINDER_H
