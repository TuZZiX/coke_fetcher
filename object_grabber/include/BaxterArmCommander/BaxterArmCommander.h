//
// Created by sxt437 on 4/30/16.
//

#ifndef COKE_GRABBER_BAXTERARMCOMMANDER_H
#define COKE_GRABBER_BAXTERARMCOMMANDER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <object_grabber/object_grabberAction.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <object_grabber/object_grabberAction.h>
#include <std_msgs/Bool.h>
#include <cartesian_planner/baxter_cart_moveAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <BaxterGripper/BaxterGripper.h>

using namespace std;  //just to avoid requiring std::,  ...
using namespace Eigen;
typedef Matrix<double, 7, 1> Vector7d;

class BaxterArmCommander {
public:
    BaxterArmCommander(ros::NodeHandle &nodehandle);

    bool rightArmBack(void);
    Vector7d rightGetJoints(void);
    geometry_msgs::Pose rightGetGripperPose(void);
    void rightShowPath();
    bool rightPlan(geometry_msgs::Pose pose);
    bool rightPlan(Vector7d joints);
    bool rightPlanOffset(Vector3d offset);
    bool rightPlan(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid);
    bool rightExecute();
    bool rightExecuteAsync();
    void rightStop();
    void rightGrab();
    void rightRelease();


    bool leftArmBack(void);

    Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::Pose pose);
    geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);
    geometry_msgs::Pose normalToPose(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid);
    geometry_msgs::Pose addPosOffset(geometry_msgs::Pose pose, Vector3d offset);
    geometry_msgs::Pose subPosOffset(geometry_msgs::Pose pose, Vector3d offset);
    geometry_msgs::Pose addPose(geometry_msgs::Pose pose_a, geometry_msgs::Pose pose_b);
    std::vector<double> quat2euler(geometry_msgs::Quaternion quaternion);

private:
    ros::NodeHandle nh_;
    BaxterGripper right_gripper;

    ros::AsyncSpinner spinner;
    moveit::planning_interface::MoveGroup right_arm, left_arm;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroup::Plan my_plan;
    ros::Publisher display_publisher;
    moveit_msgs::DisplayTrajectory display_trajectory;

    geometry_msgs::Pose right_arm_back_pose;
    geometry_msgs::Pose left_arm_back_pose;

    Vector3d gripper_offset;
    Vector3d drop_offset_left;
    Vector3d drop_offset_right;

    geometry_msgs::Pose global_pose_offset;
};


#endif //COKE_GRABBER_BAXTERARMCOMMANDER_H
