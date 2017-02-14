//
// Created by sxt437 on 5/1/16.
//

#ifndef COKE_GRABBER_ARMMOTIONCOMMANDER_H
#define COKE_GRABBER_ARMMOTIONCOMMANDER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <cartesian_planner/cart_moveAction.h>

//define a class to encapsulate some of the tedium of populating and sending goals,
// and interpreting responses
class ArmMotionCommander {
private:
    ros::NodeHandle nh_;

    //messages to send/receive cartesian goals / results:
    cartesian_planner::cart_moveGoal cart_goal_;
    cartesian_planner::cart_moveResult cart_result_;
    std::vector <double> q_vec_; //holder for right-arm angles
    geometry_msgs::PoseStamped tool_pose_stamped_;
    //an action client to send goals to cartesian-move action server
    actionlib::SimpleActionClient<cartesian_planner::cart_moveAction> cart_move_action_client_; //("cartMoveActionServer", true);
    double computed_arrival_time_;
    bool finished_before_timeout_;
    //callback fnc for cartesian action server to return result to this node:
    void doneCb_(const actionlib::SimpleClientGoalState& state,
                 const cartesian_planner::cart_moveResultConstPtr& result);
public:
    ArmMotionCommander(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition

    ~ArmMotionCommander(void) {
    }
    void send_test_goal(void);
    int plan_move_to_pre_pose(void);
    int rt_arm_execute_planned_path(void);
    int rt_arm_request_q_data(void);
    int rt_arm_request_tool_pose_wrt_torso(void);
    geometry_msgs::PoseStamped get_rt_tool_pose_stamped(void) { return tool_pose_stamped_;};

    Eigen::VectorXd get_right_arm_joint_angles(void);
    int rt_arm_plan_jspace_path_current_to_qgoal(Eigen::VectorXd q_des_vec);
    int rt_arm_plan_path_current_to_goal_pose(geometry_msgs::PoseStamped des_pose);
    int rt_arm_plan_path_current_to_goal_dp_xyz(Eigen::Vector3d dp_displacement);

    //utilities to convert between affine and pose
    Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::Pose pose);
    geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);
    double planning_time;
};

#endif //COKE_GRABBER_ARMMOTIONCOMMANDER_H
