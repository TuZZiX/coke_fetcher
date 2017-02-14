// object_grabber_as: 
// wsn, April, 2016
// illustrates use of cart_move_as, action server called "cartMoveActionServer"

#include <ArmMotionCommander/ArmMotionCommander.h>
#include <coke_grabber/coke_grabberAction.h>


//define a class to encapsulate some of the tedium of populating and sending goals,
// and interpreting responses
class ObjectGrabber {
private:
    ros::NodeHandle nh_;
    ArmMotionCommander arm_motion_commander;
    //messages to send/receive cartesian goals / results:
    coke_grabber::coke_grabberGoal grab_goal_;
    coke_grabber::coke_grabberResult grab_result_;
    coke_grabber::coke_grabberFeedback grab_fdbk_;
    geometry_msgs::PoseStamped object_pose_stamped_;
    int object_code_;
    std_msgs::Bool gripper_open,gripper_close;

    double gripper_theta_;
    double z_depart_,L_approach_;
    double gripper_table_z_;
    Eigen::Vector3d gripper_b_des_;
    Eigen::Vector3d gripper_n_des_;
    Eigen::Vector3d gripper_t_des_;
    Eigen::Vector3d grasp_origin_,approach_origin_,depart_origin_;
    Eigen::Matrix3d R_gripper_vert_cyl_grasp_;
    Eigen::Affine3d a_gripper_start_,a_gripper_end_;
    Eigen::Affine3d a_gripper_approach_,a_gripper_depart_, a_gripper_grasp_;

    Eigen::Vector3d pick_offset;
    Eigen::Vector3d hold_offset;
    Eigen::Vector3d pre_grab_offset;
    Eigen::Vector3d grab_offset;
    Eigen::Vector3d give_offset;
    geometry_msgs::Pose coke_pose;

    ros::Publisher gripper_publisher;

    actionlib::SimpleActionServer<coke_grabber::coke_grabberAction> coke_grabber_as_;
    //action callback fnc
    void executeCB(const actionlib::SimpleActionServer<coke_grabber::coke_grabberAction>::GoalConstPtr& goal);
    int grab_coke(geometry_msgs::PoseStamped object_pose);
    int vertical_cylinder_power_grasp(geometry_msgs::PoseStamped object_pose);
    int drop_coke(geometry_msgs::PoseStamped object_pose);
    int give_human();

    geometry_msgs::Pose addPosOffset(geometry_msgs::Pose pose, Eigen::Vector3d offset);
    geometry_msgs::Pose subPosOffset(geometry_msgs::Pose pose, Eigen::Vector3d offset);
    geometry_msgs::Pose addPose(geometry_msgs::Pose pose_a, geometry_msgs::Pose pose_b);

public:

    ObjectGrabber(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition

    ~ObjectGrabber(void) {
    }
    //define some member methods here

};


//name this server;
ObjectGrabber::ObjectGrabber(ros::NodeHandle* nodehandle): nh_(*nodehandle),
                                                           coke_grabber_as_(nh_, "objectGrabberActionServer", boost::bind(&ObjectGrabber::executeCB, this, _1),false),
                                                           arm_motion_commander (nodehandle)
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of ObjectGrabber");
    // do any other desired initializations here, as needed
    gripper_table_z_ = 0; //gripper origin height above torso for grasp of cyl on table
    L_approach_ = 0.25; //distance to slide towards cylinder
    z_depart_ = 0.2; //height to lift cylinder

    //define a gripper orientation for power-grasp approach of upright cylinder
    gripper_n_des_ << 0, 0, 1; //gripper x-axis points straight up;
    gripper_theta_ = M_PI/2.0; //approach yaw angle--try this, reaching out and to the left
    gripper_b_des_ << cos(gripper_theta_), sin(gripper_theta_), 0;
    gripper_t_des_ = gripper_b_des_.cross(gripper_n_des_);
    R_gripper_vert_cyl_grasp_.col(0) = gripper_n_des_;
    R_gripper_vert_cyl_grasp_.col(1) = gripper_t_des_;
    R_gripper_vert_cyl_grasp_.col(2) = gripper_b_des_;
    a_gripper_start_.linear() = R_gripper_vert_cyl_grasp_;
    a_gripper_end_.linear() = R_gripper_vert_cyl_grasp_;
    //define approach, grasp and depart poses:
    a_gripper_approach_.linear() = R_gripper_vert_cyl_grasp_;
    a_gripper_depart_.linear() = R_gripper_vert_cyl_grasp_;
    a_gripper_grasp_.linear() = R_gripper_vert_cyl_grasp_;

    gripper_open.data= false;
    gripper_close.data=true;
    gripper_publisher = nh_.advertise<std_msgs::Bool>("gripper_open_close",1,true);

    coke_grabber_as_.start(); //start the server running
    arm_motion_commander.plan_move_to_pre_pose();

    hold_offset << 0, -0.2, 0.3;
    pre_grab_offset << 0, -0.2, 0;
    grab_offset << 0, 0, 0;
    pick_offset << 0, 0, 0.3;
    give_offset << 0, 0, 0.3;

    coke_pose.position.x = 0.01;
    coke_pose.position.y = -0.08;
    coke_pose.position.z = -0.05;
    coke_pose.orientation.x = -0.708866454238;
    coke_pose.orientation.y = 0.17589525363;
    coke_pose.orientation.z = 0.135739113146;
    coke_pose.orientation.w = 0.669435660067;
}
geometry_msgs::Pose ObjectGrabber::addPose(geometry_msgs::Pose pose_a, geometry_msgs::Pose pose_b) {
    geometry_msgs::Pose result;
    result.position.x=pose_a.position.x+pose_b.position.x;
    result.position.y=pose_a.position.y+pose_b.position.y;
    result.position.z=pose_a.position.z+pose_b.position.z;

    result.orientation.x=pose_b.orientation.x;
    result.orientation.y=pose_b.orientation.y;
    result.orientation.z=pose_b.orientation.z;
    result.orientation.w=pose_b.orientation.w;
    return result;
}
geometry_msgs::Pose ObjectGrabber::addPosOffset(geometry_msgs::Pose pose, Eigen::Vector3d offset) {
    geometry_msgs::Pose result;
    result.position.x=pose.position.x+offset[0];
    result.position.y=pose.position.y+offset[1];
    result.position.z=pose.position.z+offset[2];
    result.orientation=pose.orientation;
    return result;
}
geometry_msgs::Pose ObjectGrabber::subPosOffset(geometry_msgs::Pose pose, Eigen::Vector3d offset) {
    geometry_msgs::Pose result;
    result.position.x=pose.position.x-offset[0];
    result.position.y=pose.position.y-offset[1];
    result.position.z=pose.position.z-offset[2];
    result.orientation=pose.orientation;
    return result;
}
int ObjectGrabber::vertical_cylinder_power_grasp(geometry_msgs::PoseStamped object_pose) {
    geometry_msgs::PoseStamped des_gripper_grasp_pose, des_gripper_approach_pose, des_gripper_depart_pose, des_gripper_hold_pose;
    //make sure the object pose is in the torso frame; transform if necessary;
    //skip this for now
    int rtn_val;
    //send a command to plan a joint-space move to pre-defined pose:
    rtn_val = arm_motion_commander.plan_move_to_pre_pose();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_INFO("plan_move_to_pre_pose plan failed");
        return rtn_val;
    }
    //send command to execute planned motion
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_INFO("plan_move_to_pre_pose exe failed");
        return rtn_val;
    }
    //inquire re/ right-arm joint angles:
    rtn_val=arm_motion_commander.rt_arm_request_q_data();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_INFO("rt_arm_request_q_data failed");
        return rtn_val;
    }
    Eigen::Affine3d object_affine;
    object_affine =
            arm_motion_commander.transformPoseToEigenAffine3d(object_pose.pose);
    Eigen::Vector3d object_origin;
    object_origin = object_affine.translation();
    grasp_origin_ = object_origin; //grasp origin is same as object origin...
    grasp_origin_(2) = gripper_table_z_;//except elevate the gripper for table clearance
    a_gripper_grasp_.translation() = grasp_origin_;

    //to slide sideways to approach, compute a pre-grasp approach pose;
    // corresponds to backing up along gripper-z axis by distance L_approach:
    approach_origin_ = grasp_origin_ - gripper_b_des_*L_approach_;
    a_gripper_approach_.translation() = approach_origin_;

    // after have cylinder grasped, move purely upwards by z_depart:
    depart_origin_ = grasp_origin_ + gripper_n_des_*z_depart_;
    a_gripper_depart_.translation() = depart_origin_;

    //open the gripper:
    gripper_publisher.publish(gripper_open);

    //start w/ a jnt-space move from current pose to approach pose:
    des_gripper_approach_pose.header.frame_id = "torso";
    des_gripper_approach_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(a_gripper_approach_);
    des_gripper_approach_pose.pose = addPose(des_gripper_approach_pose.pose, coke_pose);
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_approach_pose);
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_INFO("des_gripper_approach_pose plan failed");
        return rtn_val;
    }
    //try to move here:
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_INFO("des_gripper_approach_pose exe failed");
        return rtn_val;
    }
    //slide to can:
    des_gripper_grasp_pose.header.frame_id = "torso";
    des_gripper_grasp_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(a_gripper_grasp_);
    des_gripper_grasp_pose.pose = addPose(des_gripper_grasp_pose.pose, coke_pose);
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_grasp_pose);
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_INFO("des_gripper_grasp_pose plan failed");
        return rtn_val;
    }
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_INFO("des_gripper_grasp_pose exe failed");
        return rtn_val;
    }
    //close the gripper:
    gripper_publisher.publish(gripper_close);
    //wait for gripper to close:
    ros::Duration(2.0).sleep();

    //depart vertically:
    des_gripper_depart_pose.header.frame_id = "torso";
    des_gripper_depart_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(a_gripper_depart_);
    des_gripper_depart_pose.pose = addPose(des_gripper_depart_pose.pose, coke_pose);
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_depart_pose);
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_INFO("des_gripper_depart_pose plan failed");
        return rtn_val;
    }
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_INFO("des_gripper_depart_pose exe failed");
        return rtn_val;
    }
    des_gripper_hold_pose.header.frame_id = "torso";
    des_gripper_hold_pose.pose = addPosOffset(des_gripper_grasp_pose.pose, hold_offset);
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_hold_pose);
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_INFO("des_gripper_hold_pose plan failed");
        return rtn_val;
    }
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        ROS_INFO("des_gripper_hold_pose exe failed");
        return rtn_val;
    }
    return rtn_val;
}

int ObjectGrabber::grab_coke(geometry_msgs::PoseStamped object_pose) {
    geometry_msgs::PoseStamped des_gripper_grasp_pose, des_gripper_approach_pose, des_gripper_depart_pose, des_gripper_hold_pose;
    //make sure the object pose is in the torso frame; transform if necessary;
    //skip this for now
    int rtn_val;
    //send a command to plan a joint-space move to pre-defined pose:
    rtn_val = arm_motion_commander.plan_move_to_pre_pose();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    //send command to execute planned motion
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    //inquire re/ right-arm joint angles:
    rtn_val=arm_motion_commander.rt_arm_request_q_data();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    gripper_publisher.publish(gripper_open);

    //start w/ a jnt-space move from current pose to approach pose:
    des_gripper_approach_pose.header.frame_id = "torso";
    des_gripper_grasp_pose.header.frame_id = "torso";
    des_gripper_depart_pose.header.frame_id = "torso";
    des_gripper_hold_pose.header.frame_id = "torso";

    object_pose.pose = addPose(object_pose.pose, coke_pose);

    des_gripper_grasp_pose.pose = addPosOffset(object_pose.pose, grab_offset);
    des_gripper_approach_pose.pose = addPosOffset(object_pose.pose, pre_grab_offset);
    des_gripper_depart_pose.pose = addPosOffset(object_pose.pose, pick_offset);
    des_gripper_hold_pose.pose = addPosOffset(object_pose.pose, hold_offset);

    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_hold_pose);
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_approach_pose);
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    //try to move here:
    rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    //slide to can:
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_grasp_pose);
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    //close the gripper:
    gripper_publisher.publish(gripper_close);
    //wait for gripper to close:
    ros::Duration(2.0).sleep();

    //depart vertically:
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_depart_pose);
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_hold_pose);
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    return rtn_val;
}
int ObjectGrabber::drop_coke(geometry_msgs::PoseStamped object_pose) {
    geometry_msgs::PoseStamped des_gripper_grasp_pose, des_gripper_approach_pose, des_gripper_depart_pose, des_gripper_hold_pose;
    //make sure the object pose is in the torso frame; transform if necessary;
    //skip this for now
    int rtn_val;

    Eigen::Affine3d object_affine;
    object_affine = arm_motion_commander.transformPoseToEigenAffine3d(object_pose.pose);
    Eigen::Vector3d object_origin;
    object_origin = object_affine.translation();
    grasp_origin_ = object_origin; //grasp origin is same as object origin...
    grasp_origin_(2) = gripper_table_z_;//except elevate the gripper for table clearance
    a_gripper_grasp_.translation() = grasp_origin_;

    //to slide sideways to approach, compute a pre-grasp approach pose;
    // corresponds to backing up along gripper-z axis by distance L_approach:
    approach_origin_ = grasp_origin_ - gripper_b_des_*L_approach_;
    a_gripper_approach_.translation() = approach_origin_;

    // after have cylinder grasped, move purely upwards by z_depart:
    depart_origin_ = grasp_origin_ + gripper_n_des_*z_depart_;
    a_gripper_depart_.translation() = depart_origin_;

    //start w/ a jnt-space move from current pose to approach pose:
    des_gripper_depart_pose.header.frame_id = "torso";
    des_gripper_depart_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(a_gripper_depart_);
    des_gripper_depart_pose.pose = addPose(des_gripper_depart_pose.pose, coke_pose);
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_depart_pose);
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    //try to move here:
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    //slide to can:
    des_gripper_grasp_pose.header.frame_id = "torso";
    des_gripper_grasp_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(a_gripper_grasp_);
    des_gripper_grasp_pose.pose = addPose(des_gripper_grasp_pose.pose, coke_pose);
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_grasp_pose);
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    //close the gripper:
    gripper_publisher.publish(gripper_open);
    //wait for gripper to close:
    ros::Duration(2.0).sleep();

    //depart vertically:
    des_gripper_depart_pose.header.frame_id = "torso";
    des_gripper_depart_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(a_gripper_depart_);
    des_gripper_depart_pose.pose = addPose(des_gripper_depart_pose.pose, coke_pose);
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_depart_pose);
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    rtn_val = arm_motion_commander.plan_move_to_pre_pose();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    //send command to execute planned motion
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    //inquire re/ right-arm joint angles:
    rtn_val=arm_motion_commander.rt_arm_request_q_data();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    return rtn_val;
}
int ObjectGrabber::give_human() {
    /*
    geometry_msgs::PoseStamped des_gripper_grasp_pose, des_gripper_approach_pose, des_gripper_depart_pose, des_gripper_hold_pose;
    //make sure the object pose is in the torso frame; transform if necessary;
    //skip this for now
    int rtn_val;

    Eigen::Affine3d object_affine;
    object_affine = arm_motion_commander.transformPoseToEigenAffine3d(object_pose.pose);
    Eigen::Vector3d object_origin;
    object_origin = object_affine.translation();
    grasp_origin_ = object_origin; //grasp origin is same as object origin...
    grasp_origin_(2) = gripper_table_z_;//except elevate the gripper for table clearance
    a_gripper_grasp_.translation() = grasp_origin_;

    //to slide sideways to approach, compute a pre-grasp approach pose;
    // corresponds to backing up along gripper-z axis by distance L_approach:
    approach_origin_ = grasp_origin_ - gripper_b_des_*L_approach_;
    a_gripper_approach_.translation() = approach_origin_;

    // after have cylinder grasped, move purely upwards by z_depart:
    depart_origin_ = grasp_origin_ + gripper_n_des_*z_depart_;
    a_gripper_depart_.translation() = depart_origin_;

    //start w/ a jnt-space move from current pose to approach pose:
    des_gripper_depart_pose.header.frame_id = "torso";
    des_gripper_depart_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(a_gripper_depart_);
    des_gripper_depart_pose.pose = addPose(des_gripper_depart_pose.pose, coke_pose);
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_depart_pose);
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    //try to move here:
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }

    //depart vertically:
    des_gripper_depart_pose.header.frame_id = "torso";
    des_gripper_depart_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(a_gripper_depart_);
    des_gripper_depart_pose.pose = addPose(des_gripper_depart_pose.pose, coke_pose);
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_depart_pose);
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    //close the gripper:
    gripper_publisher.publish(gripper_open);
    //wait for gripper to close:
    ros::Duration(2.0).sleep();

    rtn_val = arm_motion_commander.plan_move_to_pre_pose();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    //send command to execute planned motion
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }
    //inquire re/ right-arm joint angles:
    rtn_val=arm_motion_commander.rt_arm_request_q_data();
    if (rtn_val!=cartesian_planner::cart_moveResult::SUCCESS) {
        return rtn_val;
    }*/
    gripper_publisher.publish(gripper_open);
    //wait for gripper to close:
    ros::Duration(2.0).sleep();    
    int rtn_val = cartesian_planner::cart_moveResult::SUCCESS;
    return rtn_val;
}
//callback: at present, hard-coded for Coke-can object;
//extend this to add more grasp strategies for more objects
// also, this code does NO error checking (e.g., unreachable); needs to be fixed!
void ObjectGrabber::executeCB(const actionlib::SimpleActionServer<coke_grabber::coke_grabberAction>::GoalConstPtr& goal) {
    int ret;
    int object_code = goal->object_code;
    geometry_msgs::PoseStamped object_pose = goal->object_frame;
    switch(object_code) {
        case coke_grabber::coke_grabberGoal::MOVE_BACK:
            ret = arm_motion_commander.plan_move_to_pre_pose();
            if (ret == cartesian_planner::cart_moveResult::SUCCESS) {
                grab_result_.return_code = coke_grabber::coke_grabberResult::OBJECT_ACQUIRED;
                coke_grabber_as_.setSucceeded(grab_result_);
            } else if (ret == cartesian_planner::cart_moveResult::PATH_NOT_VALID) {
                grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_CANNOT_REACH;
                coke_grabber_as_.setAborted(grab_result_);
            } else if (ret == cartesian_planner::cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT) {
                grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_TIMEOUT;
                coke_grabber_as_.setAborted(grab_result_);
            } else {
                grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_UNKNOWN;
                coke_grabber_as_.setAborted(grab_result_);
            }
            break;
        case coke_grabber::coke_grabberGoal::COKE_CAN:
        ROS_INFO("grab COKE_CAN");
            ret = vertical_cylinder_power_grasp(object_pose);
            //grab_coke(object_pose);
            if (ret == cartesian_planner::cart_moveResult::SUCCESS) {
                grab_result_.return_code = coke_grabber::coke_grabberResult::OBJECT_ACQUIRED;
                coke_grabber_as_.setSucceeded(grab_result_);
            } else if (ret == cartesian_planner::cart_moveResult::PATH_NOT_VALID) {
                grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_CANNOT_REACH;
                coke_grabber_as_.setAborted(grab_result_);
            } else if (ret == cartesian_planner::cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT) {
                grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_TIMEOUT;
                coke_grabber_as_.setAborted(grab_result_);
            } else {
                grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_UNKNOWN;
                coke_grabber_as_.setAborted(grab_result_);
            }
            break;
        case coke_grabber::coke_grabberGoal::DROP_COKE:
            ret = drop_coke(object_pose);
            if (ret == cartesian_planner::cart_moveResult::SUCCESS) {
                grab_result_.return_code = coke_grabber::coke_grabberResult::OBJECT_ACQUIRED;
                coke_grabber_as_.setSucceeded(grab_result_);
            } else if (ret == cartesian_planner::cart_moveResult::PATH_NOT_VALID) {
                grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_CANNOT_REACH;
                coke_grabber_as_.setAborted(grab_result_);
            } else if (ret == cartesian_planner::cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT) {
                grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_TIMEOUT;
                coke_grabber_as_.setAborted(grab_result_);
            } else {
                grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_UNKNOWN;
                coke_grabber_as_.setAborted(grab_result_);
            }
            break;
        case coke_grabber::coke_grabberGoal::GIVE_TO_HUMAN:
            ret = give_human();
            if (ret == cartesian_planner::cart_moveResult::SUCCESS) {
                grab_result_.return_code = coke_grabber::coke_grabberResult::OBJECT_ACQUIRED;
                coke_grabber_as_.setSucceeded(grab_result_);
            } else if (ret == cartesian_planner::cart_moveResult::PATH_NOT_VALID) {
                grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_CANNOT_REACH;
                coke_grabber_as_.setAborted(grab_result_);
            } else if (ret == cartesian_planner::cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT) {
                grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_TIMEOUT;
                coke_grabber_as_.setAborted(grab_result_);
            } else {
                grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_UNKNOWN;
                coke_grabber_as_.setAborted(grab_result_);
            }
            break;
        case coke_grabber::coke_grabberGoal::RIGHT_TO_POSE:
            ret = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(object_pose);
            if (ret==cartesian_planner::cart_moveResult::SUCCESS) {
                ret=arm_motion_commander.rt_arm_execute_planned_path();
            }
            if (ret == cartesian_planner::cart_moveResult::SUCCESS) {
                grab_result_.return_code = coke_grabber::coke_grabberResult::OBJECT_ACQUIRED;
                coke_grabber_as_.setSucceeded(grab_result_);
            } else if (ret == cartesian_planner::cart_moveResult::PATH_NOT_VALID) {
                grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_CANNOT_REACH;
                coke_grabber_as_.setAborted(grab_result_);
            } else if (ret == cartesian_planner::cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT) {
                grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_TIMEOUT;
                coke_grabber_as_.setAborted(grab_result_);
            } else {
                grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_UNKNOWN;
                coke_grabber_as_.setAborted(grab_result_);
            }
            break;
        default:
            ROS_WARN("this object ID is not implemented");
            grab_result_.return_code = coke_grabber::coke_grabberResult::FAILED_OBJECT_UNKNOWN;
            coke_grabber_as_.setAborted(grab_result_);
    }


    //grab_result_.return_code = coke_grabber::coke_grabberResult::OBJECT_ACQUIRED;

    //coke_grabber_as_.setAborted(grab_result_);
    //coke_grabber_as_.setSucceeded(grab_result_);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "coke_grabber_action_server_node"); // name this node
    ros::NodeHandle nh; //standard ros node handle   
    ObjectGrabber coke_grabber_as(&nh); // create an instance of the class "ObjectGrabber", containing an action server
    ROS_INFO("going into spin");
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
    }

    return 0;
}