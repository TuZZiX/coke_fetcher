// object_grabber_as: 
// wsn, April, 2016
// illustrates use of baxter_cart_move_as, action server called "cartMoveActionServer"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_planner/baxter_cart_moveAction.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <object_grabber/object_grabberAction.h>
#include <std_msgs/Bool.h>


//define a class to encapsulate some of the tedium of populating and sending goals,
// and interpreting responses
class ArmMotionCommander {
private:
    ros::NodeHandle nh_;

    //messages to send/receive cartesian goals / results:
    cartesian_planner::baxter_cart_moveGoal cart_goal_;
    cartesian_planner::baxter_cart_moveResult cart_result_;
    std::vector <double> q_vec_; //holder for right-arm angles
    geometry_msgs::PoseStamped tool_pose_stamped_;
    //an action client to send goals to cartesian-move action server
    actionlib::SimpleActionClient<cartesian_planner::baxter_cart_moveAction> cart_move_action_client_; //("cartMoveActionServer", true);
    double computed_arrival_time_;
    bool finished_before_timeout_;
    //callback fnc for cartesian action server to return result to this node:
    void doneCb_(const actionlib::SimpleClientGoalState& state,
                 const cartesian_planner::baxter_cart_moveResultConstPtr& result);
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

};

ArmMotionCommander::ArmMotionCommander(ros::NodeHandle* nodehandle): nh_(*nodehandle),
                                                                     cart_move_action_client_("cartMoveActionServer", true) { // constructor
    ROS_INFO("in constructor of ArmMotionInterface");

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = cart_move_action_client_.waitForServer(ros::Duration(0.5)); //
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to action server"); // if here, then we connected to the server;

}
// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
//int g_return_code=0;
void ArmMotionCommander::doneCb_(const actionlib::SimpleClientGoalState& state,
                                 const cartesian_planner::baxter_cart_moveResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return value= %d", result->return_code);
    cart_result_=*result;
}

Eigen::Affine3d ArmMotionCommander::transformPoseToEigenAffine3d(geometry_msgs::Pose pose) {
    Eigen::Affine3d affine;

    Eigen::Vector3d Oe;

    Oe(0) = pose.position.x;
    Oe(1) = pose.position.y;
    Oe(2) = pose.position.z;
    affine.translation() = Oe;

    Eigen::Quaterniond q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;
    Eigen::Matrix3d Re(q);

    affine.linear() = Re;

    return affine;
}

geometry_msgs::Pose ArmMotionCommander::transformEigenAffine3dToPose(Eigen::Affine3d e) {
    Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;
    geometry_msgs::Pose pose;
    Oe = e.translation();
    Re = e.linear();

    Eigen::Quaterniond q(Re); // convert rotation matrix Re to a quaternion, q
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}


void ArmMotionCommander::send_test_goal(void) {
    ROS_INFO("sending a test goal");
    cart_goal_.command_code = cartesian_planner::baxter_cart_moveGoal::ARM_TEST_MODE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired

    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
    } else {
        ROS_INFO("finished before timeout");
        ROS_INFO("return code: %d",cart_result_.return_code);
    }
}

int ArmMotionCommander::plan_move_to_pre_pose(void) {
    ROS_INFO("requesting a joint-space motion plan");
    cart_goal_.command_code = cartesian_planner::baxter_cart_moveGoal::RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_PRE_POSE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
        return (int) cartesian_planner::baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }

    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cartesian_planner::baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;
    }

    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;
}

int ArmMotionCommander::rt_arm_plan_jspace_path_current_to_qgoal(Eigen::VectorXd q_des_vec) {
    ROS_INFO("requesting a joint-space motion plan");
    cart_goal_.command_code = cartesian_planner::baxter_cart_moveGoal::RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_QGOAL;
    cart_goal_.q_goal_right.resize(7);
    for (int i=0;i<7;i++) cart_goal_.q_goal_right[i] = q_des_vec[i]; //specify the goal js pose
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
        return (int) cartesian_planner::baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }

    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cartesian_planner::baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;
    }

    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;

}

int ArmMotionCommander::rt_arm_plan_path_current_to_goal_pose(geometry_msgs::PoseStamped des_pose) {

    ROS_INFO("requesting a cartesian-space motion plan");
    cart_goal_.command_code = cartesian_planner::baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE;
    cart_goal_.des_pose_gripper_right = des_pose;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
        return (int) cartesian_planner::baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }

    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cartesian_planner::baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;
    }

    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;
}

int ArmMotionCommander::rt_arm_plan_path_current_to_goal_dp_xyz(Eigen::Vector3d dp_displacement) {

    ROS_INFO("requesting a cartesian-space motion plan along vector");
    cart_goal_.command_code = cartesian_planner::baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_DP_XYZ;
    //must fill in desired vector displacement
    cart_goal_.arm_dp_right.resize(3);
    for (int i=0;i<3;i++) cart_goal_.arm_dp_right[i] = dp_displacement[i];
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
        ROS_WARN("giving up waiting on result");
        return (int) cartesian_planner::baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }

    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cartesian_planner::baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;
    }

    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;
}


int ArmMotionCommander::rt_arm_execute_planned_path(void) {
    ROS_INFO("requesting execution of planned path");
    cart_goal_.command_code = cartesian_planner::baxter_cart_moveGoal::RT_ARM_EXECUTE_PLANNED_PATH;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_+2.0));
    if (!finished_before_timeout_) {
        ROS_WARN("did not complete move in expected time");
        return (int) cartesian_planner::baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }
    if (cart_result_.return_code!=cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d",cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    ROS_INFO("move returned success");
    return (int) cart_result_.return_code;
}

//send goal command to request right-arm joint angles; these will be stored in internal variable
int ArmMotionCommander::rt_arm_request_q_data(void) {
    ROS_INFO("requesting right-arm joint angles");
    cart_goal_.command_code = cartesian_planner::baxter_cart_moveGoal::RT_ARM_GET_Q_DATA;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_+2.0));
    if (!finished_before_timeout_) {
        ROS_WARN("did not respond within timeout");
        return (int) cartesian_planner::baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }
    if (cart_result_.return_code!=cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d",cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    q_vec_ = cart_result_.q_arm_right;
    ROS_INFO("move returned success; right arm angles: ");
    ROS_INFO("%f; %f; %f; %f; %f; %f; %f",q_vec_[0],q_vec_[1],q_vec_[2],q_vec_[3],q_vec_[4],q_vec_[5],q_vec_[6]);
    return (int) cart_result_.return_code;
}

Eigen::VectorXd ArmMotionCommander::get_right_arm_joint_angles(void) {
    rt_arm_request_q_data();
    Eigen::VectorXd rt_arm_angs_vecXd;
    rt_arm_angs_vecXd.resize(7);
    for (int i=0;i<7;i++) {
        rt_arm_angs_vecXd[i] = q_vec_[i];
    }
    return rt_arm_angs_vecXd;
}

int ArmMotionCommander::rt_arm_request_tool_pose_wrt_torso(void) {
    // debug: compare this to output of:
    //rosrun tf tf_echo torso yale_gripper_frame
    ROS_INFO("requesting right-arm tool pose");
    cart_goal_.command_code = cartesian_planner::baxter_cart_moveGoal::RT_ARM_GET_TOOL_POSE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    if (!finished_before_timeout_) {
        ROS_WARN("did not respond within timeout");
        return (int) cartesian_planner::baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
    }
    if (cart_result_.return_code!=cartesian_planner::baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d",cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    tool_pose_stamped_ = cart_result_.current_pose_gripper_right;
    ROS_INFO("move returned success; right arm tool pose: ");
    ROS_INFO("origin w/rt torso = %f, %f, %f ",tool_pose_stamped_.pose.position.x,
             tool_pose_stamped_.pose.position.y,tool_pose_stamped_.pose.position.z);
    ROS_INFO("quaternion x,y,z,w: %f, %f, %f, %f",tool_pose_stamped_.pose.orientation.x,
             tool_pose_stamped_.pose.orientation.y,tool_pose_stamped_.pose.orientation.z,
             tool_pose_stamped_.pose.orientation.w);
    return (int) cart_result_.return_code;
}

//define a class to encapsulate some of the tedium of populating and sending goals,
// and interpreting responses
class ObjectGrabber {
private:
    ros::NodeHandle nh_;
    ArmMotionCommander arm_motion_commander;
    //messages to send/receive cartesian goals / results:
    object_grabber::object_grabberGoal grab_goal_;
    object_grabber::object_grabberResult grab_result_;
    object_grabber::object_grabberFeedback grab_fdbk_;
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

    ros::Publisher gripper_publisher;
    ArmMotionCommander *g_arm_motion_commander_ptr;

    actionlib::SimpleActionServer<object_grabber::object_grabberAction> object_grabber_as_;
    //action callback fnc
    void executeCB(const actionlib::SimpleActionServer<object_grabber::object_grabberAction>::GoalConstPtr& goal);
    void vertical_cylinder_power_grasp(geometry_msgs::PoseStamped object_pose);

public:

    ObjectGrabber(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition

    ~ObjectGrabber(void) {
    }
    //define some member methods here

};


//name this server;
ObjectGrabber::ObjectGrabber(ros::NodeHandle* nodehandle): nh_(*nodehandle),
                                                           object_grabber_as_(nh_, "objectGrabberActionServer", boost::bind(&ObjectGrabber::executeCB, this, _1),false),
                                                           arm_motion_commander (nodehandle)
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of ObjectGrabber");
    // do any other desired initializations here, as needed
    gripper_table_z_ = 0.05; //gripper origin height above torso for grasp of cyl on table
    L_approach_ = 0.25; //distance to slide towards cylinder
    z_depart_ = 0.2; //height to lift cylinder
    //define a gripper orientation for power-grasp approach of upright cylinder
    gripper_n_des_ << 0, 0, 1; //gripper x-axis points straight up;
    gripper_theta_ = M_PI/3.0; //approach yaw angle--try this, reaching out and to the left
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
    g_arm_motion_commander_ptr= &arm_motion_commander;

    gripper_open.data= false;
    gripper_close.data=true;
    gripper_publisher = nh_.advertise<std_msgs::Bool>("gripper_open_close",1,true);

    object_grabber_as_.start(); //start the server running
}

void ObjectGrabber::vertical_cylinder_power_grasp(geometry_msgs::PoseStamped object_pose) {
    geometry_msgs::PoseStamped des_gripper_grasp_pose, des_gripper_approach_pose, des_gripper_depart_pose;
    //make sure the object pose is in the torso frame; transform if necessary;
    //skip this for now
    int rtn_val;
    //send a command to plan a joint-space move to pre-defined pose:
    rtn_val = g_arm_motion_commander_ptr->plan_move_to_pre_pose();

    //send command to execute planned motion
    rtn_val=g_arm_motion_commander_ptr->rt_arm_execute_planned_path();

    //inquire re/ right-arm joint angles:
    rtn_val=g_arm_motion_commander_ptr->rt_arm_request_q_data();

    Eigen::Affine3d object_affine;
    object_affine =
            g_arm_motion_commander_ptr->transformPoseToEigenAffine3d(object_pose.pose);
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
    int planner_rtn_code;
    des_gripper_approach_pose.header.frame_id = "torso";
    des_gripper_approach_pose.pose = g_arm_motion_commander_ptr->transformEigenAffine3dToPose(a_gripper_approach_);
    planner_rtn_code = g_arm_motion_commander_ptr->rt_arm_plan_path_current_to_goal_pose(des_gripper_approach_pose);

    //try to move here:
    g_arm_motion_commander_ptr->rt_arm_execute_planned_path();

    //slide to can:
    des_gripper_grasp_pose.header.frame_id = "torso";
    des_gripper_grasp_pose.pose = g_arm_motion_commander_ptr->transformEigenAffine3dToPose(a_gripper_grasp_);
    planner_rtn_code = g_arm_motion_commander_ptr->rt_arm_plan_path_current_to_goal_pose(des_gripper_grasp_pose);
    g_arm_motion_commander_ptr->rt_arm_execute_planned_path();

    //close the gripper:
    gripper_publisher.publish(gripper_close);
    //wait for gripper to close:
    ros::Duration(2.0).sleep();

    //depart vertically:
    des_gripper_depart_pose.header.frame_id = "torso";
    des_gripper_depart_pose.pose = g_arm_motion_commander_ptr->transformEigenAffine3dToPose(a_gripper_depart_);
    planner_rtn_code = g_arm_motion_commander_ptr->rt_arm_plan_path_current_to_goal_pose(des_gripper_depart_pose);
    g_arm_motion_commander_ptr->rt_arm_execute_planned_path();
}


//callback: at present, hard-coded for Coke-can object;
//extend this to add more grasp strategies for more objects
// also, this code does NO error checking (e.g., unreachable); needs to be fixed!
void ObjectGrabber::executeCB(const actionlib::SimpleActionServer<object_grabber::object_grabberAction>::GoalConstPtr& goal) {

    int object_code = goal->object_code;
    geometry_msgs::PoseStamped object_pose = goal->object_frame;
    switch(object_code) {
        case object_grabber::object_grabberGoal::COKE_CAN:
            vertical_cylinder_power_grasp(object_pose);
            grab_result_.return_code = object_grabber::object_grabberResult::OBJECT_ACQUIRED;
            object_grabber_as_.setSucceeded(grab_result_);
            break;
        default:
            ROS_WARN("this object ID is not implemented");
            grab_result_.return_code = object_grabber::object_grabberResult::FAILED_OBJECT_UNKNOWN;
            object_grabber_as_.setAborted(grab_result_);
    }


    //grab_result_.return_code = object_grabber::object_grabberResult::OBJECT_ACQUIRED;

    //object_grabber_as_.setAborted(grab_result_);
    //object_grabber_as_.setSucceeded(grab_result_);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "object_grabber_action_server_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle   
    ObjectGrabber object_grabber_as(&nh); // create an instance of the class "ObjectGrabber", containing an action server
    //arm motion commander is convenient means to talk to cartesian motion action server
    
        ROS_INFO("going into spin");
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
    }

    return 0;

/*
    Eigen::VectorXd right_arm_joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    geometry_msgs::PoseStamped rt_tool_pose;
    
    arm_motion_commander.send_test_goal(); // send a test command
    
    //send a command to plan a joint-space move to pre-defined pose:
    rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    
    //send command to execute planned motion
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    
    //inquire re/ right-arm joint angles:
    rtn_val=arm_motion_commander.rt_arm_request_q_data();
    
    //inquire re/ right-arm tool pose w/rt torso:    
    rtn_val=arm_motion_commander.rt_arm_request_tool_pose_wrt_torso();
    
    //do a joint-space move; get the start angles:
    right_arm_joint_angles = arm_motion_commander.get_right_arm_joint_angles();
    
    //increment all of the joint angles by a fixed amt:
    for (int i=0;i<7;i++) right_arm_joint_angles[i]+=0.2;
    
    //try planning a joint-space motion to this new joint-space pose:
    rtn_val=arm_motion_commander.rt_arm_plan_jspace_path_current_to_qgoal(right_arm_joint_angles);

    //send command to execute planned motion
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();   
    
    //let's see where we ended up...should match goal request
    rtn_val=arm_motion_commander.rt_arm_request_q_data();
    
    //return to pre-defined pose:
    rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();    

    //get tool pose
    rtn_val = arm_motion_commander.rt_arm_request_tool_pose_wrt_torso();
    rt_tool_pose = arm_motion_commander.get_rt_tool_pose_stamped();
    //alter the tool pose:
    std::cout<<"enter 1: ";
    int ans;
    std::cin>>ans;
    //rt_tool_pose.pose.position.z -= 0.2; // descend 20cm, along z in torso frame
    ROS_INFO("translating specified dp");
    rt_tool_pose.pose.position.y += 0.5; // move 20cm, along y in torso frame
    rt_tool_pose.pose.position.x += 0.2; // move 20cm, along x in torso frame
    // send move plan request:
    rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
    //send command to execute planned motion
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    
    //try vector cartesian displacement at fixed orientation:
    std::cout<<"enter delta-z: ";
    double delta_z;
    std::cin>>delta_z;    
    ROS_INFO("moving dz = %f",delta_z);
    dp_displacement<<0,0,delta_z;
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_dp_xyz(dp_displacement);
    if (rtn_val == cartesian_planner::baxter_cart_moveResult::SUCCESS)  { 
            //send command to execute planned motion
           rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    }
 * */

}

