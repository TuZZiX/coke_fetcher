// object_grabber_as: 
// wsn, April, 2016
// illustrates use of baxter_cart_move_as, action server called "cartMoveActionServer"

#include <ArmMotionCommander/ArmMotionCommander.h>
#include <object_grabber/object_grabberAction.h>


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
    gripper_table_z_ = 0.00; //gripper origin height above torso for grasp of cyl on table
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

    gripper_open.data= false;
    gripper_close.data=true;
    gripper_publisher = nh_.advertise<std_msgs::Bool>("gripper_open_close",1,true);

    object_grabber_as_.start(); //start the server running
    arm_motion_commander.plan_move_to_pre_pose();
}

void ObjectGrabber::vertical_cylinder_power_grasp(geometry_msgs::PoseStamped object_pose) {
    geometry_msgs::PoseStamped des_gripper_grasp_pose, des_gripper_approach_pose, des_gripper_depart_pose;
    //make sure the object pose is in the torso frame; transform if necessary;
    //skip this for now
    int rtn_val;
    //send a command to plan a joint-space move to pre-defined pose:
    rtn_val = arm_motion_commander.plan_move_to_pre_pose();

    //send command to execute planned motion
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();

    //inquire re/ right-arm joint angles:
    rtn_val=arm_motion_commander.rt_arm_request_q_data();

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
    int planner_rtn_code;
    des_gripper_approach_pose.header.frame_id = "torso";
    des_gripper_approach_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(a_gripper_approach_);
    planner_rtn_code = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_approach_pose);

    //try to move here:
    arm_motion_commander.rt_arm_execute_planned_path();

    //slide to can:
    des_gripper_grasp_pose.header.frame_id = "torso";
    des_gripper_grasp_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(a_gripper_grasp_);
    planner_rtn_code = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_grasp_pose);
    arm_motion_commander.rt_arm_execute_planned_path();

    //close the gripper:
    gripper_publisher.publish(gripper_close);
    //wait for gripper to close:
    ros::Duration(2.0).sleep();

    //depart vertically:
    des_gripper_depart_pose.header.frame_id = "torso";
    des_gripper_depart_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(a_gripper_depart_);
    planner_rtn_code = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(des_gripper_depart_pose);
    arm_motion_commander.rt_arm_execute_planned_path();
}


//callback: at present, hard-coded for Coke-can object;
//extend this to add more grasp strategies for more objects
// also, this code does NO error checking (e.g., unreachable); needs to be fixed!
void ObjectGrabber::executeCB(const actionlib::SimpleActionServer<object_grabber::object_grabberAction>::GoalConstPtr& goal) {

    int object_code = goal->object_code;
    geometry_msgs::PoseStamped object_pose = goal->object_frame;
    switch(object_code) {
        case object_grabber::object_grabberGoal::COKE_CAN+1:
            arm_motion_commander.plan_move_to_pre_pose();
            break;
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

