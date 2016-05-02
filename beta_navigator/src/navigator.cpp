// example_action_server: a simple action server
// Wyatt Newman

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <beta_navigator/navigatorAction.h>
#include <pub_des_state/pub_des_state.h>
#include <mobot_general/mobot_general.h>


class Navigator {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation
    actionlib::SimpleActionServer<beta_navigator::navigatorAction> navigator_as_;
    RobotCommander robot;
    DesStatePublisher desStatePublisher;
    geometry_msgs::Pose g_current_pose;
    
    // here are some message types to communicate with our client(s)
    beta_navigator::navigatorGoal goal_; // goal message, received from client
    beta_navigator::navigatorResult result_; // put results here, to be sent back to the client when done w/ goal
    beta_navigator::navigatorFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client

    int navigate_home();
    int navigate_to_table();
    int navigate_to_pose(geometry_msgs::PoseStamped goal_pose);
    int navigate_to_table2();
    geometry_msgs::Pose home_pose;
    geometry_msgs::Pose table_pose;
    geometry_msgs::Pose table2_pose;


public:
    Navigator(); //define the body of the constructor outside of class definition

    ~Navigator(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<beta_navigator::navigatorAction>::GoalConstPtr& goal);
};



Navigator::Navigator() :
   navigator_as_(nh_, "navigatorActionServer", boost::bind(&Navigator::executeCB, this, _1),false),
   robot(nh_),
   desStatePublisher(nh_){
    ROS_INFO("in constructor of beta_navigator...");
    // do any other desired initializations here...specific to your implementation
    robot.turn(2*M_PI);
    g_current_pose = desStatePublisher.get_odom().pose.pose;
    ROS_INFO("got odometery with x = %f, y = %f, th = %f", g_current_pose.position.x, g_current_pose.position.y, quat2ang(g_current_pose.orientation));
    //desStatePublisher.set_init_pose(g_current_pose.position.x, g_current_pose.position.y, quat2ang(g_current_pose.orientation));
    desStatePublisher.sync_pose();
    desStatePublisher.disable_replanning();
    navigator_as_.start(); //start the server running
    home_pose = g_current_pose;
    table_pose.position.x=1;
    table_pose.position.y=2;
    table_pose.orientation = ang2quat(1/3*M_PI);
    table2_pose.position.x=2;
    table2_pose.position.y=1;
    table2_pose.orientation = ang2quat(2/3*M_PI);

}

//specialized function: DUMMY...JUST RETURN SUCCESS...fix this
//this SHOULD do the hard work of navigating to HOME
int Navigator::navigate_home() {
    geometry_msgs::PoseStamped home_pose_stamped;
    home_pose_stamped.pose = home_pose;
    home_pose_stamped.header.frame_id = "odom_frame";
    home_pose_stamped.header.stamp = ros::Time::now();
    desStatePublisher.append_path_queue(home_pose_stamped);
    while (desStatePublisher.get_motion_mode() != DONE_W_SUBGOAL) {
        if (desStatePublisher.get_motion_mode() == E_STOPPED) {
            desStatePublisher.flush_path();
            return  beta_navigator::navigatorResult::FAILED_BLOCKED;
        }
        desStatePublisher.pub_next_state();
        g_current_pose = desStatePublisher.get_odom().pose.pose;
        ros::spinOnce();
        desStatePublisher.loop_sleep();
    }

    return beta_navigator::navigatorResult::DESIRED_POSE_ACHIEVED; //just say we were successful
} 
int Navigator::navigate_to_table() {
    desStatePublisher.append_path_queue(table_pose.position.x, table_pose.position.y, quat2ang(table_pose.orientation));
    while (desStatePublisher.get_motion_mode() != DONE_W_SUBGOAL){
        if (desStatePublisher.get_motion_mode() == E_STOPPED) {
            desStatePublisher.flush_path();
            return  beta_navigator::navigatorResult::FAILED_BLOCKED;
        }
        desStatePublisher.pub_next_state();
        g_current_pose = desStatePublisher.get_odom().pose.pose;
        ros::spinOnce();
        desStatePublisher.loop_sleep();
    }
    return beta_navigator::navigatorResult::DESIRED_POSE_ACHIEVED; //just say we were successful
}
int Navigator::navigate_to_table2() {
    desStatePublisher.append_path_queue(table2_pose.position.x, table2_pose.position.y, quat2ang(table2_pose.orientation));
    while (desStatePublisher.get_motion_mode() != DONE_W_SUBGOAL){
        if (desStatePublisher.get_motion_mode() == E_STOPPED) {
            desStatePublisher.flush_path();
            return  beta_navigator::navigatorResult::FAILED_BLOCKED;
        }
        desStatePublisher.pub_next_state();
        g_current_pose = desStatePublisher.get_odom().pose.pose;
        ros::spinOnce();
        desStatePublisher.loop_sleep();
    }
    return beta_navigator::navigatorResult::DESIRED_POSE_ACHIEVED; //just say we were successful
}
int Navigator::navigate_to_pose(geometry_msgs::PoseStamped goal_pose) {
    desStatePublisher.append_path_queue(goal_pose);
    while (desStatePublisher.get_motion_mode() != DONE_W_SUBGOAL){
        if (desStatePublisher.get_motion_mode() == E_STOPPED) {
            desStatePublisher.flush_path();
            return  beta_navigator::navigatorResult::FAILED_BLOCKED;
        }
        desStatePublisher.pub_next_state();
        g_current_pose = desStatePublisher.get_odom().pose.pose;
        ros::spinOnce();
        desStatePublisher.loop_sleep();
    }
    return beta_navigator::navigatorResult::DESIRED_POSE_ACHIEVED;
}



void Navigator::executeCB(const actionlib::SimpleActionServer<beta_navigator::navigatorAction>::GoalConstPtr& goal) {
    int destination_id = goal->location_code;
    geometry_msgs::PoseStamped destination_pose;
    int navigation_status;

    if (destination_id==beta_navigator::navigatorGoal::COORDS) {
        destination_pose=goal->desired_pose;
    }
    
    switch(destination_id) {
        case beta_navigator::navigatorGoal::HOME:
              //specialized function to navigate to pre-defined HOME coords
               navigation_status = navigate_home(); 
               if (navigation_status==beta_navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
                   ROS_INFO("reached home");
                   result_.return_code = navigation_status;
                   navigator_as_.setSucceeded(result_);
               }
               else {
                   ROS_WARN("could not navigate home!");
                   result_.return_code = navigation_status;
                   navigator_as_.setAborted(result_);
               }
               break;
        case beta_navigator::navigatorGoal::TABLE_1:
              //specialized function to navigate to pre-defined TABLE coords
               navigation_status = navigate_to_table(); 
               if (navigation_status==beta_navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
                   ROS_INFO("reached table");
                   result_.return_code = navigation_status;
                   navigator_as_.setSucceeded(result_);
               }
               else {
                   ROS_WARN("could not navigate to table!");
                   result_.return_code = navigation_status;
                   navigator_as_.setAborted(result_);
               }
               break;
        case beta_navigator::navigatorGoal::TABLE_2:
              //specialized function to navigate to pre-defined TABLE coords
               navigation_status = navigate_to_table2();
               if (navigation_status==beta_navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
                   ROS_INFO("reached table");
                   result_.return_code = navigation_status;
                   navigator_as_.setSucceeded(result_);
               }
               else {
                   ROS_WARN("could not navigate to table!");
                   result_.return_code = navigation_status;
                   navigator_as_.setAborted(result_);
               }
               break;
        case beta_navigator::navigatorGoal::COORDS: 
              //more general function to navigate to specified pose:
              destination_pose=goal->desired_pose;
               navigation_status = navigate_to_pose(destination_pose); 
               if (navigation_status==beta_navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
                   ROS_INFO("reached desired pose");
                   result_.return_code = navigation_status;
                   navigator_as_.setSucceeded(result_);
               }
               else {
                   ROS_WARN("could not navigate to desired pose!");
                   result_.return_code = navigation_status;
                   navigator_as_.setAborted(result_);
               }
               break;               
               
        default:
             ROS_WARN("this location ID is not implemented");
             result_.return_code = beta_navigator::navigatorResult::DESTINATION_CODE_UNRECOGNIZED; 
             navigator_as_.setAborted(result_);
            }
  
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_action_server"); // name this node 

    ROS_INFO("instantiating the navigation action server: ");

    Navigator navigator_as; // create an instance of the class "ObjectFinder"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        // for debug, induce a halt if we ever get our client/server communications out of sync
    }

    return 0;
}

