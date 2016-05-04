// example_navigator_action_client:
// illustrates use of beta_navigator action server called "navigatorActionServer"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <beta_navigator/navigatorAction.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

geometry_msgs::PoseStamped g_desired_pose;
int g_navigator_rtn_code;
void navigatorDoneCb(const actionlib::SimpleClientGoalState& state,
					 const beta_navigator::navigatorResultConstPtr& result) {
    ROS_INFO(" navigatorDoneCb: server responded with state [%s]", state.toString().c_str());
    g_navigator_rtn_code=result->return_code;
    ROS_INFO("got object code response = %d; ",g_navigator_rtn_code);
    if (g_navigator_rtn_code==beta_navigator::navigatorResult::DESTINATION_CODE_UNRECOGNIZED) {
        ROS_WARN("destination code not recognized");
    } else if (g_navigator_rtn_code==beta_navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
        ROS_INFO("reached desired location!");
    } else {
        ROS_WARN("desired pose not reached!");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_navigator_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    
    actionlib::SimpleActionClient<beta_navigator::navigatorAction> navigator_ac("navigatorActionServer", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    server_exists = navigator_ac.waitForServer(); 
    /*
    while (!server_exists) {
        server_exists = navigator_ac.waitForServer(); // 
        //server_exists = navigator_ac.isServerConnected();
        //ros::Duration(0.5).sleep();
        ros::spinOnce();
        ROS_INFO("retrying...");
    }*/
    ROS_INFO("connected to beta_navigator action server"); // if here, then we connected to the server; 
     
    beta_navigator::navigatorGoal navigation_goal;
    navigation_goal.location_code = beta_navigator::navigatorGoal::TABLE;
    
    ROS_INFO("sending goal: ");
	navigator_ac.sendGoal(navigation_goal,&navigatorDoneCb);
	// we could also name additional callback functions here, if desired

        
	bool finished_before_timeout = navigator_ac.waitForResult(ros::Duration(30.0));
	//bool finished_before_timeout = action_client.waitForResult(); // wait forever...
	if (!finished_before_timeout) {
		ROS_WARN("giving up waiting on result ");
		return 1;
	}
        
    return 0;
}

