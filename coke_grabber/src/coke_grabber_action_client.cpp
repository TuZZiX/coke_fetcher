// example_object_grabber_action_client: 
// wsn, April, 2016
// illustrates use of object_grabber action server called "objectGrabberActionServer"

#include <SimpleCokeFinder/SimpleCokeFinder.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <coke_grabber/coke_grabberAction.h>


void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
        const coke_grabber::coke_grabberResultConstPtr& result) {
    ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got result output = %d; ",result->return_code);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "coke_grabber_action_client"); // name this node
    ros::NodeHandle nh; //standard ros node handle    
    SimpleCokeFinder coke_finder(nh);
    actionlib::SimpleActionClient<coke_grabber::coke_grabberAction> coke_grabber_ac("objectGrabberActionServer", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = coke_grabber_ac.waitForServer(ros::Duration(0.5)); //
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to coke_grabber action server"); // if here, then we connected to the server;

    coke_grabber::coke_grabberGoal coke_grabber_goal;
    coke_grabber::coke_grabberResult coke_grabber_result;
    geometry_msgs::PoseStamped coke_pose;
    double coke_conficent;
    bool finished_before_timeout;

    ros::Duration loop_timer(3.0);

    while (ros::ok()) {
        coke_grabber_goal.object_code = coke_grabber::coke_grabberGoal::MOVE_BACK;
        coke_grabber_ac.sendGoal(coke_grabber_goal,&objectGrabberDoneCb);
        coke_grabber_ac.waitForResult();

        coke_finder.getCokePoseTorso(coke_pose, coke_conficent);

        if (coke_conficent > 0.8)
        {
            ROS_INFO("Best Similarity = %f ", coke_conficent);
            ROS_INFO("tcoke pose x is: %f", coke_pose.pose.position.x);
            ROS_INFO("coke pose y is: %f", coke_pose.pose.position.y);
            ROS_INFO("coke pose z is: %f", coke_pose.pose.position.z);
            ROS_INFO("Grab the coke!");
            coke_grabber_goal.object_code = coke_grabber::coke_grabberGoal::COKE_CAN; //specify the object to be grabbed
            coke_grabber_goal.object_frame = coke_pose;
            ROS_INFO("sending goal: ");
            coke_grabber_ac.sendGoal(coke_grabber_goal, &objectGrabberDoneCb); // we could also name additional callback functions here, if desired
            //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
            finished_before_timeout = coke_grabber_ac.waitForResult();
            //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
            if (!finished_before_timeout) {
                ROS_WARN("giving up waiting on result ");
                return 1;
            }
        }
        ros::spinOnce();
        loop_timer.sleep();
    }

    

        
    return 0;
}

