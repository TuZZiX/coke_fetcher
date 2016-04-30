// example_object_grabber_action_client: 
// wsn, April, 2016
// illustrates use of object_grabber action server called "objectGrabberActionServer"

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_grabber/object_grabberAction.h>
#include <Eigen/Eigen>  //  for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/TableArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>  //  transform listener headers
#include <tf/transform_broadcaster.h>

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

void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_grabber::object_grabberResultConstPtr& result) {
    ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got result output = %d; ",result->return_code);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_object_grabber_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    

    ros::Subscriber object_sub = nh.subscribe("/recognized_object_array", 1, &objectCallback);
    ros::Subscriber plane_sub = nh.subscribe("/table_array", 1, &planeCallback);
    actionlib::SimpleActionClient<object_grabber::object_grabberAction> object_grabber_ac("objectGrabberActionServer", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_grabber_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_grabber action server"); // if here, then we connected to the server; 
     
    object_grabber::object_grabberGoal object_grabber_goal;
    object_grabber::object_grabberResult object_grabber_result;
    /*
    geometry_msgs::PoseStamped perceived_object_pose;
    perceived_object_pose.header.frame_id = "torso";
    perceived_object_pose.pose.position.x = 0.680;
    perceived_object_pose.pose.position.y = -0.205;
    perceived_object_pose.pose.position.z = 0.047;
    perceived_object_pose.pose.orientation.x = 0.166;
    perceived_object_pose.pose.orientation.y = 0.648;
    perceived_object_pose.pose.orientation.z = 0.702;
    perceived_object_pose.pose.orientation.w = 0.109;*/
    coke_pose.header.frame_id = "camera_depth_optical_frame";

    geometry_msgs::PoseStamped transed_pose;
    tf::TransformListener tf_listener; //start a transform listener

    ros::Duration loop_timer(3.0);

    while (ros::ok()) {
        //stuff a goal message:
        if (coke_conficent > 0.95) {
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
            ROS_INFO("tf is good"); //  tf-listener found a complete chain from sensor to world; ready to roll
            object_grabber_goal.object_code = object_grabber::object_grabberGoal::COKE_CAN; //specify the object to be grabbed
            object_grabber_goal.object_frame = transed_pose;
            ROS_INFO("sending goal: ");
            object_grabber_ac.sendGoal(object_grabber_goal,&objectGrabberDoneCb); // we could also name additional callback functions here, if desired
            //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this

            bool finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(5.0));
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

