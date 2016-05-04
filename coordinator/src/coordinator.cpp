// coordinator: 
// wsn, April, 2016
// illustrates use of object_finder, object_grasper and navigator action servers

//trigger this process with:
//rostopic pub Alexa_codes std_msgs/UInt32 100

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_finder/objectFinderAction.h>
#include <coke_grabber/coke_grabberAction.h>
#include <beta_navigator/navigatorAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_msgs/UInt32.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/TableArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>  //  transform listener headers
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h> 

geometry_msgs::PoseStamped g_perceived_object_pose;
int g_found_object_code;
int g_coke_grabber_return_code;
int g_navigator_rtn_code;
bool g_get_coke_trigger=false;

const int ALEXA_GET_COKE_CODE= 100;
std::string coke_id = "";
double coke_confidence = 0;
geometry_msgs::PoseStamped coke_pose;
bool firstCB = false;
ros::Publisher pub;

void publishToScreen(ros::NodeHandle &nh, std::string path){
//	image_transport::ImageTransport it(nh);
	std::string pkg_path = ros::package::getPath("coke_fetcher");
	std::string append = "/img/";
	std::string full_path = pkg_path+append+path;
	ROS_INFO("%s",full_path.c_str());
	cv::Mat image = cv::imread(full_path, CV_LOAD_IMAGE_COLOR);
	cv::waitKey(30);
	sensor_msgs::ImagePtr msg;
	msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	pub.publish(*msg);
	//ros::Duration(1.0).sleep();
	ros::spinOnce();
}


void objectCallback(const object_recognition_msgs::RecognizedObjectArray objects_msg) {
	double confident = 0;
	int id = -1;
	ROS_WARN("Total %d objects found", (int)objects_msg.objects.size());
    //ROS_INFO("Frame_id 1: %s", objects_msg.header.frame_id.c_str());

	if (firstCB == false && (int)objects_msg.objects.size() == 1) {
		coke_id.assign(objects_msg.objects[0].type.key.c_str());
		firstCB = true;
	}
	for (int i = 0; i < objects_msg.objects.size(); ++i) {
		if (coke_id.compare(objects_msg.objects[i].type.key.c_str()) == 0) {
			if (objects_msg.objects[i].confidence > confident) {
				confident = objects_msg.objects[i].confidence;
				id = i;
			}
		}
	}
	if (id >= 0) {
		coke_pose.pose = objects_msg.objects[id].pose.pose.pose;
		coke_confidence = objects_msg.objects[id].confidence;
	} else {
		confident = 0;
	}

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
    void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
    	const object_finder::objectFinderResultConstPtr& result) {
    	ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    	g_found_object_code=result->found_object_code;
    	ROS_INFO("got object code response = %d; ",g_found_object_code);
    	if (g_found_object_code==object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED) {
    		ROS_WARN("object code not recognized");
    	}
    	else if (g_found_object_code==object_finder::objectFinderResult::OBJECT_FOUND) {
    		ROS_INFO("found object!");
    		g_perceived_object_pose= result->object_pose;
    		ROS_INFO("got pose x,y,z = %f, %f, %f",
    			g_perceived_object_pose.pose.position.x,
    			g_perceived_object_pose.pose.position.y,
    			g_perceived_object_pose.pose.position.z);
    	}
    	else {
    		ROS_WARN("object not found!");
    	}
    }

    void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
    	const coke_grabber::coke_grabberResultConstPtr& result) {
    	ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
    	ROS_INFO("got result output = %d; ",result->return_code);
    	g_coke_grabber_return_code = result->return_code;
    }


    void navigatorDoneCb(const actionlib::SimpleClientGoalState& state,
    	const beta_navigator::navigatorResultConstPtr& result) {
    	ROS_INFO(" navigatorDoneCb: server responded with state [%s]", state.toString().c_str());
    	g_navigator_rtn_code = result->return_code;
    	ROS_INFO("got object code response = %d; ", g_navigator_rtn_code);
    	switch (g_navigator_rtn_code) {
    		case beta_navigator::navigatorResult::DESTINATION_CODE_UNRECOGNIZED:
	    		ROS_WARN("destination code not recognized");
	    		break;
    		case beta_navigator::navigatorResult::DESIRED_POSE_ACHIEVED:
	    		ROS_INFO("reached desired location!");
	    		break;
    		default:
    			ROS_WARN("desired pose not reached!");
    	}
    }

//external trigger:
    void alexaCB(const std_msgs::UInt32& code_msg) {
    	int alexa_code = code_msg.data;
    	ROS_INFO("received Alexa code: %d", alexa_code);
	//BUGFIX this used to use a single equals sign
    	if (alexa_code == ALEXA_GET_COKE_CODE) {
    		g_get_coke_trigger = true;
    	}
    }

    int main(int argc, char** argv) {
    ros::init(argc, argv, "coordinator"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    
    ros::Subscriber alexa_code = nh.subscribe("/Alexa_codes", 1, alexaCB);
    ros::Subscriber object_sub = nh.subscribe("/recognized_object_array", 1, &objectCallback);
    ros::Subscriber plane_sub = nh.subscribe("/table_array", 1, &planeCallback);
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("robot/xdisplay", 10, true);

    pub = img_pub;

    //actionlib::SimpleActionClient<object_finder::objectFinderAction>
    //object_finder_ac("objectFinderActionServer", true);
    actionlib::SimpleActionClient<coke_grabber::coke_grabberAction>
    coke_grabber_ac("objectGrabberActionServer", true);
    actionlib::SimpleActionClient<beta_navigator::navigatorAction>
    navigator_ac("navigatorActionServer", true);

    publishToScreen(nh, "baxter.jpg");

    coke_pose.header.frame_id = "camera_depth_optical_frame";
    geometry_msgs::PoseStamped transed_pose;
    tf::TransformListener tf_listener; //start a transform listener
    // attempt to connect to the server:
    //ROS_INFO("waiting for server: ");
    bool server_exists = false;
    //server_exists = object_finder_ac.waitForServer();
    /*
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_finder_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");g_found_object_code
    }*/
    ROS_INFO("connected to object_finder action server"); // if here, then we connected to the server; 
    //connect to the coke_grabber server
    server_exists=false;
    server_exists = coke_grabber_ac.waitForServer();
    /*
    while ((!server_exists)&&(ros::ok())) {
        server_exists = coke_grabber_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }*/
    ROS_INFO("connected to coke_grabber action server"); // if here, then we connected to the server; 
    
    //do the same with the "navigator" action server
	// attempt to connect to the server:
    ROS_INFO("waiting for nav server: ");
    server_exists = false;
    server_exists = navigator_ac.waitForServer();
    /*
    while ((!server_exists) && (ros::ok())) {
        server_exists = navigator_ac.waitForServer(ros::Duration(0.5));
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }*/
        ROS_INFO("connected to navigator action server");

    //specifications for what we are seeking: 
        coke_grabber::coke_grabberGoal coke_grabber_goal;
        beta_navigator::navigatorGoal navigation_goal;

        bool finished_before_timeout;

    //ALL SET UP; WAITING FOR TRIGGER
    //wait for the Alexa trigger:
        g_get_coke_trigger = true;

        ROS_INFO("waiting for Alexg_found_object_codea code: rostopic pub Alexa_codes std_msgs/UInt32 100");
        while(ros::ok()) {
        	coke_grabber_goal.object_code = coke_grabber::coke_grabberGoal::MOVE_BACK;
        	coke_grabber_ac.sendGoal(coke_grabber_goal,&objectGrabberDoneCb);
        	bool finished_before_timeout = coke_grabber_ac.waitForResult();

        	if (!g_get_coke_trigger) {
			//ros::Duration(0.5).sleep();
        		usleep(500);
        		ros::spinOnce();    
        	} else {

			g_get_coke_trigger=false; // reset the trigger

			//  IF HERE, START THE FETCH BEHAVIOR!!

			ROS_INFO("sending navigation goal: TABLE");
			navigation_goal.location_code = beta_navigator::navigatorGoal::TABLE;
			navigator_ac.sendGoal(navigation_goal,&navigatorDoneCb);
			finished_before_timeout = navigator_ac.waitForResult(ros::Duration(30.0));
			if (!finished_before_timeout) {
				ROS_WARN("giving up waiting on result ");
				return 1;
			}
			if (g_navigator_rtn_code != beta_navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
				ROS_WARN("COULD NOT REACH TABLE; QUITTING");
				return 1;
			}
            ros::spinOnce();
            usleep(2000);
            ros::spinOnce();
			while (coke_confidence < 0.8) {
				usleep(500);
                ros::spinOnce();
			}
			if (coke_confidence > 0.8) {
				ROS_INFO("Best Similarity = %f ", coke_confidence);
				ROS_INFO("pose x is: %f", coke_pose.pose.position.x);
				ROS_INFO("pose y is: %f", coke_pose.pose.position.y);
				ROS_INFO("pose z is: %f", coke_pose.pose.position.z);
            //stuff a goal message:
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
	            ROS_INFO("transformed coke pose x is: %f", transed_pose.pose.position.x);
	            ROS_INFO("transformed coke pose y is: %f", transed_pose.pose.position.y);
	            ROS_INFO("transformed coke pose z is: %f", transed_pose.pose.position.z);
	            ROS_INFO("Grab the coke!");
	            coke_grabber_goal.object_code = coke_grabber::coke_grabberGoal::COKE_CAN; //specify the object to be grabbed
	            coke_grabber_goal.object_frame = transed_pose;
	            ROS_INFO("sending goal: ");
	            coke_grabber_ac.sendGoal(coke_grabber_goal, &objectGrabberDoneCb); // we could also name additional callback functions here, if desired
	            //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this

	            finished_before_timeout = coke_grabber_ac.waitForResult();
	            //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
	            if (g_coke_grabber_return_code!= coke_grabber::coke_grabberResult::OBJECT_ACQUIRED) {
	            	ROS_WARN("failed to grab object; giving up!");
	            	return 1;
	            }
        	}

			//if here, belief is that we are holding the Coke; return home
			ROS_INFO("sending navigation goal: HOME");
			navigation_goal.location_code = beta_navigator::navigatorGoal::HOME;
			navigator_ac.sendGoal(navigation_goal, &navigatorDoneCb);
			finished_before_timeout = navigator_ac.waitForResult();
			if (!finished_before_timeout) {
				ROS_WARN("Giving up on waiting for result.");
				return 1;
			}
			if (g_navigator_rtn_code != beta_navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
				ROS_WARN("COULD NOT GO HOME; QUITTING");
				return 1;
			}

            coke_grabber_goal.object_code = coke_grabber::coke_grabberGoal::GIVE_TO_HUMAN;
            coke_grabber_ac.sendGoal(coke_grabber_goal,&objectGrabberDoneCb);
            bool finished_before_timeout = coke_grabber_ac.waitForResult();

			ROS_INFO("Done fetching! Run me again?");
		}
	}
	return 0;
}

