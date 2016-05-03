//
// Created by sxt437 on 5/1/16.
//

#include <SimpleCokeFinder/SimpleCokeFinder.h>
SimpleCokeFinder::SimpleCokeFinder(ros::NodeHandle &nodehandle): nh_(nodehandle) {
    coke_confidence = 0;
    firstCB = false;
    object_sub = nh_.subscribe("/recognized_object_array", 1, &SimpleCokeFinder::objectCallback, this);
    plane_sub = nh_.subscribe("/table_array", 1, &SimpleCokeFinder::planeCallback, this);
    coke_pose.header.frame_id = "camera_depth_optical_frame";
    coke_id = "";
}

void SimpleCokeFinder::objectCallback(const object_recognition_msgs::RecognizedObjectArray objects_msg) {
    double confident = 0;
    int id = -1;
    //ROS_WARN("Total %d objects found", (int)objects_msg.objects.size());

    if (!firstCB && (int)objects_msg.objects.size() == 1) {
        coke_id.assign(objects_msg.objects[0].type.key.c_str());
        coke_pose.header.frame_id = objects_msg.header.frame_id.c_str();
        firstCB = true;
    }
    for (int i = 0; i < objects_msg.objects.size(); ++i) {
        //ROS_INFO("Frame_id 2: %s", objects_msg.objects[i].header.frame_id.c_str());
        //ROS_INFO("Frame_id 3: %s", objects_msg.objects[i].pose.header.frame_id.c_str());
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
    //ROS_INFO("Best Similarity = %f ", objects_msg.objects[id].confidence);
    //ROS_INFO("pose x is: %f", objects_msg.objects[id].pose.pose.pose.position.x);
    //ROS_INFO("pose y is: %f", objects_msg.objects[id].pose.pose.pose.position.y);
    //ROS_INFO("pose z is: %f", objects_msg.objects[id].pose.pose.pose.position.z);
    //ROS_INFO("---------------------------------");
}

void SimpleCokeFinder::planeCallback(const object_recognition_msgs::TableArray plane_msg) {
    //ROS_WARN("Total %d planes found", (int)plane_msg.tables.size());
    /*
    for (int i = 0; i < plane_msg.tables.size(); ++i) {
        ROS_INFO("Plane %d: plane pose x is: %f", i+1, plane_msg.tables[i].pose.position.x);
        ROS_INFO("Plane %d: plane pose y is: %f", i+1, plane_msg.tables[i].pose.position.y);
        ROS_INFO("Plane %d: plane pose z is: %f", i+1, plane_msg.tables[i].pose.position.z);
        ROS_INFO("---------------------------------");
    }*/

}

bool SimpleCokeFinder::getCokePoseKinect(geometry_msgs::PoseStamped &coke_pose, double &confidence) {
    ros::spinOnce();
    if (coke_confidence > 0.8) {
        coke_pose = this->coke_pose;
        confidence = this->coke_confidence;
        return true;
    }
    return false;
}

bool SimpleCokeFinder::getCokePoseTorso(geometry_msgs::PoseStamped &coke_pose, double &confidence) {
    if (getCokePoseKinect(coke_pose, confidence)) {
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
        coke_pose = transed_pose;
    }
    return false;
}

bool SimpleCokeFinder::getTableKinect(geometry_msgs::PoseStamped &table_pose, Eigen::Vector3d &start_cord,
                                      Eigen::Vector3d &end_cord) {
    return false;
}

bool SimpleCokeFinder::getTableTorso(geometry_msgs::PoseStamped &table_pose, Eigen::Vector3d &start_cord,
                                     Eigen::Vector3d &end_cord) {
    return false;
}