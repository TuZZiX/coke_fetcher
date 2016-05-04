#ifndef PUB_DES_STATE_H_
#define PUB_DES_STATE_H_

#include <queue>
#include <traj_builder/traj_builder.h> //has almost all the headers we need
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <beta_navigator/path.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

const int E_STOPPED = 0; //define some mode keywords
const int DONE_W_SUBGOAL = 1;
const int PURSUING_SUBGOAL = 2;
const int HALTING = 3;
const int RECOVERING = 4;
const double default_max_odom_draft = 2.0;
const double default_max_pos_draft = 2.0;

class DesStatePublisher {
private:

    ros::NodeHandle nh_; // we'll need a node handle; get one upon instantiation

    //some class member variables:
    nav_msgs::Path path_;
    std::vector<nav_msgs::Odometry> des_state_vec_;
    nav_msgs::Odometry des_state_;
    nav_msgs::Odometry halt_state_;
    nav_msgs::Odometry seg_end_state_;
    nav_msgs::Odometry seg_start_state_;
    nav_msgs::Odometry current_des_state_;
    geometry_msgs::Twist halt_twist_;
    geometry_msgs::PoseStamped start_pose_;
    geometry_msgs::PoseStamped end_pose_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::Twist current_twist_;
    nav_msgs::Odometry current_odom_;
    std_msgs::Float64 float_msg_;
    ros::Time last_cb_time_;

    double des_psi_;
    std::queue<geometry_msgs::PoseStamped> path_queue_; //a C++ "queue" object, stores vertices as Pose points in a FIFO queue
    int motion_mode_;
    bool e_stop_trigger_; //these are intended to enable e-stop via a service
    bool e_stop_reset_;
    int traj_pt_i_;
    int npts_traj_;
    double dt_;
    //dynamic parameters: should be tuned for target system
    double accel_max_; 
    double alpha_max_; 
    double speed_max_; 
    double omega_max_; 
    double path_move_tol_;

    bool lidar_alarm_;
    bool is_alarmed_;
    bool odomCB_;
    bool enable_replanning_;

    double time_interval_;
    double distance_interval_;
    double pose_draft_;

    double max_odom_draft;
    double max_pos_draft;

    // some objects to support service and publisher
    ros::ServiceServer estop_service_;
    ros::ServiceServer estop_clear_service_;
    ros::ServiceServer flush_path_queue_;
    ros::ServiceServer append_path_;
    
    ros::Publisher desired_state_publisher_;
    ros::Publisher des_psi_publisher_;

    ros::Subscriber alarm_subscriber_;
    ros::Subscriber odom_subscriber_;

    //a trajectory-builder object; 
    TrajBuilder trajBuilder_;
    ros::Rate looprate;

    // member methods:
    void initializePublishers();
    void initializeServices();
    bool estopServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
    bool clearEstopServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
    bool flushPathQueueCB(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
    bool appendPathQueueCB(beta_navigator::pathRequest& request,beta_navigator::pathResponse& response);
    void alarmCB(const std_msgs::Bool& alarm_msg);
    void odomCallback(const nav_msgs::Odometry& odom_msg);

public:
    DesStatePublisher(ros::NodeHandle& nh);//constructor
    int get_motion_mode() {return motion_mode_;}
    void set_motion_mode(int mode) {motion_mode_ = mode;}
    bool get_estop_trigger() { return e_stop_trigger_;}
    void set_estop_trigger() { e_stop_trigger_ = true;}
    void reset_estop_trigger() { e_stop_trigger_ = false;}
    void set_init_pose(double x,double y, double psi);
    void pub_next_state();
    nav_msgs::Odometry get_odom();
    void append_path_queue(geometry_msgs::PoseStamped pose) {
        path_queue_.push(pose); }
    void append_path_queue(double x, double y, double psi) {
        path_queue_.push(trajBuilder_.xyPsi2PoseStamped(x,y,psi)); }
    void loop_sleep() {
        looprate.sleep(); }
    void sync_pose() {
        current_pose_.pose = current_odom_.pose.pose; }
    void enable_replanning() {
        enable_replanning_ = true; }
    void disable_replanning() {
        enable_replanning_ = false; }
    void flush_path() {
        while (!path_queue_.empty()) {
            path_queue_.pop();
        }
    }
};
#endif