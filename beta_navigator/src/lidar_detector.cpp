    /* wsn example program to illustrate LIDAR processing.  1/23/15 */
/* modified by shipei tian*/

#include <ros/ros.h>                    /* Must include this for all ROS cpp projects */
#include <sensor_msgs/LaserScan.h>
#include <beta_navigator/lidar.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>              /* boolean message */

double MIN_SAFE_DISTANCE = 1.0;   /* set alarm if anything is within 0.5m of the front of robot */
double DETECT_DISTANCE = 2.5;
double DISTANCE_FILTER = 0.15;
double FRONT_ANGLE = 0.5;
double LEFT_ANGLE = 0.5;
double RIGHT_ANGLE = 0.5;

/* these values to be set within the laser callback */
int	    ping_index_		    = -1;   /* NOT real; callback will have to find this */
double	angle_min_		    = 0.0;
double	angle_max_		    = 0.0;
double	angle_increment_    = 0.0;
double	range_min_		    = 0.0;
double	range_max_		    = 0.0;
double opt_search_loop      = 10.0;

int front_ping_start;
int front_ping_end;
int left_ping_start;
int left_ping_end;
int right_ping_start;
int right_ping_end;

std::vector<double> distance_tab;

ros::Publisher	lidar_alarm_publisher_;
ros::Publisher	alarm_info_publisher_;
ros::Publisher	opt_dir_publisher_;
/*
 * really, do NOT want to depend on a single ping.  Should consider a subset of pings
 * to improve reliability and avoid false alarms or failure to see an obstacle
 */

 void laserCallback( const sensor_msgs::LaserScan & laser_scan )
 {
    bool avail = false;
    bool world_empty = true;
    int min_dis = -1;
    int min_dis_l = -1;
    int min_dis_r = -1;
    int min_dis_f = -1;
    double opt_dir = 0;

    int opt_pin = -1;
    int opt_pin_num = -1;

     int debug = 0;

    beta_navigator::lidar alarm_info_msg;
    std_msgs::Bool lidar_alarm_msg;
    std_msgs::Float64 opt_dir_msg;

    if ( ping_index_ < 0 ) {
        /* for first message received, set up the desired index of LIDAR range to eval */
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max - 1.0;
        //range_max_		= 30.0;
        /*
         * what is the index of the ping that is straight ahead?
         * BETTER would be to use transforms, which would reference how the LIDAR is mounted;
         * but this will do for simple illustration
         */
         ping_index_ = (int) (((angle_max_ - angle_min_) / 2) / angle_increment_);
         ROS_INFO("LIDAR setup: ping_index = %d", ping_index_);

         front_ping_start = ping_index_ - (int) ((FRONT_ANGLE / angle_increment_) / 2);
         front_ping_end = ping_index_ + (int) ((FRONT_ANGLE / angle_increment_) / 2);
         left_ping_start = ping_index_ - (M_PI / 2) / angle_increment_ - (int) ((LEFT_ANGLE / angle_increment_) / 2);
         left_ping_end = ping_index_ - (M_PI / 2) / angle_increment_ + (int) ((LEFT_ANGLE / angle_increment_) / 2);
         right_ping_start = ping_index_ + (M_PI / 2) / angle_increment_ - (int) ((RIGHT_ANGLE / angle_increment_) / 2);
         right_ping_end = ping_index_ + (M_PI / 2) / angle_increment_ + (int) ((RIGHT_ANGLE / angle_increment_) / 2);

         if (left_ping_start < 0) {
            left_ping_start = 0;
            ROS_INFO("left alarm range starts below minimum angle");
        }
        if (right_ping_end > ping_index_ * 2) {
            right_ping_end = ping_index_ * 2;
            ROS_INFO("right alarm range ends above maximum angle");
        }

        distance_tab.clear();
        double value;
        for (int i = front_ping_start; i < front_ping_end; ++i) {
            value = DETECT_DISTANCE * (1 / cos(fabs(i - ping_index_) * angle_increment_));
            //ROS_INFO("TAB%d: %f",i, value);
            distance_tab.push_back(value);
        }
    }

    double opt_search_range = range_max_;

    for (int j = 0; j < ping_index_*2; ++j) {
        //ROS_INFO("range = %f, max = %f, %s", laser_scan.ranges[j], range_max_, laser_scan.ranges[j] <= range_max_? "<":">");
        if (laser_scan.ranges[j] <= range_max_ && laser_scan.ranges[j] >= DISTANCE_FILTER) {
            world_empty = false;
        }
        if (laser_scan.ranges[j] <= MIN_SAFE_DISTANCE && laser_scan.ranges[j] >= DISTANCE_FILTER) {
            if (min_dis < 0) {
                min_dis = j;
            } else {
                if (laser_scan.ranges[j] < laser_scan.ranges[min_dis]) {
                    min_dis = j;
                }
            }
        }
        if (j >= left_ping_start && j < left_ping_end) {
            if (laser_scan.ranges[j] <= distance_tab[j - left_ping_start] && laser_scan.ranges[j] >= DISTANCE_FILTER) {
                if (min_dis_l < 0) {
                    min_dis_l = j;
                } else {
                    if (laser_scan.ranges[j] < laser_scan.ranges[min_dis_l]) {
                        min_dis_l = j;
                    }
                }
            }
        }
        if (j >= front_ping_start && j < front_ping_end) {
            if (laser_scan.ranges[j] <= distance_tab[j - front_ping_start] && laser_scan.ranges[j] >= DISTANCE_FILTER) {
                if (min_dis_f < 0) {
                    min_dis_f = j;
                } else {
                    if (laser_scan.ranges[j] < laser_scan.ranges[min_dis_f]) {
                        min_dis_f = j;
                    }
                }
            }
        }
        if (j >= right_ping_start && j < right_ping_end) {
            if (laser_scan.ranges[j] <= distance_tab[j - right_ping_start] && laser_scan.ranges[j] >= DISTANCE_FILTER) {
                if (min_dis_r < 0) {
                    min_dis_r = j;
                } else {
                    if (laser_scan.ranges[j] < laser_scan.ranges[min_dis_r]) {
                        min_dis_r = j;
                    }
                }
            }
        }
        if (laser_scan.ranges[j] >= opt_search_range) {
            int current_opt = 0;
            avail = true;
            for (int k = j; k < 2 * ping_index_; ++k) {
                if (laser_scan.ranges[k] >= opt_search_range) {
                    current_opt++;
                }
                else {
                    if (current_opt > opt_pin_num) {
                        opt_pin = j + (current_opt)/2;
                        opt_pin_num = current_opt;
                    }
                    break;
                }
            }
        }
    }
    while (!avail) {
        opt_search_range -= range_max_*(1/opt_search_loop);
        for (int i = 0; i < ping_index_*2; ++i) {
            if (laser_scan.ranges[i] >= opt_search_range) {
                int current_opt = 0;
                avail = true;
                for (int k = i; k < 2 * ping_index_; ++k) {
                    if (laser_scan.ranges[k] >= opt_search_range) {
                        current_opt++;
                    }
                    else {
                        if (current_opt > opt_pin_num) {
                            opt_pin = i + (current_opt)/2;
                            opt_pin_num = current_opt;
                        }
                        break;
                    }
                }
            }
        }
    }
    if (world_empty) {
        lidar_alarm_msg.data = false;
        lidar_alarm_publisher_.publish( lidar_alarm_msg );
        ROS_INFO("You have enter an empty world");
        alarm_info_msg.world_empty = true;
        alarm_info_msg.g_alarm = false;
        alarm_info_msg.f_alarm = false;
        alarm_info_msg.l_alarm = false;
        alarm_info_msg.r_alarm = false;
        alarm_info_msg.g_distance = range_max_;
        alarm_info_msg.f_distance = range_max_;
        alarm_info_msg.l_distance = range_max_;
        alarm_info_msg.r_distance = range_max_;
        alarm_info_msg.alarm_dir = 0.0;
        alarm_info_msg.wide_dir = 0.0;
        alarm_info_publisher_.publish( alarm_info_msg );
        opt_dir_msg.data = 0.0;
        opt_dir_publisher_.publish( opt_dir_msg );
        return;
    } else {
        alarm_info_msg.world_empty = false;
        opt_dir = opt_pin * angle_increment_ + angle_min_;
        //ROS_INFO("LIDAR: %s, best direction: %f",min_dis_f >= 0?"Alarmed":"Clear", (opt_dir/M_PI)*180);
        ROS_INFO("Best direction: %f", (opt_dir/M_PI)*180);
        ROS_INFO("Left alarm %s", min_dis_l >= 0?"Alarmed":"Clear");
        ROS_INFO("Right alarm %s", min_dis_r >= 0?"Alarmed":"Clear");
        ROS_INFO("Front alarm %s", min_dis_f >= 0?"Alarmed":"Clear");
        alarm_info_msg.wide_dir = opt_dir;

        if (min_dis_f >= 0)
        {
            alarm_info_msg.f_alarm = true;
            alarm_info_msg.f_distance = laser_scan.ranges[min_dis_f];
            
        } else {
            alarm_info_msg.f_alarm = false;
            alarm_info_msg.f_distance = range_max_;
        }
        if (min_dis_l >= 0)
        {
            alarm_info_msg.l_alarm = true;
            alarm_info_msg.l_distance = laser_scan.ranges[min_dis_l];
            
        } else {
            alarm_info_msg.l_alarm = false;
            alarm_info_msg.l_distance = range_max_;
        }
        if (min_dis_r >= 0)
        {
            alarm_info_msg.r_alarm = true;
            alarm_info_msg.r_distance = laser_scan.ranges[min_dis_r];
            
        } else {
            alarm_info_msg.r_alarm = false;
            alarm_info_msg.r_distance = range_max_;
        }
        if (min_dis >= 0)
        {
            ROS_INFO( "TOO CLOSE TO WALL!! min distance = %f", laser_scan.ranges[min_dis] );
            alarm_info_msg.g_alarm = true;
            alarm_info_msg.g_distance = laser_scan.ranges[min_dis_r];
            alarm_info_msg.alarm_dir = min_dis * angle_increment_ + angle_min_;
            
        } else {
            alarm_info_msg.g_alarm = false;
            alarm_info_msg.g_distance = range_max_;
            alarm_info_msg.alarm_dir = 0.0;
        }
        lidar_alarm_msg.data = alarm_info_msg.g_alarm;
        lidar_alarm_publisher_.publish( lidar_alarm_msg );
        alarm_info_publisher_.publish( alarm_info_msg );
        opt_dir_msg.data = opt_dir;
        opt_dir_publisher_.publish( opt_dir_msg );
    }
}


int main( int argc, char **argv )
{
    ros::init( argc, argv, "lidar_braker" );      /* name this node */
    ros::NodeHandle nh;
    if (!nh.hasParam("MIN_SAFE_DISTANCE")){
        ROS_INFO("No MIN_SAFE_DISTANCE specified, using default value %f", MIN_SAFE_DISTANCE);
    } else {
        nh.getParam("MIN_SAFE_DISTANCE", MIN_SAFE_DISTANCE);
        ROS_INFO("MIN_SAFE_DISTANCE set to %f", MIN_SAFE_DISTANCE);
    }
    if (!nh.hasParam("DETECT_DISTANCE")){
        ROS_INFO("No DETECT_DISTANCE specified, using default value %f", DETECT_DISTANCE);
    } else {
        nh.getParam("DETECT_DISTANCE", DETECT_DISTANCE);
        ROS_INFO("DETECT_DISTANCE set to %f", DETECT_DISTANCE);
    }
    if (!nh.hasParam("FRONT_ANGLE")){
        ROS_INFO("No FRONT_ANGLE specified, using default value %f", FRONT_ANGLE);
    } else {
        nh.getParam("FRONT_ANGLE", FRONT_ANGLE);
        ROS_INFO("FRONT_ANGLE set to  %f", FRONT_ANGLE);
    }
    if (!nh.hasParam("LEFT_ANGLE")){
        ROS_INFO("No LEFT_ANGLE specified, using default value %f", LEFT_ANGLE);
    } else {
        nh.getParam("LEFT_ANGLE", LEFT_ANGLE);
        ROS_INFO("LEFT_ANGLE set to %f", LEFT_ANGLE);
    }
    if (!nh.hasParam("RIGHT_ANGLE")){
        ROS_INFO("No RIGHT_ANGLE specified, using default value %f", RIGHT_ANGLE);
    } else {
        nh.getParam("RIGHT_ANGLE", RIGHT_ANGLE);
        ROS_INFO("RIGHT_ANGLE set to %f", RIGHT_ANGLE);
    }
    /* create a Subscriber object and have it subscribe to the lidar topic */
    ros::Publisher pub = nh.advertise<std_msgs::Bool>( "lidar_alarm", 1 );
    lidar_alarm_publisher_ = pub;                   /* let's make this global, so callback can use it */
    ros::Publisher pub2 = nh.advertise<beta_navigator::lidar>( "alarm_info", 1 );
    alarm_info_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe( "scan", 1, laserCallback );
    ros::Publisher pub3 = nh.advertise<std_msgs::Float64>( "opt_direction", 1 );
    opt_dir_publisher_ = pub3;
    ros::spin();                                    /* this is essentially a "while(1)" statement, except it */
    /*
     * forces refreshing wakeups upon new data arrival
     * main program essentially hangs here, but it must stay alive to keep the callback function alive
     */
    return(0);                                      /* should never get here, unless roscore dies */
 }


