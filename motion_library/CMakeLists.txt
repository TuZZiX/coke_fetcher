cmake_minimum_required(VERSION 2.8.3)
project(motion_library)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next line to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)

#uncomment the following 4 lines to use the Eigen library
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
#cs_add_library(jinx_traj_builder_ps8 src/traj_builder.cpp)
#cs_add_library(mobot_control_ps8 src/mobot_general.cpp)

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
#cs_add_executable(traj_publisher_ps8 src/pub_des_state_main.cpp src/pub_des_state.cpp)
#cs_add_executable(simple_controllor_ps8 src/open_loop_controller.cpp)
#cs_add_executable(steering_corrector_ps8 src/lin_steering_wrt_odom.cpp)
#cs_add_executable(steering_corrector_nl_ps8 src/mobot_nl_steering.cpp)
#cs_add_executable(append_and_stop_ps8 src/append_path_client.cpp)
#cs_add_executable(wall_detector_ps8 src/lidar_detector.cpp)
#cs_add_executable(amcl_odom_tf_ps8 src/odom_tf.cpp  src/odom_tf_fncs.cpp)
#cs_add_executable(wall_detector2_ps8 src/lidar_alarm.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
#target_link_libraries(traj_publisher_ps8  jinx_traj_builder_ps8 mobot_control_ps8)

cs_install()
cs_export()
    
