cmake_minimum_required(VERSION 2.8.3)
project(beta_navigator)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(tf    REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(traj_planner src/traj_builder.cpp)
cs_add_library(motion_lib src/mobot_general.cpp)
cs_add_library(state_publisher src/pub_des_state.cpp)
cs_add_library(odom_tf_lib src/odom_tf_fncs.cpp)

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(beta_navigator          src/navigator.cpp)
cs_add_executable(navigator_action_client src/navigator_action_client.cpp)

cs_add_executable(nob_controllor     src/open_loop_controller.cpp)
cs_add_executable(lin_controller     src/lin_steering_wrt_odom.cpp)
# cs_add_executable(non_lin_controllor src/mobot_nl_steering.cpp)
cs_add_executable(adding_path        src/append_path_client.cpp)
cs_add_executable(tian_lidar         src/lidar_detector.cpp)
cs_add_executable(odom_tf_set        src/odom_tf.cpp)
cs_add_executable(james_lidar        src/lidar_alarm.cpp)

#the following is required, if desire to link a node in this package with a
#library created in this same package
# edit the arguments to reference the named node and named library within this package
target_link_libraries(beta_navigator traj_planner motion_lib state_publisher ${catkin_LIBRARIES})
target_link_libraries(odom_tf_lib ${catkin_LIBRARIES})
target_link_libraries(odom_tf_set odom_tf_lib)

cs_install()
cs_export()
    
