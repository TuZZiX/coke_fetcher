cmake_minimum_required(VERSION 2.8.3)
project(coke_grabber)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(arm_commander src/ArmMotionCommander.cpp)
cs_add_library(baxter_gripper src/BaxterGripper.cpp)
cs_add_library(baxter_arm src/BaxterArmCommander.cpp)
cs_add_library(simple_coke_finder src/SimpleCokeFinder.cpp)

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(object_grabber_server src/object_grabber_as.cpp)
cs_add_executable(object_grabber_client src/example_object_grabber_action_client.cpp)
cs_add_executable(ork_grabber_client src/coke_grabber_action_client.cpp)
cs_add_executable(moveit_grabber src/baxter_arm_test.cpp)
#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
target_link_libraries(moveit_grabber baxter_arm baxter_gripper ${catkin_LIBRARIES})
target_link_libraries(object_grabber_server arm_commander ${catkin_LIBRARIES})
target_link_libraries(ork_grabber_client simple_coke_finder arm_commander ${catkin_LIBRARIES})

cs_install()
cs_export()
    
