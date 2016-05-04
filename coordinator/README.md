# beta_coke_fetcher
-------------------------
This is a set of nodes that includes: *coordinator*, *simple_coke_finder*, *navigator*, and *coke_grabber*. For each node, a description and example usage is written below.

## Setup
To prepare the robot for use of the beta_coke_fetcher package, complete the following:

IMPORTANT: On Jinx, for each terminal, do `baxter_master`.

### On booting Jinx
+ Give permission `a+rw` and `+x` to `/dev/ttyUSB0` and to `/dev/ttyUSB1`
+ Register object to database for recognition: `rosrun object_recognition_core object_add.py -n "coke " -d "A empty coke can" --commit`
+ Bind mesh with object, notice to replace [] with object ID: ``rosrun object_recognition_core mesh_add.py [the object id that previous command returned] `rospack find coke_grabber`/data/coke.stl --commit``

### On booting Merry
+ Wait for her head halo to turn green.
+ Enable Baxter: `rosrun baxter_tools enable_robot.py -e # subsumed by launch file`
+ Start up cwru_base with kinect: `roslaunch coke_grabber base_kinect.launch`
+ Start up CWRU nodes with: `roslaunch baxter_launch_files baxter_nodes.launch # subsumed by launch file`
+ Open rqt_reconfigure: `rosrun rqt_reconfigure rqt_reconfigure`
    + In `camera/driver`, select `depth_registration`.


# Coordinator

Top-level node that is a client of: *navigator*, *coke_grabber*, and *simple_coke_finder*.
Waits for an Alexa trigger, then starts entire process of: navigation to table,
recognition of object, grasp of object, and return to home.

## Example usage

Run these in separate terminals.  Wait until main has finished loading before runnning `coke_fetcher`.


```
roslaunch coke_fetcher main.launch # does not move the robot's wheels
roslaunch coke_fetcher coke_fetcher # Go!
```

### Obsolete old ignore
Then trigger the behavior with:
`rostopic pub Alexa_codes std_msgs/UInt32 100`

# Navigator

The navigator is written as an action client/server and is intended to move the mobile robot to a specific location. 

There are 3 cases for the navigator:
1. navigate to home (robot wake location)
2. navigation to the table (table with coke can location)
3. navigate to an input position

## Example usage

The *navigator.cpp* node navigates the mobile robot to a proposed pose. The action server is named *object_grabber_server* and the source code is *navigator.cpp* and there is a class called *Navigator* that is built for talk to the navigator to move to a proposed pose.

If you want to go home, use code HOME for the action command:
```
navigation_goal.location_code=
beta_navigator::navigatorGoal::HOME //ask jinx to walk back to where she woke up
navigator_ac.sendGoal(navigation_goal,&navigatorDoneCb);
bool finished_before_timeout = navigator_ac.waitForResult(ros::Duration(30.0));
```
If you want to move to the table, use code TABLE for the action command:
```
navigation_goal.location_code=
beta_navigator::navigatorGoal::TABLE_1 //ask jinx to walk towards table 1
navigator_ac.sendGoal(navigation_goal,&navigatorDoneCb);
bool finished_before_timeout = navigator_ac.waitForResult(ros::Duration(30.0));
```
If you want to move to the proposed pose, use code COORDS for the command:
```
navigation_goal.location_code=
beta_navigator::navigatorGoal::COORDS //ask jinx to walk towards a certain pose.
navigator_ac.sendGoal(navigation_goal,&navigatorDoneCb);
bool finished_before_timeout = navigator_ac.waitForResult(ros::Duration(30.0));
```

## Running tests/demos

```
$ rosrun beta_navigator beta_navigator
```

Make calls using `beta_navigator/navigator.action` action messages.

This should all be coordinated by the `coordinator` package.

# coke_grabber

This package contains two libraries for arm control: a library for gripper control and a library for coke recognition.

The coke_grabber package successfully recognizes the coke can and grabs it. This package also includes a Moveit! version of the grabber which has some bugs due to the Baxter's firmware update. 

The package is based on Moveit! and Object Recognition Kitchen (ORK) (*http://wg-perception.github.io/object_recognition_core/*).

Install Moveit!: 
`sudo apt-get install ros-indigo-moveit-full`

Install Object Recognition Kitchen: 
`sudo apt-get install ros-indigo-ecto* ros-indigo-opencv-candidate ros-indigo-object-recognition-*`

## Example usage

There are two versions of arm control. One is action client and server modified from Newman's example and the other is a Moveit! interface.After completing the setup steps, complete the following steps to run either version of the arm controller without the coordinator.

### General

For running action server arm controller:
`roslaunch coke_grabber coke_grabber.launch`

For running Moveit! arm controllor:
`roslaunch coke_grabber moveit_grabber.launch`

### Action server and client

The action server is named *object_grabber_server* and the source code is *object_grabber_as.cpp*. The *ArmMotionCommander* class is built to communicate with the Cartesian move action server.

The example coke grab client is *ork_grabber_client* and the source code is *example_object_grabber_action_client.cpp*. Simple usages are included here.

If you want to grab the coke, use code COKE_CAN for the action command:
```
coke_grabber_goal.object_code = coke_grabber::coke_grabberGoal::COKE_CAN;
coke_grabber_goal.object_frame = coke_pose;	//put the pose of coke can here
coke_grabber_ac.sendGoal(coke_grabber_goal, &objectGrabberDoneCb);
coke_grabber_ac.waitForResult();
```
If you want to move the arm back to the initial pose, use code MOVE_BACK:
```
coke_grabber_goal.object_code = coke_grabber::coke_grabberGoal::MOVE_BACK;
coke_grabber_ac.sendGoal(coke_grabber_goal,&objectGrabberDoneCb);
coke_grabber_ac.waitForResult();
```
If you want to place the coke, use code DROP_COKE:
```
coke_grabber_goal.object_code = coke_grabber::coke_grabberGoal::DROP_COKE;
coke_grabber_goal.object_frame = place_pose;	//put the pose to place the coke here
coke_grabber_ac.sendGoal(coke_grabber_goal, &objectGrabberDoneCb);
coke_grabber_ac.waitForResult();
```
If you want to give the coke to someone, use code GIVE_TO_HUMAN:
```
coke_grabber_goal.object_code = coke_grabber::coke_grabberGoal::GIVE_TO_HUMAN;
coke_grabber_goal.object_frame = human_pose;	//put the pose the that towards human here
coke_grabber_ac.sendGoal(coke_grabber_goal, &objectGrabberDoneCb);
coke_grabber_ac.waitForResult();
```
If you want to move the arm to a certain pose, use code RIGHT_TO_POSE:
```
coke_grabber_goal.object_code = coke_grabber::coke_grabberGoal::RIGHT_TO_POSE;
coke_grabber_goal.object_frame = pose you want;	//put the pose want to move here
coke_grabber_ac.sendGoal(coke_grabber_goal, &objectGrabberDoneCb);
coke_grabber_ac.waitForResult();
```

### Moveit! library
There is a class in *BaxterArmCommander.cpp* called *BaxterArmCommander* which contains methods for control of both arms. Since all the functions named are quite straight forward, the function names and a short comment have been provided to show how to use it.

The test program is called *moveit_grabber*, the source code is *baxter_arm_test.cpp*. If you want to use this library, in you CMakeList, link your executable with `baxter_arm` and `baxter_gripper`
```
bool rightArmBack();                        //move arm back to a pre pose and open gripper, return successful or not
Vector7d rightGetJoints();                  //return the angle (in rad) of each joint
geometry_msgs::Pose rightGetPose();         //return pose of right arm end effector
bool rightPlan(geometry_msgs::Pose pose);   //plan a trajectory to point end effector to a certain pose, return success or failed
bool rightPlan(Vector7d joints);            //another version that accept joint angles
//another version that accept this three parameter of the object, this three values can be calculated from Newman's code
bool rightPlan(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid);
bool rightPlanOffset(Vector3d offset);      //from current pose, add a offset
void rightExecute();                        //execute planned pose
bool rightMove(geometry_msgs::Pose pose);   //combine plan and execute together
bool rightMove(Vector7d joints);            //same, specify joints
bool rightMove(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid);  //same, specify the three parameters
bool rightMoveOffset(Vector3d offset);
void rightGrab();                           //close gripper
void rightRelease();                        //open gripper
void rightShowPath();                       //visualize trajectory that planned in rviz, must call plan first

bool leftArmBack();                         //everything is just the same as right one

bool ArmBack();                             //move both back to pre pose
bool grabCoke(geometry_msgs::Pose coke_pose);//a whole sequence of movement for grabbing coke
    
std::vector<double> quat2euler(geometry_msgs::Quaternion quaternion);   //this may be useful
```
### ORK Coke Finder
there is a class called *SimpleCokeFinder* written for simplify the coke recognition which is subscribe to the topic published by detection mode of ORK and transform to torso frame. Only coke can finding part is complete, which is the useful one, the usage can be found in *ork_grabber_client* which is *coke_grabber_action_client.cpp*

If you want to use this library, in you CMakeList, link your executable with `simple_coke_finder`

### Gripper class
The  class written for gripper control is called *BaxterGripper*, it have several mode for gripper control including Position mode, Torque mode and Open_close mode, however, `OPEN_CLOSE` mode is recommended since it is Newman tuned for this project. It can be changed by method `set_mode()`, other functions are too straight forward that there is no necessity for writing comment for it
```
void close();
void open();
void test();
void set_mode(const int mode);
```
