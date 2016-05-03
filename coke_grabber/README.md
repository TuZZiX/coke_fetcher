# coke_grabber
-------------------------
This package contains two libraries for arm control, a library for gripper control and a library for coke recognition.

It can successfully recognize the coke can and grab it, in that Moveit! version have some bug due to the upgrade of Baxter's frame-ware.

The package is based on Moveit! and Object Recognition Kitchen (*http://wg-perception.github.io/object_recognition_core/*).

Install moveit!: 
`sudo apt-get install ros-indigo-moveit-full`

Install Object Recognition Kitchen: 
`sudo apt-get install ros-indigo-ecto* ros-indigo-opencv-candidate ros-indigo-object-recognition-*`

## Code testing
Give permission **a+rw** and **+x** to **/dev/ttyUSB**

On Jinx, for each terminal, do `baxter_master`

Enable Baxter:
`rosrun baxter_tools enable_robot.py -e`

Start up cwru_base with kinect:
`roslaunch coke_garbber base_kinect.launch`

Open rqt_reconfigure: 
`rosrun rqt_reconfigure rqt_reconfigure`

In **camera** / **driver**, select **depth_registration**.

Register object to database for recognition:
`rosrun object_recognition_core object_add.py -n "coke " -d "A empty coke can" --commit`

Bind mesh with object, notice to replace [] with object ID:
``rosrun object_recognition_core mesh_add.py [the object id that previous command returned] `rospack find coke_garbber`/data/coke.stl --commit``

For running action server arm controller:
`roslaunch coke_grabber coke_grabber.launch`

For running Moveit! arm controllor:
`roslaunch coke_grabber moveit_grabber.launch`

## Code explain

The two versions of arm control is action client and server modified from Newman and Moveit! interface written by my own.


###Action server and client
The action server is named *object_grabber_server* and the source code is *object_grabber_as.cpp* and there is a class called *ArmMotionCommander* that is build for talk to Cartesian move action server.

The example coke grab client is*ork_grabber_client* which source code is *example_object_grabber_action_client.cpp*, simple usage are included here.

If you want to grab the coke, use  code COKE_CAN for the action command:
```
coke_grabber_goal.object_code = coke_grabber::coke_grabberGoal::COKE_CAN;
coke_grabber_goal.object_frame = coke_pose;	//put the pose of coke can here
coke_grabber_ac.sendGoal(coke_grabber_goal, &objectGrabberDoneCb);
coke_grabber_ac.waitForResult();
```
If you want to move arm back to prepose:
```
coke_grabber_goal.object_code = coke_grabber::coke_grabberGoal::MOVE_BACK;
coke_grabber_ac.sendGoal(coke_grabber_goal,&objectGrabberDoneCb);
coke_grabber_ac.waitForResult();
```
If you want to place coke to a place:
```
coke_grabber_goal.object_code = coke_grabber::coke_grabberGoal::DROP_COKE;
coke_grabber_goal.object_frame = place_pose;	//put the pose to place the coke here
coke_grabber_ac.sendGoal(coke_grabber_goal, &objectGrabberDoneCb);
coke_grabber_ac.waitForResult();
```
If you want to give coke to a people:
```
coke_grabber_goal.object_code = coke_grabber::coke_grabberGoal::GIVE_TO_HUMAN;
coke_grabber_goal.object_frame = human_pose;	//put the pose the that towards human here
coke_grabber_ac.sendGoal(coke_grabber_goal, &objectGrabberDoneCb);
coke_grabber_ac.waitForResult();
```
If you want to move arm to a certain pose:
```
coke_grabber_goal.object_code = coke_grabber::coke_grabberGoal::RIGHT_TO_POSE;
coke_grabber_goal.object_frame = pose you want;	//put the pose want to move here
coke_grabber_ac.sendGoal(coke_grabber_goal, &objectGrabberDoneCb);
coke_grabber_ac.waitForResult();
```
###Moveit! library
There is an class in *BaxterArmCommander.cpp* called *BaxterArmCommander*, it contains methods for control both arm, only useful method are shown there. Since all the function named quite straight forward, looking at function name would be a good way to show how to use it 

The test program is called *moveit_grabber*, the source code is *baxter_arm_test.cpp*. If you want to use this library, in you CMakeList, link your executable with **baxter_arm** and **baxter_gripper**
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
###ORK Coke Finder
there is a class called *SimpleCokeFinder* written for simplify the coke recognition which is subscribe to the topic published by detection mode of ORK and transform to torso frame. Only coke can finding part is complete, which is the useful one, the usage can be found in *ork_grabber_client* which is *coke_grabber_action_client.cpp*

If you want to use this library, in you CMakeList, link your executable with **simple_coke_finder**
### Gripper class
The  class written for gripper control is called *BaxterGripper*, it have several mode for gripper control including Position mode, Torque mode and Open_close mode, however, `OPEN_CLOSE` mode is recommended since it is Newman tuned for this project. It can be changed by method `set_mode()`, other functions are too straight forward that there is no necessity for writing comment for it
```
void close();
void open();
void test();
void set_mode(const int mode);
```


