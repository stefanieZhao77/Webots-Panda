# Webots-ROS (Franka Panda)

> This is a docker image built for run ROS, MoveIt and Webots. In this project, you can see the demos about how to integrate MoveIt with Webots, and how to use ROS to control robots in Webots directly.

## How to use

### Installation
*  docker build --tag \<tag\> .
*  docker run -it -p 6080:80 --rm \<tag\>
*  visit localhost:6080 in your browser

### How to run demos

* open lxterminal
* run Webots by "webots" in command line && open /root/webots/panda/worlds/panda.wbt 
* cd /root/catkin_ws && source devel/setup.bash && roslaunch webots_ros panda_moveit.launch

-----------------------------------
**Directly controlled by ROS**
* Change the controller of robot from "external" to "ROS"
* rosrun webots_ros robot_state_initializer

## How to integrate 

* Move_Group in MoveIt provides action client for external control, so we need to define action servers to get the trajectory planning and update the position of joints in Webots. (check webots-ros/scripts/trajectory_follower.py)
* We still need to define a joint_state_publisher to synchronize the position between simulation and control. (check webots-ros/scripts/joint_state_publisher.py)

## How to Turn Xacro to Urdf , then to Proto file in Webots

1. add a launch file to define where the Xacro file is. (see franka_description/launch/panda.launch)
2. cd robots && rosrun xacro xacro --inorder -o panda_arm_hand.urdf panda_arm_hand.urdf.xacro
3. install [urdf2webots](https://github.com/cyberbotics/urdf2webots)
4. python3 -m urdf2webots.importer --input=panda_arm_hand.urdf.xacro -rotation="1 0 0 -1.5708" --init-pos="[0, -0.785, 0, -2.356, 0, 1.571, 0.785]" --tool-slot=panda_link8 
