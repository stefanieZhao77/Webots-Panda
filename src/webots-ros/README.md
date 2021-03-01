## webots_ros

This is the official package provided by Webots.

### Methods
1. Directly connection between ROS and Webots.
  * choose the controller 'ROS' in Webots. All the nodes in Webots provide a service server, you can register a client to send messages. See the robot_state_initializer.cpp
2. Add Moveit 
  * Move_group listens on the /joint_states. We can synchronize information with a joint state publisher.
  * We need a robot_state_publisher to publish TF information.
  * We need a FollowJointTrajectoryAction server on the robot. Move_group will talk to this controller.
  * controller in Webots should be 'extern'
  * See panda_robots_ros.py

You can define specific robots by 'name', and run different nodes to control multiple robots.