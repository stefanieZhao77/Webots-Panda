#!/usr/bin/env python3.6

import argparse
import rospy

from controller import Robot
from joint_state_publisher import JointStatePublisher
from trajectory_follower import TrajectoryFollower
# from gripper_control import GripperFollower
from rosgraph_msgs.msg import Clock



parser = argparse.ArgumentParser()
parser.add_argument('--node-name', dest='nodeName', default='Panda', help='Specifies the name of the node.')
arguments, unknown = parser.parse_known_args()

rospy.init_node(arguments.nodeName, disable_signals=True)

jointPrefix = rospy.get_param('prefix', '')
if jointPrefix:
    print('Setting prefix to %s' % jointPrefix)

# initialPosition = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]

robot = Robot()
nodeName = arguments.nodeName + '/' if arguments.nodeName != 'Panda' else ''
# for i in range(len(initialPosition)):
#     name = 'panda_joint'+str(i+1)
#     robot.getMotor(name).setPosition(initialPosition[i])

jointStatePublisher = JointStatePublisher(robot, jointPrefix, nodeName)
trajectoryFollower = TrajectoryFollower(robot, jointStatePublisher, jointPrefix, nodeName)
trajectoryFollower.start()

# gripperFollower = GripperFollower(robot, jointStatePublisher, jointPrefix, nodeName)
# gripperFollower.start()


# we want to use simulation time for ROS
clockPublisher = rospy.Publisher('clock', Clock, queue_size=1)
if not rospy.get_param('use_sim_time', False):
    rospy.logwarn('use_sim_time is not set!')

timestep = int(robot.getBasicTimeStep())


while robot.step(timestep) != -1 and not rospy.is_shutdown():
    jointStatePublisher.publish()
    trajectoryFollower.update()
    # gripperFollower.update()

    # pulish simulation clock
    msg = Clock()
    time = robot.getTime()
    msg.clock.secs = int(time)
    # round prevents precision issues that can cause problems with ROS timers
    msg.clock.nsecs = int(1000 * (time - msg.clock.secs))
    clockPublisher.publish(msg)