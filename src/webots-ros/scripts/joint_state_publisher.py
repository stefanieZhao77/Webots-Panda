#!/usr/bin/env python3.6

import rospy
from sensor_msgs.msg import JointState


class JointStatePublisher(object):
    """
    publish as a ROS topic of joint state with the data from position sensors
    """
    # jointNames =[
    #     "panda_joint1",
    #     "panda_joint2",
    #     "panda_joint3",
    #     "panda_joint4",
    #     "panda_joint5",
    #     "panda_joint6",
    #     "panda_joint7"
    # ]

    def __init__(self, robot, jointPrefix, nodeName):
        self.robot = robot
        self.jointPrefix = 'panda_joint'
        self.jointNames = [self.jointPrefix + str(s+1) for s in range(7)]
        self.gripper = ['panda_finger_joint1', 'panda_finger_joint2']
        self.jointNames = self.jointNames + self.gripper
        self.motors = []
        self.sensors = []
        self.timestep = int(robot.getBasicTimeStep())
        # print(self.timestep)
        self.last_joint_states = None
        self.previousTime = 0
        self.previousPosition = []
        for name in self.jointNames:
            self.motors.append(robot.getDevice(name))
            self.sensors.append(robot.getDevice(name + '_sensor'))
            self.sensors[-1].enable(self.timestep)
            self.previousPosition.append(0)
        self.publisher = rospy.Publisher(nodeName + 'joint_states', JointState, queue_size=1)

    def publish(self):
        """
        publish the joint states to tf2
        
        """
        msg = JointState()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "From simulation state data"
        msg.name = [s for s in self.jointNames]
        msg.position = []
        timeDifference = self.robot.getTime() - self.previousTime
        for i in range(len(self.sensors)):
            value = self.sensors[i].getValue()
            msg.position.append(value)
            msg.velocity.append((value - self.previousPosition[i]) / timeDifference if timeDifference > 0 else 0.0)
            self.previousPosition[i] = value
        msg.effort = [0] * 9
        rospy.loginfo("msg is " + str(msg))
        self.publisher.publish(msg)
        self.last_joint_states = msg
        self.previousTime = self.robot.getTime()
