#!/usr/bin/env python3.6

import actionlib
import actionlib_msgs
import rospy


from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def trajectory_is_finite(trajectory):
    """Check if trajectory contains infinite or NaN value."""
    for point in trajectory.points:
        for position in point.positions:
            if math.isinf(position) or math.isnan(position):
                return False
        for velocity in point.velocities:
            if math.isinf(velocity) or math.isnan(velocity):
                return False
    return True

def reorder_trajectory_joints(trajectory, joint_names):
    """Reorder the trajectory points according to the order in joint_names."""
    order = [trajectory.joint_names.index(j) for j in joint_names]
    print(joint_names)
    new_points = []
    for point in trajectory.points:
        new_points.append(JointTrajectoryPoint(
            positions=[point.positions[i] for i in order])
    trajectory.joint_names = joint_names
    trajectory.points = new_points

def within_tolerance(a_vec, b_vec, tol_vec):
    """Check if two vectors are equals with a given tolerance."""
    for a, b, tol in zip(a_vec, b_vec, tol_vec):
        if abs(a - b) > tol:
            return False
    return True

def sample_trajectory(trajectory, t):
    """Return (q, qdot, qddot) for sampling the JointTrajectory at time t,
       the time t is the time since the trajectory was started."""
    # First point
    if t <= 0.0:
        return copy.deepcopy(trajectory.points[0])
    # Last point
    if t >= trajectory.points[-1].time_from_start.to_sec():
        return copy.deepcopy(trajectory.points[-1])
    # Finds the (middle) segment containing t
    i = 0
    while trajectory.points[i + 1].time_from_start.to_sec() < t:
        i += 1
    return interp_cubic(trajectory.points[i], trajectory.points[i + 1], t)

class GripperFollower(object):
    """
    Create a server listens to the gripper controller
    
    """

    def __init__(self, robot, jointStatePublisher, nodeName, goal_time_tolerance=None):
        self.robot = robot
        self.jointPrefix = 'panda_finger_joint'
        self.prefixedJointNames = [self.jointPrefix + str(s+1) for s in range(2)]
        self.jointStatePublisher = jointStatePublisher
        self.timestep = int(robot.getBasicTimeStep())
        self.motors = []
        self.sensors = []
        for name in self.prefixedJointNames:
            self.motors.append(robot.getDevice(name))
            self.sensors.append(robot.getDevice(name + '_sensor'))
            self.sensors[-1].enable(self.timestep)
        self.goal_handle = None
        self.trajectory = None
        self.joint_goal_tolerances = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
        self.server = actionlib.ActionServer(nodeName + "panda_hand_controller/follow_joint_trajectory",
                                             FollowJointTrajectoryAction,
                                             self.on_goal, self.on_cancel, auto_start=False)

    
    def init_trajectory(self):
        """Initialize a new target trajectory."""
        state = self.jointStatePublisher.last_joint_states
        self.trajectory_t0 = self.robot.getTime()
        self.trajectory = JointTrajectory()
        self.trajectory.joint_names = self.prefixedJointNames
        self.trajectory.points = [JointTrajectoryPoint(
            positions=state.position if state else [0] * 2,
            time_from_start=rospy.Duration(0.0))]

    def start(self):
        """Initialize and start the action server."""
        self.init_trajectory()
        self.server.start()
        print("The action server for this driver has been started")

    def on_goal(self, goal_handle):
        """Handle a new goal trajectory command."""
        # Checks if the joints are just incorrect
        if set(goal_handle.get_goal().trajectory.joint_names) != set(self.prefixedJointNames):
            rospy.logerr("Received a goal with incorrect joint names: (%s)" %
                         ', '.join(goal_handle.get_goal().trajectory.joint_names))
            goal_handle.set_rejected()
            return

        if not trajectory_is_finite(goal_handle.get_goal().trajectory):
            rospy.logerr("Received a goal with infinites or NaNs")
            goal_handle.set_rejected(text="Received a goal with infinites or NaNs")
            return
        

        # Orders the joints of the trajectory according to joint_names
        reorder_trajectory_joints(goal_handle.get_goal().trajectory, self.prefixedJointNames)

        # Inserts the current setpoint at the head of the trajectory
        now = self.robot.getTime()
        point0 = sample_trajectory(self.trajectory, now - self.trajectory_t0)
        point0.time_from_start = rospy.Duration(0.0)
        goal_handle.get_goal().trajectory.points.insert(0, point0)
        self.trajectory_t0 = now

        # Replaces the goal
        self.goal_handle = goal_handle
        self.trajectory = goal_handle.get_goal().trajectory
        goal_handle.set_accepted()

    def on_cancel(self, goal_handle):
        """Handle a trajectory cancel command."""
        if goal_handle == self.goal_handle:
            # stop the motors
            for i in range(len(self.prefixedJointNames)):
                self.motors[i].setPosition(self.sensors[i].getValue())
            self.goal_handle.set_canceled()
            self.goal_handle = None
        else:
            goal_handle.set_canceled()

    def update(self):
        if self.robot and self.trajectory:
            now = self.robot.getTime()
            if (now - self.trajectory_t0) <= self.trajectory.points[-1].time_from_start.to_sec():  # Sending intermediate points
                setpoint = sample_trajectory(self.trajectory, now - self.trajectory_t0)
                for i in range(len(setpoint.positions)):
                    self.motors[i].setPosition(setpoint.positions[i])
                    # Velocity control is not used on the real robot and gives bad results in the simulation
                    # self.motors[i].setVelocity(math.fabs(setpoint.velocities[i]))
            elif self.goal_handle and self.goal_handle.get_goal_status().status == actionlib_msgs.msg.GoalStatus.ACTIVE:
                # All intermediate points sent, sending last point to make sure we reach the goal.
                last_point = self.trajectory.points[-1]
                state = self.jointStatePublisher.last_joint_states
                position_in_tol = within_tolerance(state.position, last_point.positions, self.joint_goal_tolerances)
                setpoint = sample_trajectory(self.trajectory, self.trajectory.points[-1].time_from_start.to_sec())
                for i in range(len(setpoint.positions)):
                    self.motors[i].setPosition(setpoint.positions[i])
                    # Velocity control is not used on the real robot and gives bad results in the simulation
                    # self.motors[i].setVelocity(math.fabs(setpoint.velocities[i]))
                position_in_tol = within_tolerance(state.position, last_point.positions, [0.1] * 2)
                velocity_in_tol = within_tolerance(state.velocity, last_point.velocities, [0.05] * 2)
                if position_in_tol and velocity_in_tol:
                    # The arm reached the goal (and isn't moving) => Succeeded
                    self.goal_handle.set_succeeded()