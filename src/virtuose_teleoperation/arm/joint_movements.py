#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Joint Movement Manager for ROS-based robotic systems.

This module provides classes for managing joint movements in both simulated
and real robotic environments using ROS.

Classes:
    JointMovementManager: Abstract base class for joint movement management.
    SimJointMovementManager: Simulated joint movement manager.
    RealJointMovementManager: Real robot joint movement manager.

Author: [Gabriele Giudici]
"""

from __future__ import print_function, absolute_import, division
from abc import ABCMeta, abstractmethod
from collections import namedtuple
from copy import deepcopy

import rospy
import actionlib
import numpy as np

from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

def sigmoid(x):
    """Sigmoid activation function."""
    return 1 / (1 + np.exp(-x))

class JointMovementManager(object):
    """
    Abstract base class for joint movement management.

    This class defines the interface and common functionality for
    both simulated and real joint movement managers.
    """

    __metaclass__ = ABCMeta

    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    ur_state = namedtuple('UR_State', ('position', 'velocity', 'effort'))

    @classmethod
    def generate_manager(cls, init_state, sim=True):
        """Factory method to create appropriate joint movement manager."""
        if sim:
            return [c for c in cls.__subclasses__() if c.__name__.lower().startswith('sim')][0](init_state)
        else:
            return [c for c in cls.__subclasses__() if c.__name__.lower().startswith('real')][0](init_state)

    def __init__(self, init_state=[0., 0., 0., 0., 0., 0.]):
        """Initialize the joint movement manager."""
        self.stopped = False
        self.N = 5
        self.last_j_state_target = init_state
        self.last_j_vel = [0.0] * len(init_state)
        self.last_j_trajectory = JointTrajectory(joint_names=self.jnames)
        self.current_j_state = self.ur_state(init_state, [0.0] * len(init_state), [0.0] * len(init_state))

    @abstractmethod
    def define_trajectory(self, js_postion, j_velocity=None, duration=None):
        """Define a joint trajectory."""
        raise NotImplementedError

    @abstractmethod
    def go_to(self, target):
        """Execute movement to target joint state."""
        raise NotImplementedError

    @abstractmethod
    def terminate(self):
        """Terminate the joint movement manager."""
        raise NotImplementedError

    @abstractmethod
    def emergency_stop(self):
        """Perform an emergency stop."""
        raise NotImplementedError

    def generate_movement(self, j_target):
        """Generate a movement to the target joint state."""
        old_jangles = np.asarray((self.current_j_state.position))
        dist = np.absolute((np.asarray(j_target) - np.asarray(old_jangles)))
        T = rospy.Duration(dist.max()/(np.pi/20.))
        velocity = ((np.asarray(j_target) - np.asarray(old_jangles)) / T.to_sec()).tolist()
        if np.any(dist > 1e-3):
            target = self.define_trajectory(j_target, velocity, T)
            self.go_to(target)
            self.last_j_state_target = j_target
            self.last_j_vel = velocity

    def listen_j_state(self, msg):
        """Update current joint state based on received message."""
        ur_joint_idx = [msg.name.index(j) for j in self.jnames]
        pos = [msg.position[idx] for idx in ur_joint_idx]
        velocity = [msg.velocity[idx] for idx in ur_joint_idx] if len(msg.velocity) > 0 else [0.0] * len(pos)
        effort = [msg.effort[idx] for idx in ur_joint_idx] if len(msg.effort) > 0 else [0.0] * len(pos)
        self.current_j_state = self.ur_state(pos, velocity, effort)

    def interp_traj(self, j_target, max_vel, max_duration, N=19):
        """Interpolate trajectory between current state and target."""
        duration = max_duration.to_sec() if isinstance(max_duration, rospy.Duration) else max_duration
        old_jangles2 = np.asarray((self.current_j_state.position))
        js_position = np.asarray(j_target)
        
        dist2 = np.absolute((js_position - old_jangles2))
        T = dist2.max()/(np.pi/((N+1)*1.5e0))
        t_steps = np.linspace(0, T, N)
        
        steps = sigmoid(10*(np.linspace(0, 1, N)-0.5)).reshape(-1,1)
        pos_steps = old_jangles2 * (1-steps) + (steps) * js_position
        vel_increments = np.sin(np.pi * steps)
        
        vel_steps = vel_increments * max_vel 
        
        return (t_steps[-1:], pos_steps[-1:], vel_steps[-1:])

class SimJointMovementManager(JointMovementManager):
    """Simulated joint movement manager."""

    joint_states_topic = '/move_group/fake_controller_joint_states'
    joint_listener_topic = '/joint_states'

    def __init__(self, init_state=[0., 0., 0., 0., 0., 0.]):
        """Initialize the simulated joint movement manager."""
        super(SimJointMovementManager, self).__init__(init_state)
        rospy.loginfo("[%s] Connecting to fake controllers ...", rospy.get_name())
        self.joint_target_pub = rospy.Publisher(self.joint_states_topic, JointState, queue_size=1)
        rospy.loginfo("[%s] Connected to Sim Robot!", rospy.get_name())
        self.joint_listener = rospy.Subscriber(self.joint_listener_topic, JointState, self.listen_j_state)

    def define_trajectory(self, js_postion, j_velocity=None, duration=None):
        """Define trajectory for simulated environment."""
        return JointState(name=self.jnames, position=js_postion)

    def go_to(self, target):
        """Execute movement to target in simulated environment."""
        if not self.stopped:
            target.header.stamp = rospy.Time.now()
            self.joint_target_pub.publish(target)

    def emergency_stop(self):
        """Perform emergency stop in simulated environment."""
        self.stopped = True

    def restart(self):
        """Restart movement in simulated environment."""
        self.stopped = False

    def terminate(self):
        """Terminate the simulated joint movement manager."""
        self.joint_listener.unregister()
        self.joint_target_pub.unregister()

class RealJointMovementManager(JointMovementManager):
    """Real robot joint movement manager."""

    real_robot_action_server = '/scaled_pos_joint_traj_controller/follow_joint_trajectory'
    joint_listener_topic = '/joint_states'

    def __init__(self, init_state=[0., 0., 0., 0., 0., 0.]):
        """Initialize the real robot joint movement manager."""
        super(RealJointMovementManager, self).__init__(init_state)
        self.joint_target_pub = actionlib.SimpleActionClient(self.real_robot_action_server, FollowJointTrajectoryAction)
        rospy.loginfo("[%s] Waiting for control server...", rospy.get_name())
        self.joint_listener = rospy.Subscriber(
            self.joint_listener_topic, JointState, self.listen_j_state, queue_size=1)
        self.joint_target_pub.wait_for_server()
        rospy.loginfo("[%s] Connected to Robot!", rospy.get_name())

    def go_to(self, target):
        """Execute movement to target on real robot."""
        if len(target) > 0 and not self.stopped:
            goal = FollowJointTrajectoryGoal(
                trajectory=JointTrajectory(joint_names=self.jnames))
            goal.trajectory.points = target
            self.joint_target_pub.cancel_goal()
            self.joint_target_pub.send_goal(goal)

    def terminate(self):
        """Terminate the real robot joint movement manager."""
        self.joint_target_pub.cancel_all_goals()

    def emergency_stop(self):
        """Perform emergency stop on real robot."""
        self.stopped = True
        self.joint_target_pub.cancel_all_goals()

    def restart(self):
        """Restart movement on real robot."""
        self.stopped = False

    def define_trajectory(self, js_postion, j_velocity, duration):
        """Define trajectory for real robot."""
        old_jangles = self.current_j_state.position
        dist = np.absolute((np.asarray(js_postion) - np.asarray(old_jangles)))

        if np.any(dist > 1e-3):
            self.N = 11
            viapoints = self.interp_traj(js_postion, j_velocity, duration, self.N)

            traj_points = [
                JointTrajectoryPoint(
                    positions=pos.tolist(),
                    velocities=vel.tolist(),
                    time_from_start=rospy.Duration(t)) for t, pos, vel in zip(*viapoints)]
            return traj_points
        else:
            return []

if __name__ == '__main__':
    import yaml
    from rospkg.rospack import RosPack

    rospy.init_node('test')
    relaxed_ik_path = RosPack().get_path('relaxed_ik')
    relaxed_yaml_filename = rospy.get_param('~relaxed_ik_yaml', default='ur5_allegro_info_VIRTUOSE.yaml')
    yaml_file_path = relaxed_ik_path + '/src/' + '/RelaxedIK/Config/info_files/' + relaxed_yaml_filename
    
    with open(yaml_file_path) as f:
        yaml_file = yaml.load(f)
    
    manager = JointMovementManager.generate_manager(yaml_file['starting_config'], sim=True)
    
    while not rospy.is_shutdown():
        print(type(manager), manager.current_j_state, end="\r")
        rospy.sleep(1)
