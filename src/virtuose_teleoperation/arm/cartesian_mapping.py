#!/usr/bin/env python
"""
Cartesian Mapping Module for Haption to UR5 Teleoperation

This module implements a ROS node for mapping cartesian poses between
a Haption haptic device and a UR5 robot arm for teleoperation purposes.

"""

from __future__ import division, absolute_import, print_function

import rospy
import numpy as np
from copy import deepcopy

from PyKDL import Frame, Rotation, Vector
import tf_conversions.posemath as pm

from virtuose.msg import out_virtuose_physical_pose
from relaxed_ik.msg import EEPoseGoals
from geometry_msgs.msg import PoseStamped

def from_transform(msg):
    """Convert a Transform message to a PyKDL Frame."""
    pose = Frame()
    pose.p = Vector(msg.translation.x, msg.translation.y, msg.translation.z)
    pose.M = Rotation.Quaternion(msg.rotation.y, msg.rotation.z, msg.rotation.x, msg.rotation.w)
    return pose

def from_pose_stamped(msg):
    """Convert a PoseStamped message to a PyKDL Frame."""
    pose = Frame()
    pose.p = Vector(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    pose.M = Rotation.Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    return pose

class CartesianMapping:
    """
    A class to handle cartesian mapping between Haption and UR5.

    This class subscribes to Haption pose topics, transforms the poses,
    and publishes goal poses for the UR5 robot.
    """

    def __init__(self, init_pose=None):
        # Initialize subscribers
        self.virtuose_sub = rospy.Subscriber("/out_virtuose_physical_pose", out_virtuose_physical_pose, self.haption_callback)
        self.ee_pose_ur5_sub = rospy.Subscriber("/cartesian_pose_UR5", PoseStamped, self.ur5_cartesian_pose_callback)

        # Initialize publishers
        self.ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)
        self.ee_pose_goals_pub_anti = rospy.Publisher('/ee_pose_goals_anti', EEPoseGoals, queue_size=5)
        self.ee_pose_ur5_antiRotation_pub = rospy.Publisher("/cartesian_pose_antiRotation_UR5", PoseStamped, queue_size=5)

        # Initialize pose variables
        self.latest_pose = init_pose
        self.init_pose = init_pose
        self.init_rotation = None
        self.init_pose_anti = None
        self.latest_pose_anti = None

        rospy.loginfo('Cartesian Mapper: Initialized!')

    def transform_pose(self, pose):
        """
        Transform pose from Haption to UR5 frame.

        Args:
            pose (PyKDL.Frame): Pose to adapt for teleoperation

        Returns:
            PyKDL.Frame: Pose adapted for teleoperation
        """
        # Define rotation quaternions
        rot = [0.7071068, 0, 0, 0.7071068]  # 90 deg rotation on x-axis
        rot2 = [0, 0.7071068, 0, 0.7071068]  # 90 deg rotation on y-axis for frontal setup

        # Create rotation frames
        orient_bias = Frame(Rotation.Quaternion(*rot), Vector())
        orient_bias2 = Frame(Rotation.Quaternion(*rot2), Vector())

        # Apply rotations
        new_pose = orient_bias * pose * orient_bias.Inverse()
        new_pose = orient_bias2 * new_pose * orient_bias2.Inverse()

        return new_pose

    def anti_transform_pose(self, pose):
        """
        Transform pose from UR5 to Haption frame.

        Args:
            pose (PyKDL.Frame): Pose to adapt for teleoperation

        Returns:
            PyKDL.Frame: Pose adapted for teleoperation
        """
        anti_rot = [0, -0.2079117, 0, 0.9781476]
        anti_rot2 = [-0.7071068, 0, 0, 0.7071068]

        orient_bias = Frame(Rotation.Quaternion(*anti_rot), Vector())
        orient_bias2 = Frame(Rotation.Quaternion(*anti_rot2), Vector())

        new_pose = orient_bias * pose * orient_bias.Inverse()
        new_pose = orient_bias2 * new_pose * orient_bias2.Inverse()

        return new_pose

    def haption_callback(self, msg):
        """Callback for Haption pose messages."""
        current_pose = from_transform(msg.virtuose_physical_pose)
        current_rotation = from_transform(msg.virtuose_physical_pose)
        
        current_pose = self.transform_pose(current_pose)
        anti_pose = self.anti_transform_pose(current_pose)

        # Invert x-axis
        pos_fixed = list(current_pose.p)
        pos_fixed[0] *= -1
        current_pose.p = Vector(*pos_fixed)

        pos_fixed_anti = list(anti_pose.p)
        pos_fixed_anti[0] *= -1
        anti_pose.p = Vector(*pos_fixed_anti)

        if self.init_pose is not None:
            self.publish_ee_pose_goals(current_pose, current_rotation, anti_pose)
        else:
            self.initialize_poses(current_pose, current_rotation, anti_pose)

        self.latest_pose = current_pose
        self.latest_pose_anti = anti_pose

    def publish_ee_pose_goals(self, current_pose, current_rotation, anti_pose):
        """Publish end-effector pose goals."""
        ee_pose_goals_msg = EEPoseGoals()
        target = Frame()
        target.p = self.init_pose.p - self.latest_pose.p
        target.M = current_rotation.M * self.init_rotation.M.Inverse()

        ee_pose_goals_msg.ee_poses.append(pm.toMsg(target))
        self.ee_pose_goals_pub.publish(ee_pose_goals_msg)

        ee_pose_anti_msg = EEPoseGoals()
        anti_target = Frame()
        anti_target.p = self.init_pose_anti.p - self.latest_pose_anti.p
        anti_target.M = Rotation()
        ee_pose_anti_msg.ee_poses.append(pm.toMsg(target))
        self.ee_pose_goals_pub_anti.publish(ee_pose_anti_msg)

    def initialize_poses(self, current_pose, current_rotation, anti_pose):
        """Initialize pose variables."""
        self.init_pose = current_pose
        self.init_rotation = current_rotation
        self.init_pose_anti = anti_pose

    def ur5_cartesian_pose_callback(self, msg):
        """Callback for UR5 cartesian pose messages."""
        current_pose = from_pose_stamped(msg)
        current_pose = self.anti_transform_pose(current_pose)

        current_pose_goals_msg = PoseStamped()
        current_pose_goals_msg.header.stamp = rospy.Time.now()
        current_pose_goals_msg.pose.position.x = current_pose.p.x()
        current_pose_goals_msg.pose.position.y = current_pose.p.y()
        current_pose_goals_msg.pose.position.z = current_pose.p.z()
        current_pose_goals_msg.pose.orientation.x = 0
        current_pose_goals_msg.pose.orientation.y = 0
        current_pose_goals_msg.pose.orientation.z = 0
        current_pose_goals_msg.pose.orientation.w = 1

        self.ee_pose_ur5_antiRotation_pub.publish(current_pose_goals_msg)

if __name__ == '__main__':
    rospy.init_node('cartesian_mapping_node')
    cartesian_mapper = CartesianMapping()
    rospy.spin()
