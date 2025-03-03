#!/usr/bin/env python
"""
This module implements a Fake Cartesian Mapping for teleoperation.

It subscribes to virtuose physical pose messages and publishes EEPoseGoals
for relaxed inverse kinematics control.
"""

from __future__ import division, absolute_import, print_function

import rospy
import numpy as np
from copy import deepcopy

from virtuose.msg import out_virtuose_physical_pose
from relaxed_ik.msg import EEPoseGoals
from geometry_msgs.msg import PoseStamped, Pose
from PyKDL import Frame, Rotation, Vector
import tf_conversions.posemath as pm

def from_transform(msg):
    """
    Convert a transform message to a PyKDL Frame.

    Args:
        msg: Transform message containing translation and rotation.

    Returns:
        PyKDL Frame object.
    """
    pose = Frame()
    pose.p = Vector(msg.translation.x, msg.translation.y, msg.translation.z)
    pose.M = Rotation.Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w)
    return pose

def from_pose_stamped(msg):
    """
    Convert a PoseStamped message to a PyKDL Frame.

    Args:
        msg: PoseStamped message.

    Returns:
        PyKDL Frame object.
    """
    pose = Frame()
    pose.p = Vector(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    pose.M = Rotation.Quaternion(
        msg.pose.orientation.x, msg.pose.orientation.y,
        msg.pose.orientation.z, msg.pose.orientation.w
    )
    return pose

class FakeCartesianMapping:
    """
    Implements a Fake Cartesian Mapping for teleoperation.
    """

    def __init__(self, init_pose=None):
        """
        Initialize the FakeCartesianMapping object.

        Args:
            init_pose: Initial pose (optional).
        """
        rospy.loginfo("FAKE CARTESIAN CONTROLLER")

        self.virtuose_sub = rospy.Subscriber(
            "/out_virtuose_physical_pose",
            out_virtuose_physical_pose,
            self.callback_fake_virtuose
        )
        self.ee_pose_goals_pub = rospy.Publisher(
            '/relaxed_ik/ee_pose_goals',
            EEPoseGoals,
            queue_size=5
        )

        self.latest_pose = init_pose
        self.init_pose = init_pose
        self.init_rotation = None

        rospy.loginfo('Cartesian Mapper: Initialized!')

    def transform_pose(self, pose):
        """
        Transform the input pose for teleoperation.

        Args:
            pose: PyKDL Frame to be transformed.

        Returns:
            Transformed PyKDL Frame.
        """
        rot = [0, 0, 0, 1]
        rot2 = [0, 0, 0, 1]

        orient_bias = Frame(Rotation.Quaternion(*rot), Vector())
        orient_bias2 = Frame(Rotation.Quaternion(*rot2), Vector())

        new_pose = orient_bias * pose * orient_bias.Inverse()
        new_pose = orient_bias2 * new_pose * orient_bias2.Inverse()
        return new_pose

    def callback_fake_virtuose(self, msg):
        """
        Callback function for processing virtuose physical pose messages.

        Args:
            msg: out_virtuose_physical_pose message.
        """
        current_pose = from_transform(msg.virtuose_physical_pose)
        current_rotation = from_transform(msg.virtuose_physical_pose)

        # Invert x-axis
        pos_fixed = list(current_pose.p)
        pos_fixed[0] *= -1
        current_pose.p = Vector(*pos_fixed)

        if self.init_pose is not None:
            ee_pose_goals_msg = EEPoseGoals()
            target = Frame()
            target.p = self.latest_pose.p
            target.M = current_rotation.M

            ee_pose_goals_msg.ee_poses.append(pm.toMsg(target))
            self.ee_pose_goals_pub.publish(ee_pose_goals_msg)
        else:
            self.init_pose = current_pose
            self.init_rotation = current_rotation

        self.latest_pose = current_pose

if __name__ == "__main__":
    rospy.init_node('fake_cartesian_mapping')
    mapper = FakeCartesianMapping()
    rospy.spin()
