#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
from virtuose.msg import out_virtuose_physical_pose
from relaxed_ik.msg import EEPoseGoals
from PyKDL import Frame, Rotation, Vector
import tf_conversions.posemath as pm
from copy import deepcopy

def fromTransform(msg):
    pose = Frame()
    pose.p = Vector(msg.translation.x, msg.translation.y, msg.translation.z)
    pose.m = Rotation.Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w)
    return pose

class Cartesian_Mapping:
    def __init__(self, init_pose=None):
        self.virtuose_sub = rospy.Subscriber("/out_virtuose_physical_pose", out_virtuose_physical_pose, self.callback) #rosmsg show out_virtuose_physical_pose 
        self.ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)
        self.latest_pose = init_pose #if init_pose is not None else Frame()
      
    def transform_pose(self, pose):
        """method that implements the pose mapping from haption to ur5 frame.

        Args:
            pose (PyKDL.Frame): pose to adapt for the teleoperation

        Returns:
            PyKDL.Frame: Pose adapted for teleoperation
        """
        orient_bias = Rotation.Quaternion(0., 0., 0., 1.)
        # pose = pose * orient_bias
        return pose
    
        
    def callback(self, msg):
        # msg = out_virtuose_physical_pose()
        
        current_pose = pm.fromTf()
        current_pose = self.map_pose(current_pose)
        if self.latest_pose is not None:
            ee_pose_goals_msg = EEPoseGoals()
            target = Frame()
            target.p = current_pose.p - self.latest_pose.p
            target.M = current_pose.M
            ee_pose_goals_msg.ee_poses.append(pm.toMsg(target))
            self.ee_pose_goals_pub.publish(ee_pose_goals_msg)
            
        self.latest_pose = current_pose
        
        
        


'''
    [relaxed_ik_ros1/EEPoseGoals]:
    std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
    geometry_msgs/Pose[] ee_poses
    geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
    geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
'''

'''
rosmsg show out_virtuose_physical_pose
[virtuose/out_virtuose_physical_pose]:
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Transform virtuose_physical_pose
  geometry_msgs/Vector3 translation
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion rotation
    float64 x
    float64 y
    float64 z
    float64 w

'''