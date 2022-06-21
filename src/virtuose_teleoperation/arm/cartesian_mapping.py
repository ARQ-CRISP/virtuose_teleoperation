#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

from virtuose.msg import out_virtuose_physical_pose
from visualization_msgs.msg import Marker
from relaxed_ik.msg import EEPoseGoals
from PyKDL import Frame, Rotation, Vector
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
from tf.transformations import quaternion_multiply, quaternion_conjugate
import tf_conversions.posemath as pm

from scipy.spatial.transform.rotation import Rotation as R
import rospy
import numpy as np
from copy import deepcopy


def fromTransform(msg):
    pose = Frame()
    pose.p = Vector(msg.translation.x, msg.translation.y, msg.translation.z)
    pose.m = Rotation.Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w)#Rotation.Quaternion: Constructs a rotation from an x, y, z, w quaternion descripion
    return pose

# def qv_mult(q1, v1):
#     # comment this out if v1 doesn't need to be a unit vector
#     v1 = tf.transformations.unit_vector(v1)
#     q2 = list(v1)
#     q2.append(0.0)
#     return tf.transformations.quaternion_multiply(
#         tf.transformations.quaternion_multiply(q1, q2), 
#         tf.transformations.quaternion_conjugate(q1)
#     )[:3]

class Cartesian_Mapping:
    def __init__(self, init_pose=None):
        self.virtuose_sub = rospy.Subscriber("/out_virtuose_physical_pose", out_virtuose_physical_pose, self.callback) #rosmsg show out_virtuose_physical_pose 
        self.ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)
        self.latest_pose = init_pose #if init_pose is not None else Frame()
        self.init_pose = init_pose
        rospy.loginfo('Cartesian Mapper: Initialized!')
      
    def transform_pose(self, pose):
        """method that implements the pose mapping from haption to ur5 frame.

        Args:
            pose (PyKDL.Frame): pose to adapt for the teleoperation

        Returns:
            PyKDL.Frame: Pose adapted for teleoperation
        """
        # rot = [ 0, 0.7071068, 0, 0.7071068 ] # 90 deg on y axis
        # rot = [ 0, -0.7071068, 0, 0.7071068 ] # -90 deg on y axis
        #rot = [ 0, 0, 0, 1 ] # 0 deg on y axis [up->for, forw->down, left->right(if pos_fixed[0] *= -1 )]
        # rot = [ -0.7071068, 0, 0, 0.7071068 ] # -90 deg on x axis
        rot =[ 0, 0, -0.3826834, 0.9238795 ] #45deg y axis
        rot = [ 0, 0.3420201, 0, 0.9396926 ]# -40 deg y axis
        #[ 0, 0.3826834, 0, 0.9238795 ]
        rot2 = [ 0.7071068, 0, 0, 0.7071068 ] # 90 deg on x axis (Best until now) left right swapped
        
        # rot = [-0.7071067811865475, 0.7071067811865476, 0 ,0] # 
        # rot = [-0.7071067811865475, -0.7071067811865476, 0 ,0] # 
        # rot = [-0.7071067811865475, 0 , 0.7071067811865476 ,0] # 
        # rot = [-0.7071067811865475, 0 , -0.7071067811865476 ,0] # 
        # rot = [-0.7071067811865475, 0 , 0, 0.7071067811865476] # 
        #rot = [-0.7071067811865475, 0 , 0, -0.7071067811865476] #  (funziona uguale a -90)
        
        #rot2 = [ 0, 1, 0, 0 ]
        
        #Rotation.Quaternion: Constructs a rotation from an x, y, z, w quaternion descripion
        #Frame(rot, pos): Construct a frame from a rotation and a vector
        orient_bias = Frame(Rotation.Quaternion(*rot), Vector())
        print("Rotation.Quaternion(*rot)",Rotation.Quaternion(*rot))
        print("orient_bias",orient_bias)
        orient_bias2 = Frame(Rotation.Quaternion(*rot2), Vector())######
        print("orient_bias.Inverse()",orient_bias.Inverse())

        #Hamilton product H(a, b) https://math.stackexchange.com/questions/40164/how-do-you-rotate-a-vector-by-a-unit-quaternion
        new_pose = orient_bias * pose * orient_bias.Inverse()
        print("new_pose",new_pose)
        new_pose = orient_bias2 * new_pose * orient_bias2.Inverse()
        
        return new_pose

    

    def callback(self, msg):
        # msg = out_virtuose_physical_pose()
        # rospy.loginfo('received Haption data')
        current_pose = fromTransform(msg.virtuose_physical_pose)
        print("current_pose_BEFORE", current_pose)

        current_pose = self.transform_pose(current_pose)
        print("current_pose_AFTER", current_pose)
        pos_fixed = list(current_pose.p)
        pos_fixed[0] *= -1
        current_pose.p = Vector(*pos_fixed)
        # current_pose.p 
        if self.init_pose is not None:
            ee_pose_goals_msg = EEPoseGoals()
            target = Frame()
            target.p = self.init_pose.p - self.latest_pose.p
            target.M = current_pose.M
            ee_pose_goals_msg.ee_poses.append(pm.toMsg(target))
            self.ee_pose_goals_pub.publish(ee_pose_goals_msg)
        else:
            self.init_pose = current_pose
            
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