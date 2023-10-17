#!/usr/bin/env python
from __future__ import division, absolute_import, print_function

from virtuose.msg import out_virtuose_physical_pose
from visualization_msgs.msg import Marker
from relaxed_ik.msg import EEPoseGoals
from PyKDL import Frame, Rotation, Vector
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
from tf.transformations import quaternion_multiply, quaternion_conjugate
import tf_conversions.posemath as pm
from tf_conversions import fromTf, toMsg
import tf2_ros

from scipy.spatial.transform.rotation import Rotation as R
import rospy
import numpy as np


from copy import deepcopy

import csv


##########test
def fromTransform(msg):
    pose = Frame()
    pose.p = Vector(-msg.translation.x, -msg.translation.y, msg.translation.z) #- used to match the different frame from fake virtuose and RELIK
    
    # pose.M = Rotation.Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w)#Rotation.Quaternion: Constructs a rotation from an x, y, z, w quaternion descripion
    
    ###TO INCLUDE ORIENTATION FROM HAPTION -msg.rotation.y(pitch), msg.rotation.x(yaw), -msg.rotation.z(roll), msg.rotation.w
    
    #standard handler in any configuration (24 or 90 deg)
    # pose.M = Rotation.Quaternion(-msg.rotation.y, msg.rotation.x, -msg.rotation.z, msg.rotation.w)#*Rotation.Quaternion( 1, 0, 0, 0)#Rotation.Quaternion: Constructs a rotation from an x, y, z, w quaternion descripion
    # #HGlove
    # # Define the rotation of the handler relative to the original base frame
    # # handler_quat = Rotation.Quaternion( 0.92, -0.01, -0.32, 0.18)
    
    # handler_quat = Rotation.Quaternion( -0.5765579, 0.6486277, 0, 0.4968532 )#q1 and q2
    
    # handler_quat_inv = handler_quat.Inverse()

    # quat=Rotation.Quaternion(-msg.rotation.y, msg.rotation.x, -msg.rotation.z, msg.rotation.w)
    new_quat = Rotation.Quaternion(msg.rotation.y, msg.rotation.z, msg.rotation.x, msg.rotation.w)
    pose.M =new_quat

    # pose.M = Rotation.Quaternion(-msg.rotation.y, msg.rotation.x, -msg.rotation.z, msg.rotation.w)*Rotation.Quaternion( 0.92, -0.01, -0.32, 0.18)#Rotation.Quaternion: Constructs a rotation from an x, y, z, w quaternion descripion
    return pose

def fromPoseStamped(msg):
    pose = Frame()
    #msg=PoseStamped()
    pose.p = Vector(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    pose.M = Rotation.Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)#Rotation.Quaternion: Constructs a rotation from an x, y, z, w quaternion descripion
    return pose

def transform_to_vec(T):
    t = T.transform.translation
    r = T.transform.rotation
    return [[t.x, t.y, t.z], [r.x, r.y, r.z, r.w]]

class FakeCartesian_Mapping:

    def __init__(self, init_pose=None):
        print("FAKE CARTESIAN CONTROLLER")

        self.virtuose_sub = rospy.Subscriber("/out_virtuose_physical_pose", out_virtuose_physical_pose, self.callback_FAKEVIRTUOSE) #rosmsg show out_virtuose_physical_pose 
        self.ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)

        self.latest_pose = init_pose #if init_pose is not None else Frame()
        self.init_pose = init_pose
        #self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1), True)


        rospy.loginfo('Cartesian Mapper: Initialized!')



    def transform_pose(self, pose):
        """method that implements the pose mapping from haption to ur5 frame.

        Args:
            pose (PyKDL.Frame): pose to adapt for the teleoperation

        Returns:
            PyKDL.Frame: Pose adapted for teleoperation
        """
        #ROTATIONS

        """ It is calculated as 45(angle of rotation of the handel) 
        minus 15(angle between the axis parallel to lateral axis of UR5)
        """

        #Euler angles of multiple axis rotations (degrees)
        rot = [ 0, 0, 0, 1 ] 
        rot2 =[ 0, 0, 0, 1 ] 

        orient_bias = Frame(Rotation.Quaternion(*rot), Vector())
        orient_bias2 = Frame(Rotation.Quaternion(*rot2), Vector())######

        #Hamilton product H(a, b) https://math.stackexchange.com/questions/40164/how-do-you-rotate-a-vector-by-a-unit-quaternion
        new_pose = orient_bias * pose * orient_bias.Inverse()

        new_pose = orient_bias2 * new_pose * orient_bias2.Inverse() #white configuration
        return new_pose



    def callback_FAKEVIRTUOSE(self, msg):
        current_pose = fromTransform(msg.virtuose_physical_pose)
        current_rotation = fromTransform(msg.virtuose_physical_pose)
        current_pose = self.transform_pose(current_pose)

        pos_fixed = list(current_pose.p)
        pos_fixed[0] *= -1
        current_pose.p = Vector(*pos_fixed)

        #####################
        ####################    
        if self.init_pose is not None:
            ee_pose_goals_msg = EEPoseGoals()
            target = Frame()
            target.p = self.init_pose.p - self.latest_pose.p
            target.M = Rotation() #current_pose.M
            # target.M = current_pose.M
            ####################COMMENT OUT target.M = current_rotation.M*self.init_rotation.M.Inverse() FOR DISABLING ROTATION, IF UNCOMMENTED ROTATION IS ACTIVE############### 
            # target.M = current_rotation.M*self.init_rotation.M.Inverse()
            # print("target.M \n",target.M)
            # print("current_pose.m \n\n\n\n\n", current_rotation.M)
            ee_pose_goals_msg.ee_poses.append(pm.toMsg(target))
            self.ee_pose_goals_pub.publish(ee_pose_goals_msg)
                        
        else:
            self.init_pose = current_pose
            self.init_rotation= current_rotation
            # self.init_rotation.M=self.init_rotation.M*Rotation.Quaternion(0.7071068, 0, 0, 0.7071068) #current_pose.M
     
        # print("initial _rotation",self.init_rotation)    
        self.latest_pose = current_pose
    
            