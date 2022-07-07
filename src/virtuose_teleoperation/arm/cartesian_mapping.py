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
    pose.M = Rotation.Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w)#Rotation.Quaternion: Constructs a rotation from an x, y, z, w quaternion descripion
    
    ###TO INCLUDE ORIENTATION FROM APTION
    #pose.M = Rotation.Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w)#Rotation.Quaternion: Constructs a rotation from an x, y, z, w quaternion descripion
    #print("pose.m",pose.M)
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
        rot = [ 0.7071068, 0, 0, 0.7071068 ] # 90 deg on x axis required to match ur5
        """ It is calculated as 45(angle of rotation of the handel) 
        minus 15(angle between the axis parallel to lateral axis of UR5)
        """
        rot2 =[0, 0.1305262, 0, 0.9914449] #15 deg on y axis depends on the orientation of Virtuose respect to the table.
    
    #MODIFY HERE TO REGULATE THE ANGULAR OFFSET BETWEEN UR5 AND VIRTUOSE
        #to adjust the rot3 matrix, you need to use the green tablepad or another ortogonal reference
        #forward->-z  #right->x #up->y
        #you can read the cartesian position from:  rostopic echo /relaxed_ik/ee_pose_goals 
        rot3 = [ 0, 0.0784591, 0, 0.9969173 ] #9 MORE DEG ON Y AXIS


        #Rotation.Quaternion: Constructs a rotation from an x, y, z, w quaternion descripion
        #Frame(rot, pos): Construct a frame from a rotation and a vector
        orient_bias = Frame(Rotation.Quaternion(*rot), Vector())

        orient_bias2 = Frame(Rotation.Quaternion(*rot2), Vector())#######
        orient_bias3 = Frame(Rotation.Quaternion(*rot3), Vector())#######


        #Hamilton product H(a, b) https://math.stackexchange.com/questions/40164/how-do-you-rotate-a-vector-by-a-unit-quaternion
        new_pose = orient_bias * pose * orient_bias.Inverse()

        #print("new_pose.p",new_pose.p)
        #print("new_pose.M",new_pose.M)
        new_pose = orient_bias2 * new_pose * orient_bias2.Inverse()#rot2 can be included in rot3 not particular meaning
        new_pose = orient_bias3 * new_pose * orient_bias2.Inverse()

        return new_pose

    

    def callback(self, msg):
        # msg = out_virtuose_physical_pose()
        # rospy.loginfo('received Haption data')
        current_pose = fromTransform(msg.virtuose_physical_pose)
        #print("current_pose_BEFORE", current_pose)

        current_pose = self.transform_pose(current_pose)
        #print("current_pose_AFTER", current_pose)
        pos_fixed = list(current_pose.p)
        pos_fixed[0] *= -1
        current_pose.p = Vector(*pos_fixed)
        #print("current_pose.m",current_pose.M)
        # current_pose.p 
        if self.init_pose is not None:
            ee_pose_goals_msg = EEPoseGoals()
            target = Frame()
            target.p = self.init_pose.p - self.latest_pose.p
            target.M = Rotation()
            #target.M = Rotation.Quaternion(*[-0.7071068, 0, 0, 0.7071068 ] ) #-90 x axis
            #target.M = Rotation.Quaternion(*[0, -0.7071068, 0, 0.7071068 ] ) #-90 y axis
            #target.M = Rotation.Quaternion(*[-0.6214175, 0.3374022, 0.3374022, 0.6214175] ) #-90 0 57 xyz axis
            
            #target.M = Rotation.Quaternion(*[-0.5, -0.5, 0.5, 0.5  ] ) #-90 x axis -90 y axis
            #target.M = Rotation.Quaternion(*[0, 0.7071068, 0 , 0.7071068  ] ) #-90 x axis 180 y axis
             
            # target.M = Rotation.Quaternion(*[ -0.6335811, -0.6335811, 0, 0.4440158 ]) #  -90x -90y
            #current_pose.M * Rotation.Quaternion()
            #print("current_pose.M",current_pose.M)
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