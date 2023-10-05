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
    pose.p = Vector(msg.translation.x, msg.translation.y, msg.translation.z)
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



# def qv_mult(q1, v1):
#     # comment this out if v1 doesn't need to be a unit vector
#     v1 = tf.transformations.unit_vector(v1)
#     q2 = list(v1)
#     q2.append(0.0)
#     return tf.transformations.quaternion_multiply(
#         tf.transformations.quaternion_multiply(q1, q2), 
#         tf.transformations.quaternion_conjugate(q1)
#     )[:3]
def transform_to_vec(T):
    t = T.transform.translation
    r = T.transform.rotation
    return [[t.x, t.y, t.z], [r.x, r.y, r.z, r.w]]

class Cartesian_Mapping:

    def __init__(self, init_pose=None):
     
        self.virtuose_sub = rospy.Subscriber("/out_virtuose_physical_pose", out_virtuose_physical_pose, self.callback) #rosmsg show out_virtuose_physical_pose 
        self.ee_pose_ur5_sub = rospy.Subscriber("/cartesian_pose_UR5", PoseStamped, self.__OnUR5CartesianPoseReceived)

        self.ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)
        self.ee_pose_goals_pub_anti = rospy.Publisher('/ee_pose_goals_anti', EEPoseGoals, queue_size=5)

        self.ee_pose_ur5_antiRotation_pub=rospy.Publisher("/cartesian_pose_antiRotation_UR5", PoseStamped, queue_size=5)



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
        rot = [ 0.7071068, 0, 0, 0.7071068 ] # 90 deg on x axis required to match ur5  
        #MODIFY HERE TO REGULATE THE ANGULAR OFFSET BETWEEN UR5 AND VIRTUOSE
        """ to adjust the rot3 matrix, you need to use the green tablepad or another ortogonal reference
        forward->-z  #right->x #up->y
        you can read the cartesian position from:  rostopic echo /relaxed_ik/ee_pose_goals
         online 3D Rotation Converter: https://www.andre-gaschler.com/rotationconverter/ 
        """
        rot2 =[0, 0.2079117, 0, 0.9781476] #24 deg on y axis on 3D Rotation Converter depends on the orientation of Virtuose respect to the table.
        rot2 =[ 0, 0.7071068, 0, 0.7071068 ] #90degr on y for the frontal setup
        #antiROTATION [-0.7017234, -0.1871262, 0, 0.6874359]
        #Rotation.Quaternion: Constructs a rotation from an x, y, z, w quaternion descripion
        #Frame(rot, pos): Construct a frame from a rotation and a vector
        orient_bias = Frame(Rotation.Quaternion(*rot), Vector())
        orient_bias2 = Frame(Rotation.Quaternion(*rot2), Vector())######

        #Hamilton product H(a, b) https://math.stackexchange.com/questions/40164/how-do-you-rotate-a-vector-by-a-unit-quaternion
        new_pose = orient_bias * pose * orient_bias.Inverse()

        new_pose = orient_bias2 * new_pose * orient_bias2.Inverse() #white configuration
        #rot12=[0.6916548, 0.1470158, 0.1470158, 0.6916548] 90 24 0
        return new_pose

    def anti_transform_pose(self, pose):
        """method that implements the pose mapping from ur5 to haption frame.
            '/relaxed_ik/ee_pose_goals' and '/ee_pose_goals_anti' must be the same
        Args:
            pose (PyKDL.Frame): pose to adapt for the teleoperation

        Returns:
            PyKDL.Frame: Pose adapted for teleoperation
        """
        #ROTATIONS
        #anti_rot = [ -0.7017234, -0.1871262, 0, 0.6874359 ] # 90 deg on x axis required to match ur5
        anti_rot =  [         0, -0.2079117, 0, 0.9781476]
        anti_rot2 = [ -0.7071068, 0, 0, 0.7071068]

        orient_bias = Frame(Rotation.Quaternion(*anti_rot), Vector())
        orient_bias2 = Frame(Rotation.Quaternion(*anti_rot2), Vector())######

        #Hamilton product H(a, b) https://math.stackexchange.com/questions/40164/how-do-you-rotate-a-vector-by-a-unit-quaternion
        new_pose = orient_bias * pose * orient_bias.Inverse()
        new_pose = orient_bias2 * new_pose * orient_bias2.Inverse()
        return new_pose

    def callback(self, msg):
        # msg = out_virtuose_physical_pose()
        # rospy.loginfo('received Haption data')
        current_pose = fromTransform(msg.virtuose_physical_pose)
        current_rotation = fromTransform(msg.virtuose_physical_pose)
        # current_rotation.M=current_rotation.M*Rotation.Quaternion(1, 0, 0, 0)
        # print ("currentrotation",current_rotation)
        current_pose = self.transform_pose(current_pose)
        anti_pose=self.anti_transform_pose(current_pose)
        pos_fixed = list(current_pose.p)
        pos_fixed[0] *= -1
        current_pose.p = Vector(*pos_fixed)
        #####################
        pos_fixed_anti = list(anti_pose.p)
        pos_fixed_anti[0] *= -1
        anti_pose.p = Vector(*pos_fixed_anti)
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

            ######################
            ee_pose_anti_msg = EEPoseGoals()
            anti_target = Frame()
            anti_target.p = self.init_pose_anti.p - self.latest_pose_anti.p
            anti_target.M = Rotation() #current_pose.M
            ee_pose_anti_msg.ee_poses.append(pm.toMsg(target))
            self.ee_pose_goals_pub_anti.publish(ee_pose_anti_msg)
            #####################            
        else:
            self.init_pose = current_pose
            self.init_rotation= current_rotation
            # self.init_rotation.M=self.init_rotation.M*Rotation.Quaternion(0.7071068, 0, 0, 0.7071068) #current_pose.M


            self.init_pose_anti=anti_pose

        
        # print("initial _rotation",self.init_rotation)    
        self.latest_pose = current_pose
        self.latest_pose_anti = anti_pose
    
            

    def __OnUR5CartesianPoseReceived(self,msg):
        # msg = out_virtuose_physical_pose()
        # rospy.loginfo('received Haption data')
        #msgs=PoseStamped()

        current_pose = fromPoseStamped(msg)
        current_pose = self.anti_transform_pose(current_pose)

        current_pose_goals_msg = PoseStamped()

        current_pose_goals_msg.header.stamp = rospy.Time.now()
#            print(current_pose.p.x)
        current_pose_goals_msg.pose.position.x =current_pose.p.x()
        current_pose_goals_msg.pose.position.y = current_pose.p.y()
        current_pose_goals_msg.pose.position.z = current_pose.p.z()
        current_pose_goals_msg.pose.orientation.x = 0
        current_pose_goals_msg.pose.orientation.y = 0
        current_pose_goals_msg.pose.orientation.z = 0
        current_pose_goals_msg.pose.orientation.w = 1

        #p0_transform = self.tf_buffer.lookup_transform('world', 'hand_root', rospy.Time.now())
        #world_p0_list = [p0_transform.transform.translation.x, p0_transform.transform.translation.y, p0_transform.transform.translation.z] +\
        #    [p0_transform.transform.rotation.x, p0_transform.transform.rotation.y, p0_transform.transform.rotation.z, p0_transform.transform.rotation.w]
        #world_p0_list = np.asarray(world_p0_list)
        #print(world_p0_list)



        self.ee_pose_ur5_antiRotation_pub.publish(current_pose_goals_msg)




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