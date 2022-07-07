#! /usr/bin/env python

from __future__ import print_function, division, absolute_import
from tokenize import String
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from relaxed_ik.msg import EEPoseGoals, JointAngles
import rospy

import numpy as np



class IK_Solution_Manager:
    
    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
              'wrisensor_msgs.msgst_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    relaxed_ik_pose_goals_topic = '/relaxed_ik/ee_pose_goals'
    relaxed_ik_solutions_topic = '/relaxed_ik/joint_angle_solutions'
    
    joint_listener_topic = '/joint_states'
    
    moveit_controller_joint_states_topic = '/move_group/fake_controller_joint_states'
    moveit_controller_joint_states_topic2 = '/move_group/fake_controller_joint_states2'

    current_pose_topic='/cartesian_pose_UR5' ########

    def __init__(self, init_state=None):

        self.solution_listener = rospy.Subscriber(self.relaxed_ik_solutions_topic, JointAngles, self.__OnSolutionReceived, queue_size=1)
        self.ik_pose_goals_listener = rospy.Subscriber(self.relaxed_ik_pose_goals_topic, EEPoseGoals, self.__OnPoseGoalsRepuber,queue_size=1)

        self.joint_state_listener = rospy.Subscriber(self.joint_listener_topic, JointState, self.__OnJointStateReceived, queue_size=1)
        
        
        self.joint_publisher = rospy.Publisher(self.moveit_controller_joint_states_topic, JointState,queue_size=5)
        self.joint_publisher2 = rospy.Publisher(self.moveit_controller_joint_states_topic2, JointState,queue_size=5)

        self.current_pose_pub = rospy.Publisher(self.current_pose_topic, PoseStamped, queue_size=5)########


        if init_state is None:
            self.current_joint_state = np.zeros(6)
        else:
            self.current_joint_state = init_state
        self.latest_target = np.copy(self.current_joint_state)
        


    def __OnJointStateReceived(self, joint_state_msg):
        """This method updates the current state of the robot.

        Args:
            joint_state_msg (sensor_msgs.msg.JointState): ros message containing the current robot joint state
        """
        # joint_state_msg = JointState()
        new_state = np.asarray(joint_state_msg.position)
        # new_vel = np.asarray(joint_state_msg.velocity)
        # new_eff = np.asarray(joint_state_msg.effort)
        self.current_joint_state = new_state
        new_state2=np.round(new_state,2)
        
        # JOINT MSG published on '/move_group/fake_controller_joint_states'
        current_joint_state_msg = JointState()
        current_joint_state_msg.name = self.jnames
        current_joint_state_msg.header.stamp = rospy.Time.now()
        #TODO: target_msg.header.frame_id = '' 
        current_joint_state_msg.position = new_state2.tolist()
        self.joint_publisher2.publish(current_joint_state_msg)


        # rospy.loginfo("__OnSolutionReceived: " + str(new_state))

        #rospy.loginfo("__OnJointStateReceived: ") #+ str(new_state))
        
        
    def __OnSolutionReceived(self, ik_solution_msg):
        """
        ik_solution_msg: (relaxed_ik.msg.JointAngles): The JointAngles solution message defined by RelaxedIK
        """
        # np.asarray(ik_solution_msg.angles.data)
        
        # get target position data
        target_joint_state = np.asarray(ik_solution_msg.angles.data)
        # Here we define what happens every time RelaxedIK generates a solution
        # rospy.loginfo("__OnSolutionReceived: " + str(target_joint_state))
        if self.joint_states_safety_check(target_joint_state):
            self.goto(target_joint_state)
            self.latest_target = target_joint_state
        else:
            rospy.logwarn('Received unsafe solution! waiting for next one...')

    def __OnPoseGoalsRepuber(self,goals_msg):
        """
        This convert the RelaxedIk goal msg from EEPoseGoals to PoseStamped
        """
        current_pose_goals_msg = PoseStamped()
        current_pose_goals_msg.header.stamp = rospy.Time.now()
        current_pose_goals_msg.pose.position.x = goals_msg.ee_poses[0].position.x
        current_pose_goals_msg.pose.position.y = goals_msg.ee_poses[0].position.y
        current_pose_goals_msg.pose.position.z = goals_msg.ee_poses[0].position.z
        current_pose_goals_msg.pose.orientation.x = goals_msg.ee_poses[0].orientation.x
        current_pose_goals_msg.pose.orientation.y = goals_msg.ee_poses[0].orientation.y
        current_pose_goals_msg.pose.orientation.z = goals_msg.ee_poses[0].orientation.z
        current_pose_goals_msg.pose.orientation.w = goals_msg.ee_poses[0].orientation.w
        self.current_pose_pub.publish(current_pose_goals_msg)

    def joint_states_safety_check(self, joint_state):
        """This method checks if the target joint state is safe.
        This check is performed on the displacement between the target joint state and the current joint state.

        Args:
            joint_state (np.array): vector of the target joint states of the robot

        Returns:
            boolean: is target joint state safe?
        """
        
        return True    
    
    
    def goto(self, target_joint_state):
        """This method initiates the movement of the robot

        Args:
            joint_state (np.array): vector of the target joint states of the robot
        """        
        # Here we send the message to the robot controller about the joint state destiation we want to go to.
        
        target_msg = JointState()
        target_msg.name = self.jnames
        target_msg.header.stamp = rospy.Time.now()
        #TODO: target_msg.header.frame_id = '' 
        target_msg.position = target_joint_state.tolist()
        
        #rospy.loginfo("going to: " + str(target_joint_state.round(3)))
        self.joint_publisher.publish(target_msg)
        
        
        
        
    
    
    
        """
        [relaxed_ik/EEPoseGoals]:
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

        """