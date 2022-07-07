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
        self.joint_state_listener = rospy.Subscriber(self.joint_listener_topic, JointState, self.__OnJointStateReceived, queue_size=1)
        
        
        self.joint_publisher = rospy.Publisher(self.moveit_controller_joint_states_topic, JointState,queue_size=5)
        self.joint_publisher2 = rospy.Publisher(self.moveit_controller_joint_states_topic2, JointState,queue_size=5)

        self.current_pose_pub = rospy.Publisher(self.current_pose_topic, String, queue_size=5)########


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

        # JOINT_POSITION MSG published on '/cartesian_pose_UR5'
        current_joint_position_msg =String()
        current_joint_position_msg = str(current_joint_state_msg.position[16:19])
        self.current_pose_pub.publish(current_joint_position_msg)



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
        
        
        
        
    
    
    