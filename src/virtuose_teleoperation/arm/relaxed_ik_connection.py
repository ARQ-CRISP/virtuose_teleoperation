from __future__ import print_function, division, absolute_import

from sensor_msgs.msg import JointState
from relaxed_ik.msg import EEPoseGoals, JointAngles
import rospy

import numpy as np



class IK_Solution_Manager:
    
    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
              'wrisensor_msgs.msgst_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    relaxed_ik_pose_goals_topic = '/relaxed_ik/ee_pose_goals'
    relaxed_ik_solutions_topic = '/relaxed_ik/joint_angle_solutions'
    
    joint_listener_topic = '/joint_states'

    def __init__(self, ):
        self.solution_listener = rospy.Subscriber(self.relaxed_ik_solutions_topic, JointAngles, self.__OnSolutionReceived, queue_size=1)
        self.joint_state_listener = rospy.Subscriber(self.joint_listener_topic, JointState, self.__OnJointStateReceived, queue_size=1)
        
        self.current_joint_state = np.zeros(6)
        
    def __OnJointStateReceived(self, joint_state_msg):
        """This method updates the current state of the robot.

        Args:
            joint_state_msg (sensor_msgs.msg.JointState): ros message containing the current robot joint state
        """
        rospy.loginfo("__OnJointStateReceived: " + str(joint_state_msg))
        
        
    def __OnSolutionReceived(self, ik_solution_msg):
        """
        ik_solution_msg: (relaxed_ik.msg.JointAngles): The JointAngles solution message defined by RelaxedIK
        """
        # np.asarray(ik_solution_msg.angles.data)
        
        # Here we define what happens every time RelaxedIK generates a solution
        rospy.loginfo("__OnSolutionReceived: " + str(ik_solution_msg))
        
        
    def joint_states_safety_check(self, joint_state):
        """This method checks if the target joint state is safe.
        This check is performed on the displacement between the target joint state and the current joint state.

        Args:
            joint_state (np.array): vector of the target joint states of the robot

        Returns:
            boolean: is target joint state safe?
        """
        
        return False    
    
    
    def goto(self, joint_state):
        """This method initiates the movement of the robot

        Args:
            joint_state (np.array): vector of the target joint states of the robot
        """
        
        
        # Here we send the message to the robot controller about the joint state destiation we want to go to.
        pass
        
        
        
    
    
    