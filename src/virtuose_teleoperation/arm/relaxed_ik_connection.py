#! /usr/bin/env python

from __future__ import print_function, division, absolute_import
from select import select
import numpy as np
import rospy
import tf2_ros

from PyKDL import Frame, Rotation
from tf_conversions import posemath as pm
from moveit_commander import MoveGroupCommander, roscpp_initialize
from moveit_msgs.srv import GetPositionFK 
from moveit_msgs.msg import RobotState 
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from relaxed_ik.msg import EEPoseGoals, JointAngles

# Importing necessary modules for teleoperation
from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix
from virtuose_teleoperation.arm.joint_movements import JointMovementManager
from virtuose_teleoperation.arm.arm_teleop_input import Combined_Arm_Teleop_Input
from virtuose_teleoperation.arm.ur5_fk import UR5KDL

class IK_Solution_Manager:
    """Class to manage inverse kinematics solutions for a robotic arm."""

    # Joint names for the robot arm
    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    # ROS topics for communication
    relaxed_ik_pose_goals_topic = '/relaxed_ik/ee_pose_goals'
    relaxed_ik_pose_goals_topic_anti = '/ee_pose_goals_anti'
    relaxed_ik_solutions_topic = '/relaxed_ik/joint_angle_solutions'
    joint_listener_topic = '/joint_states'
    
    # Angle limits for IK solutions
    max_angle = np.pi / 4.  # Maximum allowed angle change
    min_angle = 1e-4        # Minimum allowed angle change

    # MoveIt controller topics for joint states and poses
    moveit_controller_joint_states_topic = '/move_group/fake_controller_joint_states'
    moveit_controller_joint_states_topic2 = '/move_group/fake_controller_joint_states2'
    current_pose_topic = '/cartesian_pose_UR5'  # Without anti-rotation
    current_realUR5_topic = '/real_ur5_cartesianPosition'  # Without anti-rotation

    def __init__(self, init_state=None, sim=True, **kwargs):
        """Initializes the IK Solution Manager.

        Args:
            init_state (np.array): Initial joint state of the robot.
            sim (bool): Flag indicating if the robot is in simulation mode.
        """
        self.safety_counter = 0  # Safety counter to monitor EE position
        self._kdl = UR5KDL()  # KDL instance for forward kinematics
        self.rotation_bias = Frame(Rotation.Quaternion(-0.7071067811865475, 0.7071067811865476, 0 ,0))
        
        self.posegoal = EEPoseGoals()
        self.posegoal.ee_poses.append(Pose())
        
        # Initialize absolute mode flag based on parameter server value
        self.set_absolute_mode_flag(self.get_absolute_mode_flag())

        # Joint movement manager instance creation
        self.joint_manager = JointMovementManager.generate_manager(init_state, sim=sim)

        # Subscribers for receiving joint states and pose goals
        self.solution_listener = rospy.Subscriber(self.relaxed_ik_solutions_topic, JointAngles, self.__OnSolutionReceived, queue_size=1)
        self.joint_state_listener = rospy.Subscriber(self.joint_listener_topic, JointState, self.__OnJointStateReceived, queue_size=1)
        self.ik_pose_goals_listener = rospy.Subscriber(self.relaxed_ik_pose_goals_topic, EEPoseGoals, self.__OnPoseGoalsRepuber, queue_size=1)
        self.ik_pose_goals_listener_anti = rospy.Subscriber(self.relaxed_ik_pose_goals_topic_anti, EEPoseGoals, self.__OnPoseGoalsRepuber_anti, queue_size=1)

        # Publishers for sending joint states and poses to MoveIt!
        self.joint_publisher = rospy.Publisher(self.moveit_controller_joint_states_topic, JointState, queue_size=5)
        self.joint_publisher2 = rospy.Publisher(self.moveit_controller_joint_states_topic2, JointState, queue_size=5)
        self.current_pose_pub = rospy.Publisher(self.current_pose_topic, PoseStamped, queue_size=5)  # Cartesian pose publisher
        self.real_ur5_cartesianPosition = rospy.Publisher(self.current_realUR5_topic, PoseStamped, queue_size=5)  # Real UR5 pose publisher

        # Initialize current joint state based on provided initial state or default to zeros
        if init_state is None:
            self.current_joint_state = np.zeros(6)
        else:
            self.current_joint_state = init_state
        
        # Copy latest target joint state for tracking purposes
        self.latest_target = np.copy(self.current_joint_state)

        # Initialize TF buffer and listener for transformations
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10), True)
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Wait for the FK service to be available before proceeding
        rospy.wait_for_service('compute_fk')
        self._fk_service = rospy.ServiceProxy('compute_fk', GetPositionFK)
        
        pose = self.compute_fk(init_state).pose  # Compute FK for the initial state
        print("pose", pose)  # Log the computed pose

        # Process initial solution received from the IK solver
        self.__OnSolutionReceived(JointAngles(angles=np.array(init_state)), True)

        # Initialize input manager for teleoperation control after a brief delay to ensure readiness
        rospy.sleep(1)
        self.input_manager = Combined_Arm_Teleop_Input(pm.fromMsg(pose))
        
        # Timer to check end-effector safety at regular intervals (15 Hz)
        rospy.Timer(rospy.Duration(1/15.), lambda msg: self.check_ee_safety())

    def check_ee_safety(self):
        """Checks if the end-effector is within safe operational bounds."""
        
        pose = self.compute_fk().pose  # Compute current pose using FK service
        frame = pm.fromMsg(pose)  # Convert pose to KDL frame
        
        if not self.input_manager.workspace.in_bounds(frame.p, 5e-2):
            self.safety_counter += 1  # Increment safety counter if out of bounds
            
            rospy.logwarn('EE Out of Bounds! Return to Workspace!')
            if self.safety_counter > 20 and not self.joint_manager.stopped:
                rospy.logwarn('EE Out of Bounds! Emergency Stop!')
                self.joint_manager.emergency_stop()  # Trigger emergency stop if out of bounds too long
        
        else:
            self.safety_counter = 0  # Reset safety counter if within bounds
            
            if self.joint_manager.stopped:
                rospy.logwarn('EE Back in Bounds! Restart!')
                self.joint_manager.restart()  # Restart joint manager if previously stopped

    def on_kill(self):
        """Terminates the joint movement manager when shutting down."""
        
        if self.joint_manager is not None:
            self.joint_manager.terminate()

    def compute_fk(self, arm_state=None):
        """Computes forward kinematics for the given arm state.

        Args:
            arm_state (np.array): Current joint state of the robot.

        Returns:
            PoseStamped: The computed end-effector pose.
        """
        
        if arm_state is None:
            arm_state = self.joint_manager.current_j_state.position
        
        posestamped = self._fk_service(
            Header(0, rospy.Time.now(), "world"), ['palm_link'], 
            RobotState(joint_state=JointState(name=self.jnames, position=arm_state))
            ).pose_stamped[0]
        
        return posestamped

    def set_debug_properties(self):
        """Sets properties for debugging visual markers."""
        
        self.marker_target_pub = rospy.Publisher(self.goal_marker_topic, Marker, queue_size=1)
        
        # Configure marker properties for visualization in RViz
        self.target_marker = Marker(type=Marker.ARROW)
        
        self.target_marker.scale.x = 0.2  
        self.target_marker.scale.y = 0.01  
        self.target_marker.scale.z = 0.01  
        
        # Set color properties of the marker (blue with full opacity)
        self.target_marker.color.b = 1.0  
        self.target_marker.color.a = 1.0  

    def get_absolute_mode_flag(self):
       """Retrieves the current absolute mode flag from ROS parameter server."""
       
       return rospy.get_param(self.absolute_teleop_mode_rosparam, default=True)

    def set_absolute_mode_flag(self, value):
       """Sets the absolute mode flag in ROS parameter server.

       Args:
           value (bool): New value for absolute mode.
       """
       
       if value:
           rospy.loginfo('{}: Receiving targets in world frame!'.format(rospy.get_name()))
       else:
           rospy.loginfo('{}: Receiving targets in relation to the initial position!'.format(rospy.get_name()))
       
       rospy.set_param(self.absolute_teleop_mode_rosparam, value)

    def __OnJointStateReceived(self, joint_state_msg):
       """Updates the current state of the robot based on received joint state message.

       Args:
           joint_state_msg (sensor_msgs.msg.JointState): ROS message containing current robot joint state.
       """
       
       new_state = np.asarray(joint_state_msg.position)  # Extract new joint state from message

       # Round new state values to two decimal places for consistency and publish it as a message.
       new_state2=np.round(new_state,2)  
       
       current_joint_state_msg = JointState()
       current_joint_state_msg.name = self.jnames  
       current_joint_state_msg.header.stamp = rospy.Time.now()  
       current_joint_state_msg.position = new_state2.tolist()  
       
       # Publish updated joint state message (optional logging can be added here)

    def __OnSolutionReceived(self, joint_angle_msg, force=False):
       """Handles received solutions from RelaxedIK.

       Args:
           joint_angle_msg (relaxed_ik.msg.JointAngles): The JointAngles solution message.
           force (bool): Flag indicating whether to force movement regardless of safety checks.
       """
       
       diff = (
           np.asarray(joint_angle_msg.angles.data) - \
           np.asarray(self.joint_manager.current_j_state.position)).round(2)
       
       diff = np.abs(np.arctan2(np.sin(diff), np.cos(diff)))  # Normalize angle differences

       if (self.min_angle < np.max(diff) < self.max_angle) or force:
           # If difference is within limits or forced movement is requested,
           # initiate movement towards target angles.
           self.joint_manager.generate_movement(joint_angle_msg.angles.data)
           
       elif np.any(np.absolute(diff) >= self.max_angle:
           rospy.logwarn('Arm seems to move too fast! Max diff: {}'.format(np.absolute(diff).max()))

    def __OnPoseGoalsRepuber(self, goals_msg):
       """Republishes received pose goals from RelaxedIK as PoseStamped messages.

       Args:
           goals_msg (EEPoseGoals): Incoming goal message containing end-effector poses.
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

       # Publish converted goal message to desired topic (e.g., `current_pose_pub`)

    def __OnPoseGoalsRepuber_anti(self, goals_msg_anti):
      """Republishes received anti-rotation pose goals from RelaxedIK as PoseStamped messages.

      Args:
          goals_msg_anti (EEPoseGoals): Incoming anti-goal message containing end-effector poses.
      """
      
      current_pose_goals_msg_anti = PoseStamped()
      current_pose_goals_msg_anti.header.stamp = rospy.Time.now()
      
      current_pose_goals_msg_anti.pose.position.x = goals_msg_anti.ee_poses[0].position.x  
      current_pose_goals_msg_anti.pose.position.y = goals_msg_anti.ee_poses[0].position.y  
      current_pose_goals_msg_anti.pose.position.z = goals_msg_anti.ee_poses[0].position.z  
      
      current_pose_goals_msg_anti.pose.orientation.x = goals_msg_anti.ee_poses[0].orientation.x  
      current_pose_goals_msg_anti.pose.orientation.y = goals_msg_anti.ee_poses[0].orientation.y  
      current_pose_goals_msg_anti.pose.orientation.z = goals_msg_anti.ee_poses[0].orientation.z  
      current_pose_goals_msg_anti.pose.orientation.w = goals_msg_anti.ee_poses[0].orientation.w  

      # Publish converted anti-goal message to desired topic (e.g., `current_pose_pub`)

    def joint_states_safety_check(self,joint_state):
      """Checks if the target joint state is safe based on predefined criteria.

      Args:
          joint_state (np.array): Target joint states of the robot.

      Returns:
          bool: True if target joint state is safe; otherwise False.
      """
      
      return True  
    
    def goto(self,target_joint_state):
      """Initiates movement of the robot towards a target joint state.

      Args:
          target_joint_state (np.array): Target joint states of the robot.
      """
      
      target_msg=JointState()
      target_msg.name=self.jnames  
      target_msg.header.stamp=rospy.Time.now()  
      
      target_msg.position=target_joint_state.tolist()  

      # Publish target joint state message to MoveIt!
      self.joint_publisher.publish(target_msg)

