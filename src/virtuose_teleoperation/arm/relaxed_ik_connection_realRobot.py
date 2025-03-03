#!/usr/bin/env python

"""
IK Solution Manager for UR5 Robot Arm

This module manages inverse kinematics solutions for a UR5 robot arm,
integrating with ROS, MoveIt, and RelaxedIK.

Author: [Gabriele Giudici]
"""

from __future__ import print_function, division, absolute_import
import numpy as np
import rospy
import tf2_ros

from PyKDL import Frame, Rotation
from tf_conversions import posemath as pm

from moveit_msgs.srv import GetPositionFK 
from moveit_msgs.msg import RobotState 

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64, Header
from visualization_msgs.msg import Marker
from relaxed_ik.msg import EEPoseGoals, JointAngles

from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix
from virtuose_teleoperation.arm.joint_movements import JointMovementManager
from virtuose_teleoperation.arm.arm_teleop_input import Combined_Arm_Teleop_Input
from virtuose_teleoperation.arm.ur5_fk import UR5KDL

class IK_Solution_Manager:
    """
    Manages inverse kinematics solutions for UR5 robot arm.
    """
    
    # Joint names for UR5 robot
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    # ROS topic names
    RELAXED_IK_POSE_GOALS_TOPIC = '/relaxed_ik/ee_pose_goals'
    RELAXED_IK_POSE_GOALS_TOPIC_ANTI = '/ee_pose_goals_anti'
    RELAXED_IK_SOLUTIONS_TOPIC = '/relaxed_ik/joint_angle_solutions'
    JOINT_LISTENER_TOPIC = '/joint_states'
    MOVEIT_CONTROLLER_JOINT_STATES_TOPIC = '/move_group/fake_controller_joint_states'
    MOVEIT_CONTROLLER_JOINT_STATES_TOPIC2 = '/move_group/fake_controller_joint_states2'
    CURRENT_POSE_TOPIC = '/cartesian_pose_UR5'
    
    # ROS parameter names
    ABSOLUTE_TELEOP_MODE_ROSPARAM = ur5_teleop_prefix + 'teleop_mode_absolute'
    
    # Constants
    MAX_ANGLE = np.pi/4.
    MIN_ANGLE = 1e-4

    def __init__(self, init_state=None, sim=True):
        """
        Initialize the IK Solution Manager.

        Args:
            init_state (list): Initial joint state of the robot.
            sim (bool): Whether running in simulation mode.
        """
        self.safety_counter = 0
        self._kdl = UR5KDL()
        self.rotation_bias = Frame(Rotation.Quaternion(-0.7071067811865475, 0.7071067811865476, 0, 0))
        
        self.posegoal = EEPoseGoals()
        self.posegoal.ee_poses.append(Pose())
        self.set_absolute_mode_flag(self.get_absolute_mode_flag())

        self.joint_manager = JointMovementManager.generate_manager(init_state, sim=sim)

        self._setup_ros_communication()
        self._initialize_state(init_state)
        self._setup_services()
        self._initialize_input_manager()

    def _setup_ros_communication(self):
        """Set up ROS subscribers and publishers."""
        self.solution_listener = rospy.Subscriber(self.RELAXED_IK_SOLUTIONS_TOPIC, JointAngles, self._on_solution_received, queue_size=1)
        self.joint_state_listener = rospy.Subscriber(self.JOINT_LISTENER_TOPIC, JointState, self._on_joint_state_received, queue_size=1)
        self.ik_pose_goals_listener = rospy.Subscriber(self.RELAXED_IK_POSE_GOALS_TOPIC, EEPoseGoals, self._on_pose_goals_repuber, queue_size=1)
        self.ik_pose_goals_listener_anti = rospy.Subscriber(self.RELAXED_IK_POSE_GOALS_TOPIC_ANTI, EEPoseGoals, self._on_pose_goals_repuber_anti, queue_size=1)

        self.joint_publisher = rospy.Publisher(self.MOVEIT_CONTROLLER_JOINT_STATES_TOPIC, JointState, queue_size=5)
        self.joint_publisher2 = rospy.Publisher(self.MOVEIT_CONTROLLER_JOINT_STATES_TOPIC2, JointState, queue_size=5)
        self.current_pose_pub = rospy.Publisher(self.CURRENT_POSE_TOPIC, PoseStamped, queue_size=5)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10), True)
        tf2_ros.TransformListener(self.tf_buffer)

    def _initialize_state(self, init_state):
        """Initialize the current joint state."""
        self.current_joint_state = np.zeros(6) if init_state is None else init_state
        self.latest_target = np.copy(self.current_joint_state)

    def _setup_services(self):
        """Set up ROS services."""
        rospy.wait_for_service('compute_fk')
        self._fk_service = rospy.ServiceProxy('compute_fk', GetPositionFK)
        
        pose = self.compute_fk(self.current_joint_state).pose
        rospy.loginfo(f"Initial pose: {pose}")

    def _initialize_input_manager(self):
        """Initialize the input manager."""
        rospy.sleep(1)
        pose = self.compute_fk(self.current_joint_state).pose
        self.input_manager = Combined_Arm_Teleop_Input(pm.fromMsg(pose))
        
        rospy.Timer(rospy.Duration(1/15.), lambda msg: self.check_ee_safety())

    def check_ee_safety(self):
        """Check if the end effector is within safe bounds."""
        pose = self.compute_fk().pose
        frame = pm.fromMsg(pose)
        
        if not self.input_manager.workspace.in_bounds(frame.p, 5e-2):
            self.safety_counter += 1
            rospy.logwarn('EE Out of Bounds! Return to Workspace!')
            if self.safety_counter > 20 and not self.joint_manager.stopped:
                rospy.logwarn('EE Out of Bounds! Emergency Stop!')
                self.joint_manager.emergency_stop()
        else:
            self.safety_counter = 0
            if self.joint_manager.stopped:
                rospy.logwarn('EE Back in Bounds! Restart!')
                self.joint_manager.restart()
            
    def on_kill(self):
        """Clean up resources on node shutdown."""
        if self.joint_manager is not None:
            self.joint_manager.terminate()

    def compute_fk(self, arm_state=None):
        """
        Compute forward kinematics for the given arm state.

        Args:
            arm_state (list): Joint angles for the arm. If None, use current state.

        Returns:
            PoseStamped: The computed end-effector pose.
        """
        if arm_state is None:
            arm_state = self.joint_manager.current_j_state.position
        posestamped = self._fk_service(
            Header(0, rospy.Time.now(), "world"), ['palm_link'], 
            RobotState(joint_state=JointState(name=self.JOINT_NAMES, position=arm_state))).pose_stamped[0]
        return posestamped

    def get_absolute_mode_flag(self):
        """Get the absolute teleop mode flag from ROS parameter server."""
        return rospy.get_param(self.ABSOLUTE_TELEOP_MODE_ROSPARAM, default=True)
    
    def set_absolute_mode_flag(self, value):
        """Set the absolute teleop mode flag in ROS parameter server."""
        mode = "world frame" if value else "relative to the initial position"
        rospy.loginfo(f'{rospy.get_name()}: Receiving targets in {mode}!')
        rospy.set_param(self.ABSOLUTE_TELEOP_MODE_ROSPARAM, value)

    def _on_joint_state_received(self, joint_state_msg):
        """
        Update the current state of the robot.

        Args:
            joint_state_msg (JointState): ROS message containing the current robot joint state.
        """
        self.current_joint_state = np.asarray(joint_state_msg.position)
        new_state2 = np.round(self.current_joint_state, 2)
        
        current_joint_state_msg = JointState()
        current_joint_state_msg.name = self.JOINT_NAMES
        current_joint_state_msg.header.stamp = rospy.Time.now()
        current_joint_state_msg.position = new_state2.tolist()
        self.joint_publisher2.publish(current_joint_state_msg)

    def _on_solution_received(self, ik_solution_msg, force=False):
        """
        Process received IK solution.

        Args:
            ik_solution_msg (JointAngles): The JointAngles solution message defined by RelaxedIK.
            force (bool): Force the movement regardless of safety checks.
        """
        diff = (np.asarray(ik_solution_msg.angles.data) - np.asarray(self.joint_manager.current_j_state.position)).round(2)
        diff = np.abs(np.arctan2(np.sin(diff), np.cos(diff)))
        if (self.MIN_ANGLE < np.max(diff) < self.MAX_ANGLE) or force:
            self.joint_manager.generate_movement(ik_solution_msg.angles.data)
        elif np.any(np.absolute(diff) >= self.MAX_ANGLE):
            rospy.logwarn(f'Arm seems to move too fast! max diff: {np.absolute(diff).max()}')

        self._update_current_pose()

    def _update_current_pose(self):
        """Update and publish the current pose of the robot."""
        try:
            p0_transform = self.tf_buffer.lookup_transform('world', 'hand_root', rospy.Time.now(), rospy.Duration(0.1))
            current_pose_goals_msg = PoseStamped()
            current_pose_goals_msg.header.stamp = rospy.Time.now()
            current_pose_goals_msg.pose.position = p0_transform.transform.translation
            current_pose_goals_msg.pose.orientation = p0_transform.transform.rotation
            self.current_pose_pub.publish(current_pose_goals_msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF lookup failed: {e}")

    def _on_pose_goals_repuber(self, goals_msg):
        """
        Convert the RelaxedIK goal msg from EEPoseGoals to PoseStamped and publish.

        Args:
            goals_msg (EEPoseGoals): The received pose goals message.
        """
        current_pose_goals_msg = PoseStamped()
        current_pose_goals_msg.header.stamp = rospy.Time.now()
        current_pose_goals_msg.pose = goals_msg.ee_poses[0]
        self.current_pose_pub.publish(current_pose_goals_msg)

    def _on_pose_goals_repuber_anti(self, goals_msg_anti):
        """
        Process anti-rotated pose goals (currently not used).

        Args:
            goals_msg_anti (EEPoseGoals): The received anti-rotated pose goals message.
        """
        # This method is currently not used, but kept for potential future use
        pass

if __name__ == "__main__":
    rospy.init_node('ik_solution_manager')
    manager = IK_Solution_Manager()
    rospy.spin()
