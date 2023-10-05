#! /usr/bin/env python

from __future__ import print_function, division, absolute_import
from select import select
from tokenize import String
import numpy as np
import rospy
import tf2_ros

from PyKDL import Frame, Rotation, Vector
from tf_conversions import posemath as pm
from tf_conversions import fromTf, toMsg

from moveit_commander import MoveGroupCommander, roscpp_initialize
from moveit_commander.conversions import list_to_pose, pose_to_list
from moveit_msgs.srv import GetPositionFK 
from moveit_msgs.msg import RobotState 

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Float64, Header
from visualization_msgs.msg import Marker
from relaxed_ik.msg import EEPoseGoals, JointAngles

#from teleoperation_ur5_allegro_leap.teleop.ur5.joint_movements import JointMovementManager
#from teleoperation_ur5_allegro_leap.teleop.ur5.ur5_fk import UR5KDL
from teleoperation_ur5_allegro_leap.teleop.ur5 import ur5_teleop_prefix
from virtuose_teleoperation.arm.joint_movements import JointMovementManager
from virtuose_teleoperation.arm.arm_teleop_input import Combined_Arm_Teleop_Input
from virtuose_teleoperation.arm.ur5_fk import UR5KDL



class IK_Solution_Manager:
    
    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    relaxed_ik_pose_goals_topic = '/relaxed_ik/ee_pose_goals'
    relaxed_ik_pose_goals_topic_anti='/ee_pose_goals_anti'
    
    relaxed_ik_solutions_topic = '/relaxed_ik/joint_angle_solutions'
    
    absolute_teleop_mode_rosparam = ur5_teleop_prefix + 'teleop_mode_absolute'

    joint_listener_topic = '/joint_states'
    
    max_angle = np.pi/4.
    min_angle = 1e-4

    moveit_controller_joint_states_topic = '/move_group/fake_controller_joint_states'
    moveit_controller_joint_states_topic2 = '/move_group/fake_controller_joint_states2'

    current_pose_topic='/cartesian_pose_UR5' ######## without anti rotation
    current_realUR5_topic='/real_ur5_cartesianPosition' ######## without anti rotation

    ##  out_virtuose_physical_pose corresponds in ur5 anti  (-xV=-xUR5; +yV=+zUR5; +zV=+yUR5)

    ##current_pose_topic_anti='/cartesian_pose_UR5_anti' ####THIS IS THE ONE WITH ANTI ROTATION TO BE COMPATIBLE WITH THE VIRTUOSE AXIS
    


    def __init__(self, init_state=None, sim=True, **kwargs):
# ####    #SAFETY BLOCKS
        self.safety_counter = 0
        self._kdl = UR5KDL()
        self.rotation_bias = Frame(Rotation.Quaternion(-0.7071067811865475, 0.7071067811865476, 0 ,0)) ########
        
        #self.rotation_bias = Frame(Rotation.Quaternion(0.6916548, 0.1470158, 0.1470158, 0.6916548)) #deg: 90x,24y, 0z
        self.posegoal = EEPoseGoals()
        self.posegoal.ee_poses.append(Pose())
        self.set_absolute_mode_flag(self.get_absolute_mode_flag()) # sets to default value
# #####

        self.joint_manager = JointMovementManager.generate_manager(init_state, sim=sim)

        self.solution_listener = rospy.Subscriber(self.relaxed_ik_solutions_topic, JointAngles, self.__OnSolutionReceived, queue_size=1)
        self.joint_state_listener = rospy.Subscriber(self.joint_listener_topic, JointState, self.__OnJointStateReceived, queue_size=1)
        self.ik_pose_goals_listener = rospy.Subscriber(self.relaxed_ik_pose_goals_topic, EEPoseGoals, self.__OnPoseGoalsRepuber,queue_size=1)
        self.ik_pose_goals_listener_anti = rospy.Subscriber(self.relaxed_ik_pose_goals_topic_anti, EEPoseGoals, self.__OnPoseGoalsRepuber_anti,queue_size=1)

        self.joint_publisher = rospy.Publisher(self.moveit_controller_joint_states_topic, JointState,queue_size=5)
        self.joint_publisher2 = rospy.Publisher(self.moveit_controller_joint_states_topic2, JointState,queue_size=5)
        self.current_pose_pub = rospy.Publisher(self.current_pose_topic, PoseStamped, queue_size=5) #CARTESIAN 
        self.real_ur5_cartesianPosition = rospy.Publisher(self.current_realUR5_topic, PoseStamped, queue_size=5) #CARTESIAN 

        #self.current_pose_pub_anti = rospy.Publisher(self.current_pose_topic_anti, PoseStamped, queue_size=5) #CARTESIAN 
        ##self.current_pose_tf_pub_anti = rospy.Publisher(self.current_pose_topic_anti, PoseStamped, queue_size=5) #CARTESIAN 

        if init_state is None:
            self.current_joint_state = np.zeros(6)
        else:
            self.current_joint_state = init_state
        self.latest_target = np.copy(self.current_joint_state)
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10), True)
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

#####
        rospy.wait_for_service('compute_fk')
        self._fk_service = rospy.ServiceProxy('compute_fk', GetPositionFK)
        
        pose = self.compute_fk(init_state).pose
        print("pose", pose)
#####

        self.__OnSolutionReceived(
            JointAngles(angles=Float64(init_state)), True)

#####
        #Input manager Initialised here
        rospy.sleep(1)
        self.input_manager = Combined_Arm_Teleop_Input(pm.fromMsg(pose))
        # self.init_pose = pm.fromMsg(pose)
        
        rospy.Timer(rospy.Duration(1/15.), lambda msg: self.check_ee_safety())
        
#####
####
    def check_ee_safety(self):
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
        if self.joint_manager is not None:
            self.joint_manager.terminate()

    def compute_fk(self, arm_state=None):
        if arm_state is None:
            arm_state = self.joint_manager.current_j_state.position
        posestamped = self._fk_service(
            Header(0,rospy.Time.now(), "world"), ['palm_link'], 
            RobotState(joint_state=JointState(name=self.jnames, position=arm_state))).pose_stamped[0]
        return posestamped

    def set_debug_properties(self):
        self.marker_target_pub = rospy.Publisher(self.goal_marker_topic, Marker, queue_size=1)
        self.target_marker = Marker(type=Marker.ARROW)
        # self.target_marker.type = Marker.ARROW
        self.target_marker.scale.x, self.target_marker.scale.y, self.target_marker.scale.z = 0.2, 0.01, 0.01
        self.target_marker.color.b = self.target_marker.color.a = 1.0
    
    def get_absolute_mode_flag(self):
        return rospy.get_param(self.absolute_teleop_mode_rosparam, default=True)
    
    def set_absolute_mode_flag(self, value):
        if value==True:
            rospy.loginfo('{}: Receiving targets in world frame!'.format(rospy.get_name()))
        else:
            rospy.loginfo('{}: Receiving targets in in retation to the initial position!'.format(rospy.get_name()))
        rospy.set_param(self.absolute_teleop_mode_rosparam, value)
####

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
        # self.joint_publisher2.publish(current_joint_state_msg)


        #rospy.loginfo("__OnSolutionReceived: " + str(new_state))
        #rospy.loginfo("__OnJointStateReceived: ") #+ str(new_state))
        
    def __OnSolutionReceived(self, joint_angle_msg, force=False):
        """Function activated once the relaxedIK module has a ready solution

        Args:
            joint_angle_msg (relaxed_ik.msg.JointAngles): The JointAngles solution message
        """
        #print('(Virtuosejoint_angle_msg.angles.data)',np.asarray(joint_angle_msg.angles.data))
        #[-3.51998331 -2.35621324 -1.85322204 -0.90234304  1.71986696  1.00960177]
        #print('(REAL ROBOT joint_manager.current_j_state.position)',np.asarray(self.joint_manager.current_j_state.position))
        #[ 2.75358438 -2.58036501 -1.94370062 -0.02060968  1.69204485 -0.06718046]
        diff = (
            np.asarray(joint_angle_msg.angles.data) - \
                np.asarray(self.joint_manager.current_j_state.position)).round(2)
        diff = np.abs(np.arctan2(np.sin(diff), np.cos(diff)))
        if (self.min_angle < np.max(diff) < self.max_angle) or force:
            self.joint_manager.generate_movement(joint_angle_msg.angles.data)
            
        elif np.any(np.absolute(diff) >= self.max_angle):
            rospy.logwarn('Arm seems to move too fast! max diff: {}'.format(np.absolute(diff).max()))
            # rospy.logwarn('max angles {}'.format(self.max_angle))
            # rospy.logwarn('angles {}'.format(np.asarray(joint_angle_msg.angles.data)))
            # rospy.logwarn('current_j_state {}'.format(np.asarray(self.joint_manager.current_j_state.position)))
        
        # cartesian_pose = self.compute_fk(joint_angle_msg.angles.data)
        # self.real_ur5_cartesianPosition.publish(cartesian_pose)

        ## These lines are needed only to update the marker of the EE on Rviz
        # success, ee_pose = self._kdl.solve(joint_angle_msg.angles.data)
        # marker = self._kdl.generate_pose_marker(ee_pose)
        # self.marker_target_pub.publish(marker)


    # def __OnSolutionReceived(self, ik_solution_msg):
    #     """
    #     ik_solution_msg: (relaxed_ik.msg.JointAngles): The JointAngles solution message defined by RelaxedIK
    #     """
    #     # np.asarray(ik_solution_msg.angles.data)

    #     # get target position data
    #     target_joint_state = np.asarray(ik_solution_msg.angles.data)
    #     # Here we define what happens every time RelaxedIK generates a solution
    #     # rospy.loginfo("__OnSolutionReceived: " + str(target_joint_state))
    #     if self.joint_states_safety_check(target_joint_state):
    #         self.goto(target_joint_state)
    #         self.latest_target = target_joint_state
            
    #     else:
    #         rospy.logwarn('Received unsafe solution! waiting for next one...')
    #    # 

    #     #joint_target_publisher = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1) 
    #     #p0_transform =self.tf_buffer.lookup_transform_full('world',rospy.Time.now(),'hand_root',rospy.Time.now(),'world',rospy.Duration(1))
    #     p0_transform = self.tf_buffer.lookup_transform('world', 'hand_root', rospy.Time.now(),rospy.Duration(0.1))
    #     world_p0_list = [p0_transform.transform.translation.x, p0_transform.transform.translation.y, p0_transform.transform.translation.z] +\
    #         [p0_transform.transform.rotation.x, p0_transform.transform.rotation.y, p0_transform.transform.rotation.z, p0_transform.transform.rotation.w]
    #     world_p0_list = np.asarray(world_p0_list)
    #     print(world_p0_list)

    #     current_pose_goals_msg_anti = PoseStamped()
    #     current_pose_goals_msg_anti.header.stamp = rospy.Time.now()
    #     current_pose_goals_msg_anti.pose.position.x = p0_transform.transform.translation.x
    #     current_pose_goals_msg_anti.pose.position.y = p0_transform.transform.translation.y
    #     current_pose_goals_msg_anti.pose.position.z = p0_transform.transform.translation.z
    #     current_pose_goals_msg_anti.pose.orientation.x = p0_transform.transform.rotation.x
    #     current_pose_goals_msg_anti.pose.orientation.y = p0_transform.transform.rotation.y
    #     current_pose_goals_msg_anti.pose.orientation.z = p0_transform.transform.rotation.w
    #     current_pose_goals_msg_anti.pose.orientation.w = p0_transform.transform.rotation.z
    #     #self.current_pose_tf_pub_anti.publish(current_pose_goals_msg_anti)


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

    def __OnPoseGoalsRepuber_anti(self,goals_msg_anti):
        """
        This convert the RelaxedIk goal msg from EEPoseGoals to PoseStamped
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
        #self.current_pose_pub_anti.publish(current_pose_goals_msg_anti)

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
