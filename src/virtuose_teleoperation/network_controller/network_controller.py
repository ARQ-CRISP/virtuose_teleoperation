#!/usr/bin/env python
import rospy
import actionlib

# from virtuose.msg import out_virtuose_physical_pose
from virtuose_teleoperation.msg import out_virtuose_physical_pose,xServerMsg, xSensorData
from virtuose_teleoperation.msg import PoseControlAction,PoseControlGoal

# from crisp_fingertips.msg import xServerMsg
# from allegro_hand_kdl.msg import PoseControlAction, PoseControlGoal, PoseControlResult, PoseControlFeedback

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, Point, WrenchStamped, PoseArray
from tf2_msgs.msg import TFMessage
from PolicyController import Policy, Policy_EE, PolicyAHEE, PolicyAHEE_yes_vel_no_eff ,PolicyAHEE_no_vel_no_eff, PolicyBC_MLP_AHEE_yVel_nEff_yTact, PolicyAHEE_no_eff_yes_ff, PolicyAHEE_yes_norm, PolicyAHEE_yes_norm_delta_q , PolicyAH_yes_norm_delta_q, PolicyAH_yes_norm_output_delta_q, PolicyAHFF_output_delta_q, PolicyAH_only_output_delta_q, PolicyAH_time_series_only_output_delta_q, PolicyAHFF_output_delta_q_TAC, PolicyAH_time_series_only_output_delta_q_TAC, PolicyAHFF_output_hglove, Classification_with_time_series_feature, PolicyAHFF_output_hglove_ee
import numpy as np
import csv
import signal
import sys
from datetime import datetime
from scipy.signal import butter, lfilter



nn_joints_pose_data = []
nn_joints_pose_interpolation_data = []
# # subriscers....3 topics

# # Policy = Policy()

# # ee_pose, joints_pose = Policy.output(3 topics)

# # pulisher.... ee_pose, joints_pose

class Network_FakeVirtuose():
    def __init__(self):
        self.ur5_ee_sub = rospy.Subscriber("/ur5/tf", TFMessage, self.__OnUr5EEposeReceived)
        self.ur5_joints_sub = rospy.Subscriber("/ur5/joint_states", JointState, self.__OnUr5JointsReceived)

        # self.ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)
        self.net_fakevirtuose_pub = rospy.Publisher("/out_virtuose_physical_pose", out_virtuose_physical_pose, queue_size=5)
        self.ee_data = [0, 0, 0, 0, 0, 0, 1]
        self.ur5_joints_setup = [2.62, -2.11, -2.04, -0.38, 1.67, -0.92]
        self.ur5_joints_data = [0, 0, 0, 0, 0, 0]
        self.setup_msg=out_virtuose_physical_pose()
        
    def __OnUr5EEposeReceived(self, tf_message):
        # Process ur5_ee_pose
        transform = tf_message.transforms[0]
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        # Accessing individual components
        self.ee_data[0] = translation.x
        self.ee_data[1] = translation.y
        self.ee_data[2] = translation.z
        # Accessing rotation components
        self.ee_data[3] = rotation.x
        self.ee_data[4] = rotation.y
        self.ee_data[5] = rotation.z
        self.ee_data[6] = rotation.w
        # self.ee_data = ee_pose

    def __OnUr5JointsReceived(self, joints_state):
        # Accessing individual components
        self.ur5_joints_data[0] = joints_state.position[0]
        self.ur5_joints_data[1] = joints_state.position[1]
        self.ur5_joints_data[2] = joints_state.position[2]
        self.ur5_joints_data[3] = joints_state.position[3]
        self.ur5_joints_data[4] = joints_state.position[4]
        self.ur5_joints_data[5] = joints_state.position[5]

    def go2setupPosition(self):

        self.setup_msg.virtuose_physical_pose.translation.x=0
        self.setup_msg.virtuose_physical_pose.translation.y=0
        self.setup_msg.virtuose_physical_pose.translation.z=0
        self.setup_msg.virtuose_physical_pose.rotation.x=0
        self.setup_msg.virtuose_physical_pose.rotation.y=0
        self.setup_msg.virtuose_physical_pose.rotation.z=0
        self.setup_msg.virtuose_physical_pose.rotation.w=1
        
        self.net_fakevirtuose_pub.publish(self.setup_msg) #this msg replaces the input of virtuose in cartesian_mapping_FakeVirtuose.py

        # print("self.ur5_joints_data",self.ur5_joints_data)

        if all(abs(self.ur5_joints_data[j] - self.ur5_joints_setup[j]) <= 0.05 for j in range(1, 7)):
            return True
        else:
            print("setup not reached")
            return False  

class CrispFingertipNode(object):
    def __init__(self):
        self.arduino_sub = rospy.Subscriber("/xServTopic", xServerMsg, self.__OnMuxReceived)
        self.FF_th_sub=rospy.Subscriber("/Crisp_TH_2HGlove",  WrenchStamped, self.__OnCrisp_FF_Received)#ForceFeedback
        self.FF_in_sub=rospy.Subscriber("/Crisp_IN_2HGlove",  WrenchStamped, self.__OnCrisp_FF_Received)
        self.FF_mi_sub=rospy.Subscriber("/Crisp_MID_2HGlove", WrenchStamped, self.__OnCrisp_FF_Received)
        self.FF_ri_sub=rospy.Subscriber("/Crisp_RING_2HGlove",WrenchStamped, self.__OnCrisp_FF_Received)

        self.crisp_th_pub = rospy.Publisher("/crisp_th_topic", xServerMsg, queue_size=10)
        self.crisp_in_pub = rospy.Publisher("/crisp_in_topic", xServerMsg, queue_size=10)
        self.crisp_mi_pub = rospy.Publisher("/crisp_mi_topic", xServerMsg, queue_size=10)
        self.crisp_ri_pub = rospy.Publisher("/crisp_ri_topic", xServerMsg, queue_size=10)

        self.tactile_offset = {
            'thumb': [[0.0, 0.0, 0.0] for _ in range(4)],
            'index': [[0.0, 0.0, 0.0] for _ in range(4)],
            'middle': [[0.0, 0.0, 0.0] for _ in range(4)],
            'ring': [[0.0, 0.0, 0.0] for _ in range(4)]
        }
        self.first_received = [False] * 4  # Flags for checking first data from each sensor
        self.tactile_data = {
            'thumb': [[0.0, 0.0, 0.0] for _ in range(4)],
            'index': [[0.0, 0.0, 0.0] for _ in range(4)],
            'middle': [[0.0, 0.0, 0.0] for _ in range(4)],
            'ring': [[0.0, 0.0, 0.0] for _ in range(4)]
        }
        self.ff_data = {
            'thumb': [0.0, 0.0, 0.0],
            'index': [0.0, 0.0, 0.0],
            'middle': [0.0, 0.0, 0.0],
            'ring': [0.0, 0.0, 0.0] 
        }

        self.frame_id_mapping = {
            '/Crisp_TH_2HGlove': 'thumb',
            '/Crisp_IN_2HGlove': 'index',
            '/Crisp_MID_2HGlove': 'middle',
            '/Crisp_RING_2HGlove': 'ring'
        }

    def __OnCrisp_FF_Received(self,ff_msg):
        ff_msg_ret = [ff_msg.wrench.force.x, ff_msg.wrench.force.y, ff_msg.wrench.force.z]
        # print("ff_msg_ret",ff_msg_ret)
        frame_id = ff_msg.header.frame_id
        # print('frame_id', frame_id)
        if frame_id in self.frame_id_mapping:
            key = self.frame_id_mapping[frame_id]
            # print("KEY",key)
            self.ff_data[key] = ff_msg_ret     

    def __OnMuxReceived(self, force_sensed):
        """
        Process tactile sensor data received from the Arduino magnetic sensor.

        Args:
            force_sensed (xServerMsg): Message containing sensor data and sensor ID.
        """
        sensor_id = force_sensed.sensorid
        points = force_sensed.points

        finger_name='unknown'
        


        if 1 <= sensor_id <= 4:
        
            # for i in range(4):
                # Update tactile_data based on sensor ID and point index
            if sensor_id == 1:
                finger_name='thumb'
            elif sensor_id == 4:
                finger_name='index'
            elif sensor_id == 3:
                finger_name='middle'
            elif sensor_id == 2:
                finger_name='ring'  

            if not self.first_received[sensor_id - 1]:
                # Set noise offsets for the current sensor and finger based on the first received data for all points
                    self.tactile_offset[finger_name] = [[points[i].point.x, points[i].point.y, points[i].point.z] for i in range(4)]
                    self.first_received[sensor_id - 1] = True
            for i in range(4):
                # Remove noise offsets from x, y, and z coordinates for each point
                points[i].point.x -= self.tactile_offset[finger_name][i][0] 
                points[i].point.y -= self.tactile_offset[finger_name][i][1] 
                points[i].point.z -= self.tactile_offset[finger_name][i][2]
                
                # if sensor_id == 2:
                    # print("points[i] ring", points[i])
                
            self.tactile_data[finger_name] = [[points[i].point.x, points[i].point.y, points[i].point.z] for i in range(4)]
            # if sensor_id == 2:
            #     print("self.tactile_data[ring]",self.tactile_data[finger_name])

    def publish_finger_message(self, finger_name):
        # Create a message and fill it with the last information of tactile_data for the specified finger
        message = xServerMsg()
        message.sensorid = self.get_sensor_id_by_finger(finger_name)
        message.points = []  # Initialize the points list

        # Iterate through tactile_data for the specified finger and populate the points list
        for point_data in self.tactile_data[finger_name]:
            x_sensor_data = xSensorData()
            x_sensor_data.taxels = 1  # Set the taxels attribute (replace 1 with appropriate value)
            
            # Populate the geometry_msgs/Point object
            x_sensor_data.point = Point()
            x_sensor_data.point.x = point_data[0]
            x_sensor_data.point.y = point_data[1]
            x_sensor_data.point.z = point_data[2]
            
            # Append the xSensorData object to the points list of the xServerMsg message
            message.points.append(x_sensor_data)

        # Publish the message based on the finger name
        if finger_name == 'thumb':
            self.crisp_th_pub.publish(message)
        elif finger_name == 'index':
            self.crisp_in_pub.publish(message)
        elif finger_name == 'middle':
            self.crisp_mi_pub.publish(message)
        elif finger_name == 'ring':
            self.crisp_ri_pub.publish(message)


    def get_sensor_id_by_finger(self, finger_name):
        # Define a mapping from finger names to sensor IDs (adjust as needed)
        finger_to_sensor_mapping = {
            'thumb': 1,
            'index': 4,
            'middle': 3,
            'ring': 2
        }
        return finger_to_sensor_mapping.get(finger_name, 0)  # Default to 0 if the finger name is not found


class PoseActionClient(object):
    joints_controller=True
    
    def __init__(self,init_pose=None):
        self.ah_sub = rospy.Subscriber("allegroHand_0/joint_states", JointState, self.__OnAHposeReceived)
        self.ah_sub = rospy.Subscriber("crisp_merged_poses", PoseArray, self.__OnFingetipMergedPositionReceived)
        self.allegro_joints = [0]*16
        self.allegro_velocity = [0]*16
        self.allegro_joints_eff=[0]*16
        self.crispFingertipMerged=None

        
        self.NN_AH_solution = rospy.Publisher("/NN_AH_solution", JointState, queue_size=10)
        self.joint_pose_msg=JointState()

        self.client =\
            actionlib.SimpleActionClient('pose_control_action', PoseControlAction)       
            
    def __OnAHposeReceived(self, allegro_joints_msg):
        # Process Allegro hand joint states
        for i in range(0,16):
            self.allegro_joints[i] = allegro_joints_msg.position[i]
            self.allegro_velocity[i] = allegro_joints_msg.velocity[i]
            self.allegro_joints_eff[i]=allegro_joints_msg.effort[i]
        # print("self ALLLLLLEGRO allegro joints",self.allegro_joints)

    def __OnFingetipMergedPositionReceived(self, crisp_merged_poses_msg):
        """
        IMPORTANT: To access values later, use crispFingertipMerged[0].position.x  0=index, 1=middle, 2=ring, 3=thumb
        # For example, crispFingertipMerged[0].position.x gives you the x/y/z-coordinate of the position for the first pose.
        # Similarly, crispFingertipMerged[0].orientation.x would give you the x/y/z/w-component of the orientation quaternion
        # for the first pose.      
        """
        self.crispFingertipMerged = crisp_merged_poses_msg.poses[:] #use crispFingertipMerged[0].position.x
            
    def joints_control(self, jgoal, setup=False):
        
        self.client.wait_for_server()
        # print("JOINT CONTROLLER")
        # close thumb to touch the palm
        # joint_pose = \
        #     [0.1, 0.1, 0.0, 0.2,\
        #     0.1, 0.1, 0.0, 0.2,\
        #     0.1, 0.1, 0.0, 0.0,\
        #     3.14, -0.03, 0.0, 0.0] ## 0 broken joint, 1 rot vertical, 2 fing pitch, 3 tip pitch
        joint_pose = \
            [jgoal[0], jgoal[1], jgoal[2],  jgoal[3],\
             jgoal[4], jgoal[5], jgoal[6],  jgoal[7],\
             jgoal[8], jgoal[9], jgoal[10], jgoal[11],\
             jgoal[12],jgoal[13],jgoal[14], jgoal[15]] ## 0 broken joint, 1 rot vertical, 2 fing pitch, 3 tip pitch        
        # send the pose to the server
        goal = PoseControlGoal(joint_pose=joint_pose)
        self.client.send_goal(goal, feedback_cb=self.feedbackCallback)

        self.joint_pose_msg.position=joint_pose
        self.NN_AH_solution.publish(self.joint_pose_msg)
        # rospy.sleep(0.1)#/////############### SOPRA LO 0.2(5hz) FUNZIONA ABBASTANZA BENE, best at 0.5(2hz)# ovviamente questo e' perche gli sto chiedendo di fare 0.4 radianti istantaneamente
        if setup==True:
            rospy.sleep(2)
            print("SETUP_TRUE")
            return False


    def cartesian_control(self, de_in, de_mi,de_ri,de_th):
        self.client.wait_for_server()
        print("CARTESIAN CONTROLLER")
        finger_poses = []

        in_set = [0.0935, 0.0873, 0.1425, 0.9597, 0.2028, 0.1940, 0.0015]
        mi_set = [0.1064, 0.0092, 0.1627, 0.9020, 0.0393, 0.4294, -0.0178]  # Initial pose values for middle finger
        ri_set = [0.0689, -0.0519, 0.1396, 0.9860, -0.0378, 0.1579, -0.0373]  # Initial pose values for ring finger
        th_set = [0.0687, 0.1170, 0.0563, 0.1961, 0.0134, 0.4522, 0.8699]  # Initial pose values for thumb


        finger_poses.append(self.list_to_pose([in_set[0] + de_in[0], in_set[1] + de_in[1], in_set[2] + de_in[2], \
                                            in_set[3] + de_in[3], in_set[4] + de_in[4], in_set[5] + de_in[5],
                                            in_set[6] + de_in[6]]))  # finger 0 index

        finger_poses.append(self.list_to_pose([mi_set[0] + de_mi[0], mi_set[1] + de_mi[1], mi_set[2] + de_mi[2], \
                                            mi_set[3] + de_mi[3], mi_set[4] + de_mi[4], mi_set[5] + de_mi[5],
                                            mi_set[6] + de_mi[6]]))  # finger 1 middle

        finger_poses.append(self.list_to_pose([ri_set[0] + de_ri[0], ri_set[1] + de_ri[1], ri_set[2] + de_ri[2], \
                                            ri_set[3] + de_ri[3], ri_set[4] + de_ri[4], ri_set[5] + de_ri[5],
                                            ri_set[6] + de_ri[6]]))  # finger 2 ring finger

        finger_poses.append(self.list_to_pose([th_set[0] + de_th[0], th_set[1] + de_th[1], th_set[2] + de_th[2], \
                                            th_set[3] + de_th[3], th_set[4] + de_th[4], th_set[5] + de_th[5],
                                            th_set[6] + de_th[6]]))  # finger 3 thumb

        goal = PoseControlGoal(cartesian_pose=finger_poses)
        self.client.send_goal(goal, feedback_cb=self.feedbackCallback)
        # to test the feedback mechanism:
        self.feedbacks_received = False


    def feedbackCallback(self,feedback):
    
        self.feedbacks_received = True

        rospy.logdebug("pose_action_client: got feedback:")
        rospy.logdebug(feedback)

def signal_handler(sig, frame):
    print("Saving data to CSV files...")
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    
    file1_name = f'C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\csv_outputs\\nn_joints_pose_{current_time}.csv'
    file2_name = f'C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\csv_outputs\\nn_joints_pose_interpolation_{current_time}.csv'

    with open(file1_name, 'w', newline='') as csvfile1:
        csvwriter = csv.writer(csvfile1)
        csvwriter.writerows(nn_joints_pose_data)

    with open(file2_name, 'w', newline='') as csvfile2:
        csvwriter = csv.writer(csvfile2)
        csvwriter.writerows(nn_joints_pose_interpolation_data)

    print(f"Data has been successfully saved to '{file1_name}' and '{file2_name}'.")
    sys.exit(0)

# Set the signal handler
signal.signal(signal.SIGINT, signal_handler)


class ActionPrediction(object):
    def __init__(self,Policy_NN, Seg_net, ee_data, allegro_joints, interp_rate, fs ):
        
        self.nn_ee_pose_interpolation = np.zeros((10, 3))  #change it to 7 later
        self.nn_AH_joints_interpolation = np.zeros((10, 16))
        self.nn_ee_pose_butterfilter = np.zeros((interp_rate, 3))
        self.nn_AH_joints_butterfilter = np.zeros((interp_rate, 16))

        self.Policy_NN = Policy_NN
        self.IM_ur5_buffer = np.array(ee_data)
        self.IM_ah_buffer = np.array(allegro_joints)
        self.IM_ee_vir_buffer = np.zeros(3)
        self.ee_predict_pose = np.zeros(3)
        self.record_action = np.zeros((interp_rate, 16))
        self.interp_rate = interp_rate
        self.fs = fs

        self.ee_data_history = []
        self.allegro_joints_history = []
        self.ff_data_history = []
        self.ff_full_data_history = []
        self.finger_state_history = []
        self.time_length = 0
        self.max_length = 6
        self.Seg_NET = Seg_net

    def butter_lowpass_filter(self, data, cutoff=11, order=5):
        nyq = 0.5 * self.fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        y = lfilter(b, a, data)
        return y
    def action_prediction_with_interpolation(self, i, ee_state, allegro_joints, tactile_data):
        if (i%self.interp_rate == 0):

            allegro_state = allegro_joints.copy()
            nn_joints_pose = self.Policy_NN.output(allegro_state, tactile_data.copy())
            nn_ee_pose = ee_state

        
            self.nn_ee_pose_interpolation = np.linspace(ee_state[0:3], nn_ee_pose, num=self.interp_rate+1)[1:]
            self.nn_AH_joints_interpolation = np.linspace(allegro_state, nn_joints_pose, num=self.interp_rate+1)[1:]
            return self.nn_ee_pose_interpolation[0], self.nn_AH_joints_interpolation[0]
        else:
            return self.nn_ee_pose_interpolation[i%self.interp_rate], self.nn_AH_joints_interpolation[i%self.interp_rate]
                    
    def action_prediction_with_interpolation_delta_q(self,i,  ee_state, allegro_joints, tactile_data):
        if (i%interp_rate == 0):
            # print("ACTION PRED IM_AH",IM_ah.allegro_joints)
            # print("ACTION PRED",allegro_joints)

            # nn_ee_pose, nn_joints_pose = Policy_NN.output(IM_ur5.ee_data,IM_sensors.tactile_data, IM_ah.allegro_joints, IM_ah.allegro_velocity)         
            # print(nn_ee_pose_interpolation[i%10,0])
            # print(nn_AH_joints_interpolation[i%10,15])

            # nn_ee_pose = Policy_NN.output(IM_ur5.ee_data,IM_sensors.tactile_data, IM_ah.allegro_joints, IM_ah.allegro_velocity, IM_ah.allegro_joints_eff)
            # nn_ee_pose, nn_joints_pose = Policy_NN.output(IM_ur5.ee_data, IM_ah.allegro_joints, IM_ah.allegro_velocity, IM_ah.allegro_joints_eff)
            # nn_ee_pose, nn_joints_pose = Policy_NN.output(IM_ur5.ee_data, IM_ah.allegro_joints, IM_ah.allegro_velocity)
            input_ee_delta = np.array(ee_state) - self.IM_ur5_buffer

            input_ah_delta = np.array(allegro_joints) - self.IM_ah_buffer
            # nn_ee_pose_delta, nn_joints_pose_delta = Policy_NN.output(input_ee_delta, input_ah_delta, tactile_data)
            nn_joints_pose_delta = self.Policy_NN.output( allegro_joints, input_ah_delta, tactile_data)
            nn_ee_pose = np.array(ee_state[0:3])
            # print(allegro_joints) 
            # print(nn_joints_pose_delta)
            nn_joints_pose = np.array(allegro_joints) + nn_joints_pose_delta
            self.IM_ur5_buffer = np.array(ee_state)
            self.IM_ah_buffer = np.array(allegro_joints)

            # nn_ee_pose, nn_joints_pose = Policy_NN.output(IM_ur5.ee_data, IM_ah.allegro_joints, tactile_data)

        # nn_ee_pose, nn_joints_pose = Policy_NN.output(IM_ur5.ee_data, IM_ah.allegro_joints)

            # IM_ur5.ee_data[i]   i=0-x,1-y,2-z,3-rx,4-ry,5-rz,6-rw 
            # IM_sensors.tactile_data.finger[i].$  finger=thumb,index,middle,ring   i=0,1,2,3   $=xyz

        # Publish processed data if necessary
            # nn_ee_pose_interpolation = np.linspace(ee_state[0:3], nn_ee_pose, num=interp_rate)
            self.nn_ee_pose_interpolation = nn_ee_pose
            self.nn_AH_joints_interpolation = nn_joints_pose
            # print("self.nn_AH_joints_interpolation",self.nn_AH_joints_interpolation)
            return self.nn_ee_pose_interpolation, self.nn_AH_joints_interpolation
        else:
            # print("ELSE.nn_AH_joints_interpolation",self.nn_AH_joints_interpolation)

            return self.nn_ee_pose_interpolation, self.nn_AH_joints_interpolation
        #     self.nn_AH_joints_interpolation = np.linspace(np.array(allegro_joints), nn_joints_pose, num=interp_rate)
        #     print("self.nn_AH_joints_interpolation",self.nn_AH_joints_interpolation)
        #     return self.nn_ee_pose_interpolation[i%interp_rate], self.nn_AH_joints_interpolation[i%interp_rate]
        # else:
        #     print("ELSE.nn_AH_joints_interpolation",self.nn_AH_joints_interpolation)

        #     return self.nn_ee_pose_interpolation[i%interp_rate], self.nn_AH_joints_interpolation[i%interp_rate]
                    
    def action_prediction_without_interpolation_delta_q_filtered(self,i, ee_state, allegro_joints, tactile_data):
        # self.IM_xx_buffer is used to keep the filtered value
        if (i%interp_rate == 0):
            a = 0.1
            # input_ee_delta = np.array(ee_state) - self.IM_ur5_buffer

            # input_ah_delta = np.array(allegro_joints) - self.IM_ah_buffer
            # nn_ee_pose_delta, nn_joints_pose_delta = Policy_NN.output(input_ee_delta, input_ah_delta, tactile_data)
            nn_joints_pose_delta = self.Policy_NN.output( allegro_joints, tactile_data)
            nn_ee_pose_predict = np.array(ee_state[0:3])
            nn_joints_pose_predict = np.array(allegro_joints) + nn_joints_pose_delta
            # self.IM_ur5_buffer += a * (nn_ee_pose_predict - self.IM_ur5_buffer)
            self.IM_ah_buffer += a * (nn_joints_pose_predict - self.IM_ah_buffer)

            return  nn_ee_pose_predict, self.IM_ah_buffer
        else:

            return nn_ee_pose_predict,  self.IM_ah_buffer

    def action_prediction_delta_q_butter_filtered(self,i, ee_state, allegro_joints, tactile_data):

        if (i%self.interp_rate == 0):
            filtered_nn_joint_pose_delta = np.zeros((self.interp_rate, 16))
            ah_previous = self.IM_ah_buffer.copy()
            for m in range(16):
                filtered_nn_joint_pose_delta[:, m] = self.butter_lowpass_filter(self.record_action[:, m])
            self.nn_ee_pose_butterfilter = np.tile(ee_state[0:3], (self.interp_rate, 1))
            self.nn_AH_joints_butterfilter = np.tile(allegro_joints, (self.interp_rate, 1)) + filtered_nn_joint_pose_delta
            # print(self.nn_AH_joints_butterfilter.shape)
            nn_joints_pose_delta = self.Policy_NN.output( allegro_joints, tactile_data)
            self.record_action[i%self.interp_rate, :] = nn_joints_pose_delta
            return self.nn_ee_pose_butterfilter[0], self.nn_AH_joints_butterfilter[0]
        else:
            nn_joints_pose_delta = self.Policy_NN.output( allegro_joints, tactile_data)
            self.record_action[i%self.interp_rate, :] = nn_joints_pose_delta
            return self.nn_ee_pose_butterfilter[i%self.interp_rate], self.nn_AH_joints_butterfilter[i%self.interp_rate]

    def action_prediction_with_interpolation_delta_q_filtered(self,i, ee_state, allegro_joints, tactile_data):
        
        if (i%self.interp_rate == 0):
            a = 0.05

            ah_previous = self.IM_ah_buffer.copy()
            nn_joints_pose = self.Policy_NN.output( allegro_joints.copy(), tactile_data.copy())
            # nn_joints_pose_delta = self.Policy_NN.output( allegro_joints, tactile_data)
            # nn_joints_pose_delta = self.Policy_NN.output( allegro_joints)
            nn_ee_pose_predict = np.array(ee_state[0:3])
            # self.IM_ur5_buffer += a * (nn_ee_pose_predict - self.IM_ur5_buffer)
            self.IM_ah_buffer += a * (nn_joints_pose - self.IM_ah_buffer)
            self.nn_ee_pose_interpolation = np.linspace(ee_state[0:3], nn_ee_pose_predict, num=self.interp_rate+1)[1:]
            self.nn_AH_joints_interpolation = np.linspace(ah_previous, self.IM_ah_buffer, num=self.interp_rate+1)[1:]
            return self.nn_ee_pose_interpolation[0], self.nn_AH_joints_interpolation[0]
        else:
            return self.nn_ee_pose_interpolation[i%self.interp_rate], self.nn_AH_joints_interpolation[i%self.interp_rate]
        
    def action_prediction_with_time_series_input_delta_q(self,i,  ee_state, allegro_joints, tactile_data):
        if (i%interp_rate == 0):
            nn_joints_pose_delta, tactile_prediction = self.Policy_NN.output(self.IM_ah_buffer.copy(),  allegro_joints)
            print('tactile_prediction', tactile_prediction)
            nn_ee_pose = np.array(ee_state[0:3])
            nn_joints_pose = np.array(allegro_joints) + nn_joints_pose_delta
            self.IM_ur5_buffer = np.array(ee_state)
            self.IM_ah_buffer = np.array(allegro_joints)
            self.nn_ee_pose_interpolation = np.linspace(ee_state[0:3], nn_ee_pose, num=self.interp_rate+1)[1:]
            self.nn_AH_joints_interpolation = np.linspace(allegro_joints, nn_joints_pose, num=self.interp_rate+1)[1:]
            for j in range(4):
                if j == 3: 
                    if tactile_prediction[j] > 0.2:
                        self.nn_AH_joints_interpolation[:, 14] += 0.08
                else:
                    if tactile_prediction[j] > 0.2:
                        self.nn_AH_joints_interpolation[:, 1+4*j] += 0.05
                

            return self.nn_ee_pose_interpolation[0], self.nn_AH_joints_interpolation[0]
        else:


            return self.nn_ee_pose_interpolation[i%self.interp_rate], self.nn_AH_joints_interpolation[i%self.interp_rate]

    def action_prediction_with_interpolation_delta_q_filtered_and_tactile_prediction(self,i, ee_state, allegro_joints, tactile_data):
        
        if (i%self.interp_rate == 0):
            # a = 0.1

            ah_previous = self.IM_ah_buffer.copy()
            nn_joints_pose_delta, tactile_prediction = self.Policy_NN.output( allegro_joints, tactile_data)
            # nn_joints_pose_delta = self.Policy_NN.output( allegro_joints)
            print('tactile_prediction', tactile_prediction)
            nn_ee_pose_predict = np.array(ee_state[0:3])
            nn_joints_pose_predict = np.array(allegro_joints) + nn_joints_pose_delta
            # self.IM_ur5_buffer += a * (nn_ee_pose_predict - self.IM_ur5_buffer)
            # self.IM_ah_buffer += a * (nn_joints_pose_predict - self.IM_ah_buffer)
            self.IM_ah_buffer = np.array(nn_joints_pose_predict)
            self.nn_ee_pose_interpolation = np.linspace(ee_state[0:3], nn_ee_pose_predict, num=self.interp_rate+1)[1:]
            self.nn_AH_joints_interpolation = np.linspace(ah_previous, self.IM_ah_buffer, num=self.interp_rate+1)[1:]
            for j in range(4):
                if j == 3: 
                    if tactile_prediction[j] > 0.1:
                        self.nn_AH_joints_interpolation[:, 14] += 0.08
                else:
                    if tactile_prediction[j] > 0.1:
                        self.nn_AH_joints_interpolation[:, 1+4*j] += 0.05
                        self.nn_AH_joints_interpolation[:, 2+4*j] += 0.05
            return self.nn_ee_pose_interpolation[0], self.nn_AH_joints_interpolation[0]
        else:
            return self.nn_ee_pose_interpolation[i%self.interp_rate], self.nn_AH_joints_interpolation[i%self.interp_rate]
        
    def label_prediction(self,i, ee_data, allegro_joints, tactile_data, finger_raw_state):
        finger_raw_state_0 = np.array(([finger_raw_state[0].position.x, finger_raw_state[0].position.y,finger_raw_state[0].position.z, finger_raw_state[0].orientation.x, finger_raw_state[0].orientation.y, finger_raw_state[0].orientation.z, finger_raw_state[0].orientation.w]))
        finger_raw_state_1 = np.array(([finger_raw_state[1].position.x, finger_raw_state[1].position.y,finger_raw_state[1].position.z, finger_raw_state[1].orientation.x, finger_raw_state[1].orientation.y, finger_raw_state[1].orientation.z, finger_raw_state[1].orientation.w]))
        finger_raw_state_2 = np.array(([finger_raw_state[2].position.x, finger_raw_state[2].position.y,finger_raw_state[2].position.z, finger_raw_state[2].orientation.x, finger_raw_state[2].orientation.y, finger_raw_state[2].orientation.z, finger_raw_state[2].orientation.w]))
        finger_raw_state_3 = np.array(([finger_raw_state[3].position.x, finger_raw_state[3].position.y,finger_raw_state[3].position.z, finger_raw_state[3].orientation.x, finger_raw_state[3].orientation.y, finger_raw_state[3].orientation.z, finger_raw_state[3].orientation.w]))

        finger_state = np.concatenate((finger_raw_state_0, finger_raw_state_1, finger_raw_state_2, finger_raw_state_3))

        if (i%self.interp_rate == 0):
            ff_data, ff_full_state = self.tactile_data_process(tactile_data)
            self.ee_data_history.insert(0, ee_data)
            self.allegro_joints_history.insert(0, allegro_joints)
            self.finger_state_history.insert(0, finger_state)
            self.ff_data_history.insert(0, ff_data)
            self.ff_full_data_history.insert(0, ff_full_state)
            if (self.time_length < self.max_length):
                self.time_length += 1
                # self.nn_ee_pose_interpolation = np.zeros((10,3))
                # self.nn_ee_pose_interpolation = np.tile(ee_data[0:3], (self.interp_rate, 1))
                self.nn_AH_joints_interpolation = np.tile(allegro_joints, (self.interp_rate, 1))
                nn_joints_pose, ee_predict_control = self.Policy_NN.output( allegro_joints.copy(), tactile_data.copy(), ee_data.copy())
                self.ee_predict_pose = ee_predict_control
                return self.ee_predict_pose, self.nn_AH_joints_interpolation[0]
            
            else:
                a = 0.1
                self.ee_data_history.pop()
                self.allegro_joints_history.pop()
                self.finger_state_history.pop()
                self.ff_data_history.pop()
                self.ff_full_data_history.pop()

                Policy_name = self.Seg_NET.output(self.ee_data_history, self.allegro_joints_history,  self.ff_data_history, self.ff_full_data_history, self.finger_state_history)
                # print(Policy_name)

                ################test
                # self.nn_ee_pose_interpolation = np.tile(ee_data[0:3], (self.interp_rate, 1))
                # self.nn_AH_joints_interpolation = np.tile(allegro_joints, (self.interp_rate, 1))

                ah_previous = self.IM_ah_buffer.copy()
                # nn_joints_pose = self.Policy_NN.output( allegro_joints.copy(), tactile_data.copy())
                nn_joints_pose, ee_predict_control = self.Policy_NN.output( allegro_joints.copy(), tactile_data.copy(), ee_data.copy())
                # nn_joints_pose_delta = self.Policy_NN.output( allegro_joints, tactile_data)
                # nn_joints_pose_delta = self.Policy_NN.output( allegro_joints)
                # nn_ee_pose_predict = np.array(ee_data[0:3])
                # print('ee_predict_control', ee_predict_control)
                # self.IM_ur5_buffer += a * (nn_ee_pose_predict - self.IM_ur5_buffer)
                self.IM_ah_buffer += a * (nn_joints_pose - self.IM_ah_buffer)
                # self.nn_ee_pose_interpolation = np.linspace(self.IM_ee_vir_buffer, ee_predict_control, num=self.interp_rate+1)[1:]
                self.nn_AH_joints_interpolation = np.linspace(ah_previous, self.IM_ah_buffer, num=self.interp_rate+1)[1:]
                self.ee_predict_pose = ee_predict_control
                # print("IM_ee_vir_buffer",self.IM_ee_vir_buffer)
                # self.ee_pose = ee_predict_control
                return self.ee_predict_pose, self.nn_AH_joints_interpolation[0]
        else:
            return self.ee_predict_pose, self.nn_AH_joints_interpolation[i%self.interp_rate]
        
    def tactile_data_process(self, force_feedback):
        thumb_data = np.array(force_feedback['thumb'])
        index_data = np.array(force_feedback['index'])
        middle_data = np.array(force_feedback['middle'])
        ring_data = np.array(force_feedback['ring'])
        thumb_norm = np.linalg.norm(thumb_data)
        index_norm = np.linalg.norm(index_data)
        middle_norm = np.linalg.norm(middle_data)
        ring_norm = np.linalg.norm(ring_data)
        # print("force_feedback",force_feedback)

        ff_full_state_limit1 = 30
        ff_full_state_limit2 = 30
        index_data = np.where(index_data < ff_full_state_limit1, 0, index_data) #index
        middle_data = np.where(middle_data < ff_full_state_limit2, 0, middle_data) #mid
        ring_data = np.where(ring_data < ff_full_state_limit1, 0, ring_data) #ring
        thumb_data = np.where(thumb_data < ff_full_state_limit1, 0, thumb_data) #thumb
        #Normalize:
        index_data = self.normalize_using_sigmoid(index_data, -50, 400)
        middle_data = self.normalize_using_sigmoid(middle_data, -50, 400)
        ring_data = self.normalize_using_sigmoid(ring_data, -50, 400)
        thumb_data = self.normalize_using_sigmoid(thumb_data, -50, 400)

        ff_full_state = np.concatenate((index_data, middle_data, ring_data, thumb_data))



        # Limit the value
        limit1 = 100
        limit2 = 50
        thumb_norm = np.where(thumb_norm < limit1, 0, thumb_norm)
        index_norm = np.where(index_norm < limit1, 0, index_norm)
        middle_norm = np.where(middle_norm < limit2, 0, middle_norm)
        ring_norm = np.where(ring_norm < limit1, 0, ring_norm)
        #Normalize:
        index_norm = self.normalize_using_sigmoid(index_norm, -50, 400)
        middle_norm = self.normalize_using_sigmoid(middle_norm, -50, 400)
        ring_norm = self.normalize_using_sigmoid(ring_norm, -50, 400)
        thumb_norm = self.normalize_using_sigmoid(thumb_norm, -50, 400)

        ff_data = np.concatenate(([index_norm], [index_norm], [ring_norm], [thumb_norm]))

        return  ff_data, ff_full_state

    def normalize_using_sigmoid(self, array, min_val, max_val, k=0.02):
        # Center the range around 0 and scale it
        b = (max_val + min_val) / 2
        # Apply the sigmoid function to the entire array
        normalized_array = 1 / (1 + np.exp(-k*(array - b)))
        return normalized_array

if __name__ == '__main__':
    try:
        rospy.init_node('NNetwork_Controller')

        IM_ur5 = Network_FakeVirtuose()
        IM_ah = PoseActionClient()
        IM_sensors = CrispFingertipNode()

        # Policy_NN = Policy()  # Initialize Policy class
        # Policy_NN = Policy_EE() #  Initialize Policy class
        # Policy_NN = PolicyBC_MLP_AHEE_yVel_nEff_yTact()

        # Policy_NN = PolicyAHEE() #  Initialize Policy class
        # Policy_NN = PolicyAHEE_yes_vel_no_eff()
        # Policy_NN = PolicyAHEE_yes_norm()
        # Policy_NN = PolicyAH_yes_norm_output_delta_q()
        # Policy_NN = PolicyAHFF_output_delta_q_TAC()
        # Policy_NN = PolicyAHFF_output_hglove()
        Policy_NN = PolicyAHFF_output_hglove_ee()
        Seg_Net = Classification_with_time_series_feature()
        # Policy_NN = PolicyAH_time_series_only_output_delta_q_TAC()
        # Policy_NN = PolicyAH_only_output_delta_q()
        # Policy_NN = PolicyAH_time_series_only_output_delta_q()
        # Policy_NN = PolicyAHFF_output_delta_q()
        # Policy_NN = PolicyAHEE_no_vel_no_eff()
        # setup_AH_temp=[0.17,0.47,0.25,-0.27,    0.18,0.50,0.28,-0.14,  0.12,0.72,0.44,-0.25,  1.49,-0.07,0.31,-0.16]
        # setup_AH_temp=[0.0,0.79,0.52,-0.21,    -0.00,0.75,0.61,-0.14,  0.14,1.01,0.61,-0.13,  1.49,-0.06,0.29,-0.19]
        # setup_AH_temp = [0.15, 0.82, 0.46, -0.13,    0.14, 0.84, 0.47, -0.10,       0.26, 0.91, 0.68,-0.14,    1.49, -0.05, 0.26, -0.188]   ####Top grasp
        # setup_AH_temp = [-0.04, 0.52, 0.28, 0.29,      -0.03, 0.52, 0.31, 0.30,      0.00, 0.44, 0.26, 0.19,     1.49, -0.14, 0.11, -0.20]   ###Side grasping
        # setup_AH_temp = [0.07, 0.166, 0.04, 0.12,     0.04, 0.18, 0.04, 0.11,       0.07, 0.53, 0.14, 0.015,      1.49, -0.14, 0.119, -0.145]  ###Rotation

        # setup_AH_temp = [0.0, 0.0, 0.0, -0.0,    0.0, 0.0, 0.0, -0.0,       0.0, 0.0, 0.0,0.0,    1.49, -0.05, 0.26, -0.188]   ####Top grasp
        # setup_AH_temp = [0.07, 0.42, 0.19, 0.05,    0.07, 0.42, 0.20, 0.06,       0.13, 0.4, 0.23, -0.04,    1.49, -0.12, 0.12, -0.20]   ####Top grasp
        setup_AH_temp = [0.09, 0.48, 0.15, -0.02,    0.09, 0.38, 0.27, 0.05,       0.19, 0.44, 0.25, -0.06,    1.49, -0.04, 0.17, -0.20]
        # setup_AH_temp =    [0.1, 0.1, 0.0, 0.2,\
        #     0.1, 0.1, 0.0, 0.2,\
        #     0.1, 0.1, 0.0, 0.0,\
        #     1.49, -0.05, 0.26, -0.188] 
        
        ee_pose_fakevirtuose=out_virtuose_physical_pose()
        setup=True #true to skip the setup phase
        setup_hand=True
        i = 0

        freq_net=10
        interp_rate=10
        rate = rospy.Rate(interp_rate*freq_net) #Hz
        # IM_AP = ActionPrediction(Policy_NN, IM_ur5.ee_data.copy(), setup_AH_temp, interp_rate, interp_rate*freq_net)
        IM_AP = ActionPrediction(Policy_NN, Seg_Net, IM_ur5.ee_data.copy(), setup_AH_temp, interp_rate, interp_rate*freq_net)
        while not rospy.is_shutdown():

            if setup==False:
                rospy.loginfo("GOING TO SETUP POSITION")
                setup=IM_ur5.go2setupPosition()

                if setup==True:
                    i=0
                    rospy.logwarn("SETUP POSITION REACHED")
                    rospy.sleep(3)


            # Call Policy.output with necessary inputs from subscribers
            # nn_ee_pose, nn_joints_pose = Policy_NN.output(IM_ur5.ee_data,IM_sensors.tactile_data, IM_ah.allegro_joints, IM_ah.allegro_velocity, IM_ah.allegro_joints_eff)
            else:

                if setup_hand==True:
                   setup_hand=IM_ah.joints_control(setup_AH_temp,True) 
                #    setup_hand=IM_ah.joints_control([0.0,0.0,0.0,-0.0,    0.0,0.0,0.0,-0.0,  0.0,0.0,0.0,-0.0, 1.49,-0.0,0.0,-0.0],True)

                # nn_ee_pose, nn_joints_pose_interpolation = IM_AP.action_prediction_with_interpolation_delta_q_filtered(i,  IM_ur5.ee_data.copy(), IM_ah.allegro_joints.copy(), IM_sensors.ff_data.copy())
                nn_ee_pose, nn_joints_pose_interpolation = IM_AP.label_prediction(i,  IM_ur5.ee_data.copy(), IM_ah.allegro_joints.copy(), IM_sensors.ff_data.copy(), IM_ah.crispFingertipMerged.copy())
                # print(nn_ee_pose)
                # print("NETWORK INPUT",IM_sensors.ff_data)

                # #CONTROL MESSAGES FOR cartesian_mapping_FakeVirtuose.py
                ee_pose_fakevirtuose.virtuose_physical_pose.translation.x=-0.02#nn_ee_pose[0]#0#nn_ee_pose[0]#2 # positive x robot move in lab direction, negative x robot move to the base wrt initial pose
                ee_pose_fakevirtuose.virtuose_physical_pose.translation.y=0#nn_ee_pose[1]#nn_ee_pose[1]#0 ###positive y robot down, negative robot up wrt initial pose
                ee_pose_fakevirtuose.virtuose_physical_pose.translation.z=0#nn_ee_pose[2]#nn_ee_pose[2]#1 positive z robot go forward, negative z robot go backward wrt initial pos
                ee_pose_fakevirtuose.virtuose_physical_pose.rotation.x=0 #nn_ee_pose[3]
                ee_pose_fakevirtuose.virtuose_physical_pose.rotation.y=0 #nn_ee_pose[4]
                ee_pose_fakevirtuose.virtuose_physical_pose.rotation.z=0 #nn_ee_pose[5]
                ee_pose_fakevirtuose.virtuose_physical_pose.rotation.w=1 #nn_ee_pose[6]

                print("nn_ee_pose",nn_ee_pose)
                # print("ee_pose_fakevirtuose",ee_pose_fakevirtuose.virtuose_physical_pose.translation)
                IM_ur5.net_fakevirtuose_pub.publish(ee_pose_fakevirtuose) #this msg replaces the input of virtuose in cartesian_mapping_FakeVirtuose.py
                
                # print("nn_joint_pose", nn_joints_pose)

                # IM_ah.joints_control(nn_joints_pose_interpolation)
                # print("PUBLISHER",nn_joints_pose_interpolation)
                IM_ah.joints_control(nn_joints_pose_interpolation)
                


                # print("nn_joints_pose",nn_joints_pose)
                # Publish processed data for each finger
                
                IM_sensors.publish_finger_message('thumb')
                IM_sensors.publish_finger_message('index')
                IM_sensors.publish_finger_message('middle')
                IM_sensors.publish_finger_message('ring')

                i+=1

            nn_joints_pose_data.append(IM_ah.allegro_joints.copy())  # Convert numpy array to list and append to the data list
            nn_joints_pose_interpolation_data.append(nn_joints_pose_interpolation.tolist())  # Similar for interpolation data

            rate.sleep()

    except rospy.ROSInterruptException:
        print("Error")

    finally:
        # Save data to CSV files before exiting
        print("Saving data to CSV files...")
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        
        file1_name = f'nn_joints_pose_{current_time}.csv'
        file2_name = f'nn_joints_pose_interpolation_{current_time}.csv'

        with open(file1_name, 'w', newline='') as csvfile1:
            csvwriter = csv.writer(csvfile1)
            csvwriter.writerows(nn_joints_pose_data)

        with open(file2_name, 'w', newline='') as csvfile2:
            csvwriter = csv.writer(csvfile2)
            csvwriter.writerows(nn_joints_pose_interpolation_data)

        print(f"Data has been successfully saved to '{file1_name}' and '{file2_name}'.")
