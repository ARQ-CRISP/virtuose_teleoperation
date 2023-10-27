#!/usr/bin/env python
import rospy
import actionlib

# from virtuose.msg import out_virtuose_physical_pose
from virtuose_teleoperation.msg import out_virtuose_physical_pose,xServerMsg, xSensorData
from virtuose_teleoperation.msg import PoseControlAction,PoseControlGoal

# from crisp_fingertips.msg import xServerMsg
# from allegro_hand_kdl.msg import PoseControlAction, PoseControlGoal, PoseControlResult, PoseControlFeedback

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, Point
from tf2_msgs.msg import TFMessage
from PolicyController import Policy, Policy_EE, PolicyAHEE, PolicyAHEE_yes_vel_no_eff ,PolicyAHEE_no_vel_no_eff, PolicyBC_MLP_AHEE_yVel_nEff_yTact
import numpy as np
import csv
import signal
import sys

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

        print("self.ur5_joints_data",self.ur5_joints_data)

        if all(abs(self.ur5_joints_data[j] - self.ur5_joints_setup[j]) <= 0.05 for j in range(1, 7)):
            return True
        else:
            print("setup not reached")
            return False  

class CrispFingertipNode(object):
    def __init__(self):
        self.arduino_sub = rospy.Subscriber("/xServTopic", xServerMsg, self.__OnMuxReceived)
        self.FF_th_sub=rospy.Subrscriber("", )


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


    def __OnMuxReceived(self, force_sensed):
        """
        Process tactile sensor data received from the Arduino magnetic sensor.

        Args:
            force_sensed (xServerMsg): Message containing sensor data and sensor ID.
        """
        sensor_id = force_sensed.sensorid
        points = force_sensed.points
        finger_name='unknown'
        
        print("offset\n",self.tactile_offset)
        print("Tactile data\n\n",self.tactile_data)

        if 1 <= sensor_id <= 4:
        
            for i in range(4):
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

            self.tactile_data[finger_name] = [[points[i].point.x, points[i].point.y, points[i].point.z] for i in range(4)]

            # # Update tactile_data based on sensor ID and point index
            # if sensor_id == 1:
            #     self.tactile_data['thumb'] = [[points[i].point.x, points[i].point.y, points[i].point.z] for i in range(4)]
            # elif sensor_id == 2:
            #     self.tactile_data['index'] = [[points[i].point.x, points[i].point.y, points[i].point.z] for i in range(4)]
            # elif sensor_id == 3:
            #     self.tactile_data['middle'] = [[points[i].point.x, points[i].point.y, points[i].point.z] for i in range(4)]
            # elif sensor_id == 4:
            #     self.tactile_data['ring'] = [[points[i].point.x, points[i].point.y, points[i].point.z] for i in range(4)]

            # # Publish messages for each finger with the last information of tactile_data
            # self.publish_finger_message('thumb')
            # self.publish_finger_message('index')
            # self.publish_finger_message('middle')
            # self.publish_finger_message('ring')
    
    # def publish_finger_message(self, finger_name):
    #     # Create a message and fill it with the last information of tactile_data for the specified finger
    #     message = xServerMsg()
    #     message.sensorid = self.get_sensor_id_by_finger(finger_name)
    #     message.points = []
    #     for point_data in self.tactile_data[finger_name]:
    #         point = PointStamped()
    #         point.point.x = point_data[0]
    #         point.point.y = point_data[1]
    #         point.point.z = point_data[2]
    #         message.points.append(point)
    #     # Publish the message
    #     # Determine the appropriate publisher based on the finger name and publish the message
    #     if finger_name == 'thumb':
    #         self.crisp_th_pub.publish(message)
    #     elif finger_name == 'index':
    #         self.crisp_in_pub.publish(message)
    #     elif finger_name == 'middle':
    #         self.crisp_mi_pub.publish(message)
    #     elif finger_name == 'ring':
    #         self.crisp_ri_pub.publish(message)


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
        self.allegro_joints = [0]*16
        self.allegro_velocity = [0]*16
        self.allegro_joints_eff=[0]*16

        self.client =\
            actionlib.SimpleActionClient('pose_control_action', PoseControlAction)       
            
    def __OnAHposeReceived(self, allegro_joints_msg):
        # Process Allegro hand joint states
        for i in range(0,16):
            self.allegro_joints[i] = allegro_joints_msg.position[i]
            self.allegro_velocity[i] = allegro_joints_msg.velocity[i]
            self.allegro_joints_eff[i]=allegro_joints_msg.effort[i]
        # print("self allegro joints",self.allegro_joints)
    
    def joints_control(self, jgoal):
        
        self.client.wait_for_server()
        print("JOINT CONTROLLER")
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
        
        # rospy.sleep(0.2)#/////############### SOPRA LO 0.2(5hz) FUNZIONA ABBASTANZA BENE, best at 0.5(2hz)# ovviamente questo e' perche gli sto chiedendo di fare 0.4 radianti istantaneamente
        
    def feedbackCallback(self,feedback):
    
        self.feedbacks_received = True

        rospy.logdebug("pose_action_client: got feedback:")
        rospy.logdebug(feedback)

def signal_handler(sig, frame):
    print("Saving data to CSV files...")
    with open('nn_joints_pose.csv', 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerows(nn_joints_pose_data)

    with open('nn_joints_pose_interpolation.csv', 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerows(nn_joints_pose_interpolation_data)

    print("Data has been successfully saved to 'nn_joints_pose.csv' and 'nn_joints_pose_interpolation.csv'.")
    sys.exit(0)

# Set the signal handler
signal.signal(signal.SIGINT, signal_handler)

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
        Policy_NN = PolicyAHEE_yes_vel_no_eff()
        # Policy_NN = PolicyAHEE_no_vel_no_eff()
        ee_pose_fakevirtuose=out_virtuose_physical_pose()
        setup=True #true to skip the setup phase
        i = 0
        freq_net=10
        interp_rate=10
        rate = rospy.Rate(interp_rate*freq_net) #Hz
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
                if (i%interp_rate == 0):
                    # nn_ee_pose, nn_joints_pose = Policy_NN.output(IM_ur5.ee_data,IM_sensors.tactile_data, IM_ah.allegro_joints, IM_ah.allegro_velocity)         
                    # print(nn_ee_pose_interpolation[i%10,0])
                    # print(nn_joints_pose_interpolation[i%10,15])

                    # nn_ee_pose = Policy_NN.output(IM_ur5.ee_data,IM_sensors.tactile_data, IM_ah.allegro_joints, IM_ah.allegro_velocity, IM_ah.allegro_joints_eff)
                    # nn_ee_pose, nn_joints_pose = Policy_NN.output(IM_ur5.ee_data, IM_ah.allegro_joints, IM_ah.allegro_velocity, IM_ah.allegro_joints_eff)
                    nn_ee_pose, nn_joints_pose = Policy_NN.output(IM_ur5.ee_data, IM_ah.allegro_joints, IM_ah.allegro_velocity)
                # nn_ee_pose, nn_joints_pose = Policy_NN.output(IM_ur5.ee_data, IM_ah.allegro_joints)

                    # IM_ur5.ee_data[i]   i=0-x,1-y,2-z,3-rx,4-ry,5-rz,6-rw 
                    # IM_sensors.tactile_data.finger[i].$  finger=thumb,index,middle,ring   i=0,1,2,3   $=xyz
                
                # Publish processed data if necessary
                    nn_ee_pose_interpolation = np.linspace(IM_ur5.ee_data[0:3], nn_ee_pose, num=interp_rate)
                    nn_joints_pose_interpolation = np.linspace(IM_ah.allegro_joints, nn_joints_pose, num=interp_rate)
                    
                

                # #CONTROL MESSAGES FOR cartesian_mapping_FakeVirtuose.py
                ee_pose_fakevirtuose.virtuose_physical_pose.translation.x=nn_ee_pose_interpolation[i%interp_rate,0]#0#nn_ee_pose[0]#2
                ee_pose_fakevirtuose.virtuose_physical_pose.translation.y=nn_ee_pose_interpolation[i%interp_rate,1]#nn_ee_pose[1]#0
                ee_pose_fakevirtuose.virtuose_physical_pose.translation.z=nn_ee_pose_interpolation[i%interp_rate,2]#nn_ee_pose[2]#1
                ee_pose_fakevirtuose.virtuose_physical_pose.rotation.x=0 #nn_ee_pose[3]
                ee_pose_fakevirtuose.virtuose_physical_pose.rotation.y=0 #nn_ee_pose[4]
                ee_pose_fakevirtuose.virtuose_physical_pose.rotation.z=0 #nn_ee_pose[5]
                ee_pose_fakevirtuose.virtuose_physical_pose.rotation.w=1 #nn_ee_pose[6]

                # print("nn_ee_pose",nn_ee_pose)
                # print("ee_pose_fakevirtuose",ee_pose_fakevirtuose.virtuose_physical_pose.translation)
                IM_ur5.net_fakevirtuose_pub.publish(ee_pose_fakevirtuose) #this msg replaces the input of virtuose in cartesian_mapping_FakeVirtuose.py
                # print("nn_joint_pose", nn_joints_pose)

                IM_ah.joints_control(nn_joints_pose)
                # IM_ah.joints_control(nn_joints_pose_interpolation[i%interp_rate])
                


                # print("nn_joints_pose",nn_joints_pose)
                # Publish processed data for each finger
                
                IM_sensors.publish_finger_message('thumb')
                IM_sensors.publish_finger_message('index')
                IM_sensors.publish_finger_message('middle')
                IM_sensors.publish_finger_message('ring')

                i+=1

            nn_joints_pose_data.append(nn_joints_pose.tolist())  # Convert numpy array to list and append to the data list
            nn_joints_pose_interpolation_data.append(nn_joints_pose_interpolation[i % interp_rate].tolist())  # Similar for interpolation data

            rate.sleep()

    except rospy.ROSInterruptException:
        print("Error")

    finally:
            # Save data to CSV files before exiting
            print("Saving data to CSV files...")
            with open('nn_joints_pose.csv', 'w', newline='') as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerows(nn_joints_pose_data)

            with open('nn_joints_pose_interpolation.csv', 'w', newline='') as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerows(nn_joints_pose_interpolation_data)

            print("Data has been successfully saved to 'nn_joints_pose.csv' and 'nn_joints_pose_interpolation.csv'.")