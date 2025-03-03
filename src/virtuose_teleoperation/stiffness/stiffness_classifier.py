#!/usr/bin/env python

import rospy
import joblib
import numpy as np
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import PoseArray, WrenchStamped

class StiffnessClassifierNode:
    def __init__(self):
        self.model = joblib.load('stiffness_classifier_noisy.pkl')

        rospy.init_node('stiffness_classifier_node', anonymous=True)
        self.pub = rospy.Publisher('/stiffness_classification', String, queue_size=10)
        
        rospy.Subscriber('/crisp_merged_poses', PoseArray, self.crisp_pose_callback)
        rospy.Subscriber('/Crisp_IN_2HGlove', WrenchStamped, self.finger_callback, callback_args='index')
        rospy.Subscriber('/Crisp_MID_2HGlove', WrenchStamped, self.finger_callback, callback_args='middle')
        rospy.Subscriber('/Crisp_RING_2HGlove', WrenchStamped, self.finger_callback, callback_args='ring')
        rospy.Subscriber('/Crisp_TH_2HGlove', WrenchStamped, self.finger_callback, callback_args='thumb')
        rospy.Subscriber('/stiffness_detection', Float64MultiArray, self.stiffness_callback)

        self.data = {
            'index_position_x': 0.0, 'index_position_y': 0.0, 'index_position_z': 0.0,
            'index_orientation_x': 0.0, 'index_orientation_y': 0.0, 'index_orientation_z': 0.0, 'index_orientation_w': 0.0,
            'middle_position_x': 0.0, 'middle_position_y': 0.0, 'middle_position_z': 0.0,
            'middle_orientation_x': 0.0, 'middle_orientation_y': 0.0, 'middle_orientation_z': 0.0, 'middle_orientation_w': 0.0,
            'ring_position_x': 0.0, 'ring_position_y': 0.0, 'ring_position_z': 0.0,
            'ring_orientation_x': 0.0, 'ring_orientation_y': 0.0, 'ring_orientation_z': 0.0, 'ring_orientation_w': 0.0,
            'thumb_position_x': 0.0, 'thumb_position_y': 0.0, 'thumb_position_z': 0.0,
            'thumb_orientation_x': 0.0, 'thumb_orientation_y': 0.0, 'thumb_orientation_z': 0.0, 'thumb_orientation_w': 0.0,
            'force_x_index': 0.0, 'force_y_index': 0.0, 'force_z_index': 0.0,
            'force_x_middle': 0.0, 'force_y_middle': 0.0, 'force_z_middle': 0.0,
            'force_x_ring': 0.0, 'force_y_ring': 0.0, 'force_z_ring': 0.0,
            'force_x_thumb': 0.0, 'force_y_thumb': 0.0, 'force_z_thumb': 0.0,
            'torque_norm_index': 0.0, 'torque_norm_middle': 0.0, 'torque_norm_ring': 0.0, 'torque_norm_thumb': 0.0,
            'stiffness_index': 0.0, 'stiffness_middle': 0.0, 'stiffness_ring': 0.0, 'stiffness_thumb': 0.0
        }

        self.previous_prediction = None

    def crisp_pose_callback(self, msg):
        if len(msg.poses) != 4:
            rospy.logwarn("Expected 4 poses in /crisp_merged_poses message, got {}".format(len(msg.poses)))
            return

        fingers = ['index', 'middle', 'ring', 'thumb']
        for i, finger in enumerate(fingers):
            self.data['{}_position_x'.format(finger)] = msg.poses[i].position.x
            self.data['{}_position_y'.format(finger)] = msg.poses[i].position.y
            self.data['{}_position_z'.format(finger)] = msg.poses[i].position.z
            self.data['{}_orientation_x'.format(finger)] = msg.poses[i].orientation.x
            self.data['{}_orientation_y'.format(finger)] = msg.poses[i].orientation.y
            self.data['{}_orientation_z'.format(finger)] = msg.poses[i].orientation.z
            self.data['{}_orientation_w'.format(finger)] = msg.poses[i].orientation.w

        self.classify_stiffness()

    def finger_callback(self, msg, finger):
        self.data['force_x_{}'.format(finger)] = msg.wrench.force.x
        self.data['force_y_{}'.format(finger)] = msg.wrench.force.y
        self.data['force_z_{}'.format(finger)] = msg.wrench.force.z
        self.data['torque_norm_{}'.format(finger)] = msg.wrench.torque.z

        self.classify_stiffness()

    def stiffness_callback(self, msg):
        if len(msg.data) != 4:
            rospy.logwarn("Received stiffness message with unexpected length: {}".format(len(msg.data)))
            return
        self.data['stiffness_index'] = msg.data[0]
        self.data['stiffness_middle'] = msg.data[1]
        self.data['stiffness_ring'] = msg.data[2]
        self.data['stiffness_thumb'] = msg.data[3]

        self.classify_stiffness()

    def classify_stiffness(self):
        features = np.array([
            # self.data['index_position_x'], 
            # self.data['index_position_y'], 
            # self.data['index_position_z'],
            # self.data['index_orientation_x'], 
            # self.data['index_orientation_y'], 
            # self.data['index_orientation_z'], 
            # self.data['index_orientation_w'],
            # self.data['middle_position_x'], 
            # self.data['middle_position_y'], 
            # self.data['middle_position_z'],
            # self.data['middle_orientation_x'], 
            # self.data['middle_orientation_y'], 
            # self.data['middle_orientation_z'], 
            # self.data['middle_orientation_w'],
            # self.data['ring_position_x'], 
            # self.data['ring_position_y'], 
            # self.data['ring_position_z'],
            # self.data['ring_orientation_x'], 
            # self.data['ring_orientation_y'], 
            # self.data['ring_orientation_z'], 
            # self.data['ring_orientation_w'],
            # self.data['thumb_position_x'], 
            # self.data['thumb_position_y'], 
            # self.data['thumb_position_z'],
            # self.data['thumb_orientation_x'], 
            # self.data['thumb_orientation_y'], 
            # self.data['thumb_orientation_z'], 
            # self.data['thumb_orientation_w'],
            # self.data['force_x_index'], 
            # self.data['force_y_index'],
            self.data['force_z_index'], 
            # self.data['force_x_middle'], 
            # self.data['force_y_middle'], 
            self.data['force_z_middle'],
            # self.data['force_x_ring'], 
            # self.data['force_y_ring'], 
            # self.data['force_z_ring'],
            # self.data['force_x_thumb'], 
            # self.data['force_y_thumb'], 
            self.data['force_z_thumb'],
            # self.data['torque_norm_index'], 
            # self.data['torque_norm_middle'], 
            # self.data['torque_norm_ring'], 
            # self.data['torque_norm_thumb'],
            # self.data['stiffness_index'],
            # self.data['stiffness_middle'], 
            # self.data['stiffness_ring'], 
            # self.data['stiffness_thumb']
        ]).reshape(1, -1)

        prediction = self.model.predict(features)[0]
        if prediction != self.previous_prediction:
            self.previous_prediction = prediction
            self.pub.publish(String(data=str(prediction)))
            rospy.loginfo("Current object classification: {}".format(prediction))
            print("Current object classification: {}".format(prediction))

if __name__ == '__main__':
    try:
        node = StiffnessClassifierNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
