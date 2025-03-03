#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseArray, WrenchStamped

class StiffnessDetectionNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('stiffness_detection_node', anonymous=True)

        # Subscribe to the specified topics
        self.sub_crisp_pose = rospy.Subscriber('/crisp_merged_poses', PoseArray, self.crisp_pose_callback)
        self.sub_finger_wrench_0 = rospy.Subscriber('/Crisp_IN_2HGlove', WrenchStamped, self.finger_callback, callback_args='index')
        self.sub_finger_wrench_1 = rospy.Subscriber('/Crisp_MID_2HGlove', WrenchStamped, self.finger_callback, callback_args='middle')
        self.sub_finger_wrench_2 = rospy.Subscriber('/Crisp_RING_2HGlove', WrenchStamped, self.finger_callback, callback_args='ring')
        self.sub_finger_wrench_3 = rospy.Subscriber('/Crisp_TH_2HGlove', WrenchStamped, self.finger_callback, callback_args='thumb')

        # Publisher for the stiffness detection topic
        self.pub_stiffness_detection = rospy.Publisher('/stiffness_detection', Float64MultiArray, queue_size=10)

        # Initialize storage for the data from each topic
        self.stiffness_msg = Float64MultiArray()
        self.stiffness = [0.0, 0.0, 0.0, 0.0]

        self.initial_contact = {finger: None for finger in ['index', 'middle', 'ring', 'thumb']}
        self.curr_contact = {finger: [0.0, 0.0, 0.0] for finger in ['index', 'middle', 'ring', 'thumb']}
        self.contact_flags = {finger: 0 for finger in ['index', 'middle', 'ring', 'thumb']}
        self.wrench_force_z = {finger: 0.0 for finger in ['index', 'middle', 'ring', 'thumb']}

    def crisp_pose_callback(self, msg):
        fingers = ['index', 'middle', 'ring', 'thumb']
        for i, finger in enumerate(fingers):
            # Convert positions from meters to centimeters
            self.curr_contact[finger] = [
                msg.poses[i].position.x * 100,
                msg.poses[i].position.y * 100,
                msg.poses[i].position.z * 100
            ]

            if self.wrench_force_z[finger] >= 30:
                if self.initial_contact[finger] is None:
                    self.initial_contact[finger] = self.curr_contact[finger][2]
                delta_z = self.curr_contact[finger][2] - self.initial_contact[finger]
                if delta_z != 0:
                    self.stiffness[i] = self.wrench_force_z[finger] / abs(delta_z)
                else:
                    self.stiffness[i] = 0.0
            else:
                self.initial_contact[finger] = None
                self.stiffness[i] = 0.0

        self.publish_stiffness_detection()

    def finger_callback(self, msg, finger):
        self.wrench_force_z[finger] = msg.wrench.force.z

    def publish_stiffness_detection(self):
        self.stiffness_msg.data = self.stiffness
        self.pub_stiffness_detection.publish(self.stiffness_msg)

if __name__ == '__main__':
    try:
        node = StiffnessDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
