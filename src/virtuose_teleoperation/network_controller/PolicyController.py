import torch
import numpy as np
from network import BC_MLP
import rospy

class Policy():
    def __init__(self):
        self.BC_MLP = BC_MLP()
        self.BC_MLP.load_state_dict(torch.load("~/trained_policy_MLP/BC_epoch_3009.pth"))
    
    def output(self, ee_data, tactile_data, allegro_data):
        thumb_data = np.squeeze(tactile_data['thumb'])
        index_data = np.squeeze(tactile_data['index'])
        middle_data = np.squeeze(tactile_data['middle'])
        ring_data = np.squeeze(tactile_data['ring'])
        input = np.concatenate((ee_data, thumb_data, index_data, middle_data, ring_data, allegro_joints, allegro_velocity, allegro_joints_eff))
        out = self.BC_MLP(input)
        ee_position = out[:, 0:3]
        AH_joint_angle = out[:, 3:]
        return ee_position, AH_joint_angle


# class ROSPolicyNode():
#     def __init__(self):
#         # Initialize your BC_MLP model and other variables here
#         self.policy = Policy()

#         rospy.Subscriber('ee_data', ee_data, self.callback)  # Adjust input message type
#         rospy.Subscriber('tactile_data', tactile_data, self.callback)  # Adjust input message type
#         rospy.Subscriber('allegro_data', allegro_data, self.callback)  # Adjust input message type

#         self.pub = rospy.Publisher('ee_position', ee_position, queue_size=10) 
#         self.pub = rospy.Publisher('AH_joint_angle', AH_joint_angle, queue_size=10)  

#     def callback(self, msg):
#         # Extract data from the received message if necessary
#         ee_data = msg.ee_data
#         tactile_data = msg.tactile_data
#         allegro_joints = msg.allegro_joints
#         allegro_velocity = msg.allegro_velocity
#         allegro_joints_eff = msg.allegro_joints_eff

#         # Call your output function
#         ee_position, AH_joint_angle = self.policy.output(ee_data, tactile_data, allegro_joints, allegro_velocity, allegro_joints_eff)

#         # Create a new message for the output data
#         output_msg = YourCustomMsg()
#         output_msg.ee_position = ee_position
#         output_msg.AH_joint_angle = AH_joint_angle

#         # Publish the output
#         self.pub.publish(output_msg)

# if __name__ == '__main__':
#     rospy.init_node('policy_node')
#     node = ROSPolicyNode()
#     rospy.spin()
