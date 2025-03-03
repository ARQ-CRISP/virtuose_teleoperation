#!/usr/bin/env python2
import rosbag
import os
import pandas as pd
from geometry_msgs.msg import PoseArray, WrenchStamped
from std_msgs.msg import Float64MultiArray

def extract_data_from_rosbag(bag_file, label):
    bag = rosbag.Bag(bag_file)
    
    data = {
        'index_position_x': [], 'index_position_y': [], 'index_position_z': [],
        'index_orientation_x': [], 'index_orientation_y': [], 'index_orientation_z': [], 'index_orientation_w': [],
        'middle_position_x': [], 'middle_position_y': [], 'middle_position_z': [],
        'middle_orientation_x': [], 'middle_orientation_y': [], 'middle_orientation_z': [], 'middle_orientation_w': [],
        'ring_position_x': [], 'ring_position_y': [], 'ring_position_z': [],
        'ring_orientation_x': [], 'ring_orientation_y': [], 'ring_orientation_z': [], 'ring_orientation_w': [],
        'thumb_position_x': [], 'thumb_position_y': [], 'thumb_position_z': [],
        'thumb_orientation_x': [], 'thumb_orientation_y': [], 'thumb_orientation_z': [], 'thumb_orientation_w': [],
        'force_x_index': [], 'force_y_index': [], 'force_z_index': [],
        'force_x_middle': [], 'force_y_middle': [], 'force_z_middle': [],
        'force_x_ring': [], 'force_y_ring': [], 'force_z_ring': [],
        'force_x_thumb': [], 'force_y_thumb': [], 'force_z_thumb': [],
        'torque_norm_index': [], 'torque_norm_middle': [], 'torque_norm_ring': [], 'torque_norm_thumb': [],
        'stiffness_index': [], 'stiffness_middle': [], 'stiffness_ring': [], 'stiffness_thumb': []
    }

    # Get the total number of messages
    message_count = bag.get_message_count(topic_filters=['/crisp_merged_poses', '/Crisp_IN_2HGlove', '/Crisp_MID_2HGlove', '/Crisp_RING_2HGlove', '/Crisp_TH_2HGlove', '/stiffness_detection'])
    
    # Calculate the number of messages to skip (50% at the start and 10% at the end)
    skip_start = int(message_count * 0.5)
    skip_end = int(message_count * 0.1)
    current_count = 0

    # Last seen values to fill missing data
    last_seen = {
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

    for topic, msg, t in bag.read_messages(topics=['/crisp_merged_poses', '/Crisp_IN_2HGlove', '/Crisp_MID_2HGlove', '/Crisp_RING_2HGlove', '/Crisp_TH_2HGlove', '/stiffness_detection']):
        current_count += 1
        if current_count <= skip_start or current_count > message_count - skip_end:
            continue
        
        if topic == '/crisp_merged_poses':
            for i, pose in enumerate(msg.poses):
                finger = ['index', 'middle', 'ring', 'thumb'][i]
                data['{}_position_x'.format(finger)].append(pose.position.x)
                data['{}_position_y'.format(finger)].append(pose.position.y)
                data['{}_position_z'.format(finger)].append(pose.position.z)
                data['{}_orientation_x'.format(finger)].append(pose.orientation.x)
                data['{}_orientation_y'.format(finger)].append(pose.orientation.y)
                data['{}_orientation_z'.format(finger)].append(pose.orientation.z)
                data['{}_orientation_w'.format(finger)].append(pose.orientation.w)

                last_seen['{}_position_x'.format(finger)] = pose.position.x
                last_seen['{}_position_y'.format(finger)] = pose.position.y
                last_seen['{}_position_z'.format(finger)] = pose.position.z
                last_seen['{}_orientation_x'.format(finger)] = pose.orientation.x
                last_seen['{}_orientation_y'.format(finger)] = pose.orientation.y
                last_seen['{}_orientation_z'.format(finger)] = pose.orientation.z
                last_seen['{}_orientation_w'.format(finger)] = pose.orientation.w

        elif topic == '/Crisp_IN_2HGlove':
            finger = 'index'
        elif topic == '/Crisp_MID_2HGlove':
            finger = 'middle'
        elif topic == '/Crisp_RING_2HGlove':
            finger = 'ring'
        elif topic == '/Crisp_TH_2HGlove':
            finger = 'thumb'
        
        if topic in ['/Crisp_IN_2HGlove', '/Crisp_MID_2HGlove', '/Crisp_RING_2HGlove', '/Crisp_TH_2HGlove']:
            data['force_x_{}'.format(finger)].append(msg.wrench.force.x)
            data['force_y_{}'.format(finger)].append(msg.wrench.force.y)
            data['force_z_{}'.format(finger)].append(msg.wrench.force.z)
            data['torque_norm_{}'.format(finger)].append(msg.wrench.torque.z)

            last_seen['force_x_{}'.format(finger)] = msg.wrench.force.x
            last_seen['force_y_{}'.format(finger)] = msg.wrench.force.y
            last_seen['force_z_{}'.format(finger)] = msg.wrench.force.z
            last_seen['torque_norm_{}'.format(finger)] = msg.wrench.torque.z

        elif topic == '/stiffness_detection':
            data['stiffness_index'].append(msg.data[0])
            data['stiffness_middle'].append(msg.data[1])
            data['stiffness_ring'].append(msg.data[2])
            data['stiffness_thumb'].append(msg.data[3])

            last_seen['stiffness_index'] = msg.data[0]
            last_seen['stiffness_middle'] = msg.data[1]
            last_seen['stiffness_ring'] = msg.data[2]
            last_seen['stiffness_thumb'] = msg.data[3]

    bag.close()
    
    # Fill any remaining empty values with the last seen values
    for key in data.keys():
        while len(data[key]) < (message_count - skip_start - skip_end):
            data[key].append(last_seen[key])
    
    return pd.DataFrame(data)

def save_demonstrations(base_path):
    subfolders = ['brick', 'no_contact', 'rice', 'tom', 'us', 'ys']
    for subfolder in subfolders:
        subfolder_path = os.path.join(base_path, subfolder)
        output_folder = os.path.join(base_path, 'processed', subfolder)
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
        if os.path.isdir(subfolder_path):
            for bag_file in os.listdir(subfolder_path):
                if bag_file.endswith('.bag'):
                    label = subfolder  # Label is the name of the subfolder
                    bag_file_path = os.path.join(subfolder_path, bag_file)
                    df = extract_data_from_rosbag(bag_file_path, label)
                    output_file = os.path.join(output_folder, os.path.splitext(bag_file)[0] + '.csv')
                    df.to_csv(output_file, index=False)

# Usage example
base_path = '/home/hair/stiffness_estimation'
save_demonstrations(base_path)
