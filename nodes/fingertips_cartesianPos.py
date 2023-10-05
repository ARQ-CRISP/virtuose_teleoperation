#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

def forward_kinematics_node():
    rospy.init_node('forward_kinematics_node')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # Publisher for the poses of link 3, 7, 11, and 15
    pub_Tip = rospy.Publisher('link_crispTip_poses', geometry_msgs.msg.PoseArray, queue_size=10)
    pub_Link = rospy.Publisher('link_poses', geometry_msgs.msg.PoseArray, queue_size=10)
    pub_Merged = rospy.Publisher('crisp_merged_poses', geometry_msgs.msg.PoseArray, queue_size=10)

    # Publisher for the poses of link 3, 7, 11, and 15
        
    rate = rospy.Rate(100.0)  # Update rate 
    
    while not rospy.is_shutdown():
        try:
            ##GET AN ESTIMATION OF THE CENTER OF THE TIP WHERE THE CONTACT POINT IS SUPPOSED TO BE 

            # Get the transform from "world" to link_tip=center of Crisp
            transform_3_tip = tf_buffer.lookup_transform("world", "link_3_tip", rospy.Time(0), rospy.Duration(1.0))
            transform_7_tip = tf_buffer.lookup_transform("world", "link_7_tip", rospy.Time(0), rospy.Duration(1.0))
            transform_11_tip = tf_buffer.lookup_transform("world", "link_11_tip", rospy.Time(0), rospy.Duration(1.0))
            transform_15_tip = tf_buffer.lookup_transform("world", "link_15_tip", rospy.Time(0), rospy.Duration(1.0))
            
            # Create a PoseArray message to store the poses
            poses_msg_tip = geometry_msgs.msg.PoseArray()
            poses_msg_tip.header.stamp = rospy.Time.now()
            poses_msg_tip.header.frame_id = "world-index-middle-ring-thumb TIP"
            
            # Create Pose objects for each link
            pose_3_tip = geometry_msgs.msg.Pose()
            pose_3_tip.position = transform_3_tip.transform.translation
            pose_3_tip.orientation = transform_3_tip.transform.rotation
            
            pose_7_tip = geometry_msgs.msg.Pose()
            pose_7_tip.position = transform_7_tip.transform.translation
            pose_7_tip.orientation = transform_7_tip.transform.rotation
            
            pose_11_tip = geometry_msgs.msg.Pose()
            pose_11_tip.position = transform_11_tip.transform.translation
            pose_11_tip.orientation = transform_11_tip.transform.rotation
            
            pose_15_tip = geometry_msgs.msg.Pose()
            pose_15_tip.position = transform_15_tip.transform.translation
            pose_15_tip.orientation = transform_15_tip.transform.rotation
            
            # Assign the poses to the PoseArray
            poses_msg_tip.poses = [pose_3_tip, pose_7_tip, pose_11_tip, pose_15_tip]
            
            ##CREATE THE SAME FOR THE LINK EE at the end of the fingers(wihtout the tip)
            ## USED TO ESTIMATE THE ORIENTATION OF THE TIP

            # Get the transform from "world" to link_tip=center of Crisp
            transform_3_link = tf_buffer.lookup_transform("world", "link_3", rospy.Time(0), rospy.Duration(1.0))
            transform_7_link = tf_buffer.lookup_transform("world", "link_7", rospy.Time(0), rospy.Duration(1.0))
            transform_11_link = tf_buffer.lookup_transform("world", "link_11", rospy.Time(0), rospy.Duration(1.0))
            transform_15_link = tf_buffer.lookup_transform("world", "link_15", rospy.Time(0), rospy.Duration(1.0))
            
            # Create a PoseArray message to store the poses
            poses_msg_link = geometry_msgs.msg.PoseArray()
            poses_msg_link.header.stamp = rospy.Time.now()
            poses_msg_link.header.frame_id = "world-index-middle-ring-thumb link"
            
            # Create Pose objects for each link
            pose_3_link = geometry_msgs.msg.Pose()
            pose_3_link.position = transform_3_link.transform.translation
            pose_3_link.orientation = transform_3_link.transform.rotation
            
            pose_7_link = geometry_msgs.msg.Pose()
            pose_7_link.position = transform_7_link.transform.translation
            pose_7_link.orientation = transform_7_link.transform.rotation
            
            pose_11_link = geometry_msgs.msg.Pose()
            pose_11_link.position = transform_11_link.transform.translation
            pose_11_link.orientation = transform_11_link.transform.rotation
            
            pose_15_link = geometry_msgs.msg.Pose()
            pose_15_link.position = transform_15_link.transform.translation
            pose_15_link.orientation = transform_15_link.transform.rotation
            
            # Assign the poses to the PoseArray
            poses_msg_link.poses = [pose_3_link, pose_7_link, pose_11_link, pose_15_link]


            # Create a PoseArray message to store the poses
            merged_msg_linkTip = geometry_msgs.msg.PoseArray()
            merged_msg_linkTip.header.stamp = rospy.Time.now()
            merged_msg_linkTip.header.frame_id = "MERGED world-index-middle-ring-thumb tip-link"
            
            # Create a PoseArray message to store the poses
            poses_msg_merged = geometry_msgs.msg.PoseArray()
            poses_msg_merged.header.stamp = rospy.Time.now()
            poses_msg_merged.header.frame_id = "world-index-middle-ring-thumb merged"
            
            # Create Pose objects for each merged
            pose_3_merged = geometry_msgs.msg.Pose()
            pose_3_merged.position = transform_3_tip.transform.translation
            pose_3_merged.orientation = transform_3_link.transform.rotation
            
            pose_7_merged = geometry_msgs.msg.Pose()
            pose_7_merged.position = transform_7_tip.transform.translation
            pose_7_merged.orientation = transform_7_link.transform.rotation
            
            pose_11_merged = geometry_msgs.msg.Pose()
            pose_11_merged.position = transform_11_tip.transform.translation
            pose_11_merged.orientation = transform_11_link.transform.rotation
            
            pose_15_merged = geometry_msgs.msg.Pose()
            pose_15_merged.position = transform_15_tip.transform.translation
            pose_15_merged.orientation = transform_15_link.transform.rotation
            
            # Assign the poses to the PoseArray
            poses_msg_merged.poses = [pose_3_merged, pose_7_merged, pose_11_merged, pose_15_merged]
           

            # Publish the poses messages
            # pub_Tip.publish(poses_msg_tip)
            # pub_Link.publish(poses_msg_link)
            pub_Merged.publish(poses_msg_merged) 

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed!")
        
        rate.sleep()

if __name__ == '__main__':
    print("Kinematics Online")
    forward_kinematics_node()
