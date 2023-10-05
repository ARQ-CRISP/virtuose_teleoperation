#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, WrenchStamped,Point, Quaternion, Vector3
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
from virtuose_teleoperation.srv import SetPrior, AddPoint
import math
import tf
import numpy as np
threshold1 = 80.0  # Example threshold value
stored_poses = []  # List to store the unique poses
first_pose_added = False  # Flag to track if the first pose has been added

def format_pose(pose):
    # Format the pose position with two decimal places
    # formatted_position = [format(p, '.2f') for p in pose.position]
    formatted_pose = pose
    formatted_pose.position.x = round(pose.position.x,2)
    formatted_pose.position.y = round(pose.position.y,2)
    formatted_pose.position.z = round(pose.position.z,2)
    formatted_pose.orientation.x = round(pose.orientation.x,2)
    formatted_pose.orientation.y = round(pose.orientation.y,2)
    formatted_pose.orientation.z = round(pose.orientation.z,2)
    formatted_pose.orientation.w = round(pose.orientation.w,2)

    # print("formatted",formatted_pose)
    return formatted_pose

def call_add_point_service(x, y, z):
    rospy.wait_for_service('add_point_gp_service')
    try:
        # Create a proxy for the "add_point_gp_service" service
        add_point_gp_service = rospy.ServiceProxy('add_point_gp_service', AddPoint)
        # Call the service with the x, y, and z values
        print(x,y,z)
        response = add_point_gp_service(x, y, z)
        # Check the service response for success
        if response.succ:
            rospy.loginfo("Point added successfully.")
        else:
            rospy.logwarn("Failed to add point.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))


def call_set_prior_service(sphere_center, radius):
    rospy.wait_for_service('set_prior')
    try:
        # Create a proxy for the "set_prior" service
        set_prior_service = rospy.ServiceProxy('set_prior', SetPrior)
        # Call the service with the SetPrior request
        response = set_prior_service(sphere_center.x, sphere_center.y, sphere_center.z, radius)
        return response.succ
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        return False


def publish_pose(pose):
    # Create a publisher for the pose
    pose_publisher = rospy.Publisher('new_pose', PoseArray, queue_size=10)
    pose_msg = PoseArray()
    pose_msg.poses.append(pose)
    # Publish the pose
    pose_publisher.publish(pose_msg)

# def calculate_sphere(pose):
#     # Calculate the sphere center and radius based on the pose coordinates
#     # Replace this with your custom implementation
#     sphere_center = [pose.position.x, pose.position.y, pose.position.z]
#     radius = 1.0  # Example radius value
#     return sphere_center, radius


def generate_sphere(pose_array,pose_index):#, z_table):
    # Extract the touched point's pose from the PoseArray
    touched_point_pose = pose_array.poses[pose_index]

    # Create a Quaternion object to represent the finger pose orientation
    finger_pose_orientation = touched_point_pose.orientation


    # Create a Vector3 object to store the direction vector
    direction_vector = Vector3()

    # Convert the Quaternion to Euler angles
    euler_angles = tf.transformations.euler_from_quaternion([finger_pose_orientation.x, finger_pose_orientation.y,
                                                             finger_pose_orientation.z, finger_pose_orientation.w])
    # print("Position is", touched_point_pose)
    # print("Quaternion is", finger_pose_orientation)
    # print("Euler angles...", euler_angles)

    # Extract the roll, pitch, and yaw angles from the Euler angles
    roll, pitch, yaw = euler_angles

    # Calculate the direction vector
    direction_vector.x = math.cos(pitch) * math.cos(yaw)
    direction_vector.y = math.cos(pitch) * math.sin(yaw)
    direction_vector.z = math.sin(pitch)

    # print("Direciton vector is then", direction_vector)

    # Normalize the direction vector to obtain a unit vector
    norm = math.sqrt(direction_vector.x**2 + direction_vector.y**2 + direction_vector.z**2)
    direction_vector.x /= norm
    direction_vector.y /= norm
    direction_vector.z /= norm

    # Calculate the line equation parameters
    line_point = Point(x=touched_point_pose.position.x, y=touched_point_pose.position.y,
                       z=touched_point_pose.position.z)
    line_direction = Vector3(x=direction_vector.x, y=direction_vector.y, z=direction_vector.z)
    # print("line_point",line_point)
    # print("line_direction",line_direction)

    # Calculate the intersection point with the table
    table_height = 1.2 #setup pos 1.2 , paper box 1.08 , table 1.00
    t = (table_height - line_point.z) / line_direction.z
    intersection_point = Point()
    intersection_point.x = line_point.x + t * line_direction.x
    intersection_point.y = line_point.y + t * line_direction.y
    intersection_point.z = table_height

    # Calculate the Euclidean distance between the intersection point and the touched point
    distance = math.sqrt((intersection_point.x - line_point.x)**2 +
                         (intersection_point.y - line_point.y)**2 +
                         (intersection_point.z - line_point.z)**2)

    # Check if line_direction.z > 0
    if line_direction.z > 0:
        # Calculate the position of the point on the line with a distance of 0.2 and same z as touched point
        scale_factor = 0.20 / math.sqrt(line_direction.x**2 + line_direction.y**2)
        point_on_line = Point()
        point_on_line.x = round(line_point.x + scale_factor * line_direction.x,2)
        point_on_line.y = round(line_point.y + scale_factor * line_direction.y,2)
        # print("point_on_line.x",point_on_line.x)
        # print("point_on_line.y",point_on_line.y)
        
        point_on_line.z = round(table_height,2) #line_point.z

        # Calculate the distance between the new point and the touched point
        distance = round(math.sqrt((point_on_line.x - line_point.x)**2 +
                             (point_on_line.y - line_point.y)**2 +
                             (point_on_line.z - line_point.z)**2),2)

        return point_on_line, distance

    # Check if the distance is larger than 0.20
    if distance > 0.20:
        # Calculate the position of the point on the line with a distance of 0.20
        scale_factor = 0.20 / distance
        point_on_line = Point()
        point_on_line.x = line_point.x + scale_factor * (intersection_point.x - line_point.x)
        point_on_line.y = line_point.y + scale_factor * (intersection_point.y - line_point.y)
        point_on_line.z = table_height

        # Calculate the distance between the new point and the touched point
        distance = math.sqrt((point_on_line.x - line_point.x)**2 +
                             (point_on_line.y - line_point.y)**2 +
                             (point_on_line.z - line_point.z)**2)
        # print ("distance1",distance)
        return point_on_line, distance
   
    # print ("distance2",distance)

    return intersection_point, distance



def wrench_stamped_callback(msg):
    global stored_poses
    global first_pose_added

    # Wait for the link_poses message crisp_merged_poses it's a combination used for merging the orientation of the last joint of each fingertip
    # with the Cartesian position of tip of optoforce(this to match approximately the Crisp fingertip center)
    link_poses = rospy.wait_for_message('crisp_merged_poses', PoseArray)
    # print("link_poses", link_poses)
    # Check the threshold for each finger separately
    if msg.header.frame_id == "/Crisp_TH_2HGlove":
        finger_name = "thumb"
        pose_index = 3
    elif msg.header.frame_id == "/Crisp_IN_2HGlove":
        finger_name = "index"
        pose_index = 0
    elif msg.header.frame_id == "/Crisp_MID_2HGlove":
        finger_name = "middle"
        pose_index = 1
    elif msg.header.frame_id == "/Crisp_RING_2HGlove":
        finger_name = "ring"
        pose_index = 2
    else:
        rospy.logwarn("Unknown finger: {}".format(msg.header.frame_id))
        return

    if msg.wrench.force.z > threshold1:
        if link_poses is not None and len(link_poses.poses) > pose_index:
            # print("pose_index",pose_index)
            # print("finger_name",finger_name)
            finger_pose = format_pose(link_poses.poses[pose_index])
            # print("figner_pose",finger_pose)
            # Calculate Euclidean distance between finger_pose.position and stored poses
            distances = [np.linalg.norm(np.array([finger_pose.position.x, finger_pose.position.y, finger_pose.position.z]) - np.array([pose.position.x, pose.position.y, pose.position.z])) for pose in stored_poses]
            # print(len(stored_poses))
            
            if not first_pose_added:
                first_pose_added = True
                # Calculate the sphere parameters
                pose_array = PoseArray()
                pose_array.poses = link_poses.poses
                sphere_center, radius = generate_sphere(pose_array,pose_index)
                sphere_center.x=-0.07
                sphere_center.y=0.37
                sphere_center.z=1.08
                radius=0.15
                print("sphere center, radius",sphere_center, radius)
                # Call the set_prior service with the sphere parameters
                call_set_prior_service(sphere_center, radius)            

            # Check if the minimum distance is greater than 1 cm (0.01 m)
            if not distances or min(distances) > 0.01:                
                stored_poses.append(finger_pose)
                # print(stored_poses[-1])
                rospy.loginfo("New {} pose: {}".format(finger_name, finger_pose.position))
                print("STORED POSES",len(stored_poses))

                # point = Float64MultiArray()
                # point.data = [finger_pose.position.x, finger_pose.position.y, finger_pose.position.z]
                # print("point.data[0], point.data[1], point.data[2]",point.data[0], point.data[1], point.data[2])
                call_add_point_service(finger_pose.position.x, finger_pose.position.y, finger_pose.position.z)

                # Publish the new pose
                publish_pose(finger_pose)
            
            
                # if not first_pose_added:
                #     first_pose_added = True

                #     # Calculate the sphere parameters
                #     pose_array = PoseArray()
                #     pose_array.poses = link_poses.poses
                #     sphere_center, radius = generate_sphere(pose_array)
                #     # print("sphere center, radius",sphere_center, radius)
                #     # Call the set_prior service with the sphere parameters
                #     sphere_center.x=-0.07
                #     sphere_center.y=0.37
                #     sphere_center.z=1.08
                #     radius=0.10
                #     print("sphere center, radius",sphere_center,radius)
                #     call_set_prior_service(sphere_center, radius)
                    
        else:
            rospy.loginfo("Link poses not available.")
def main():
    print("newContact_detection Online")
    print(stored_poses)
    rospy.init_node('pointCloud_node')
    # Subscribe to the four topics publishing WrenchStamped messages
    rospy.Subscriber('/Crisp_TH_2HGlove', WrenchStamped, wrench_stamped_callback)
    rospy.Subscriber('/Crisp_IN_2HGlove', WrenchStamped, wrench_stamped_callback)
    rospy.Subscriber('/Crisp_MID_2HGlove', WrenchStamped, wrench_stamped_callback)
    rospy.Subscriber('/Crisp_RING_2HGlove', WrenchStamped, wrench_stamped_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
