#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, WrenchStamped,Point, Quaternion, Vector3
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
from virtuose_teleoperation.srv import SetPrior, AddPoint, ResetGP, CloneMesh
import math
import tf
import numpy as np
from visualization_msgs.msg import Marker

threshold1 = 80.0  # Example threshold value
stored_poses = []  # List to store the unique poses
first_pose_added = False  # Flag to track if the first pose has been added
first_prior_added = False
# Timer durations in seconds
GREEN_TIMER_DURATION = 40
BLUE_TIMER_DURATION = 120  # 2 minutes and 15 seconds

# Flag to track if the green timer is active
green_timer_active = False
# Flag to track if the blue timer is active
blue_timer_active = False

start_time_green = None
start_time_blue = None
green_timer_initialized = False
# Initialize countdown_marker_publisher as a global variable
countdown_marker_publisher = None
goal_marker_publisher = None
plane_marker_publisher = None
# Function to format the pose data with two decimal places

sphere_center2= Point()
sphere_center2.x = 0.14
sphere_center2.y = 0.38
sphere_center2.z = 1.01
radius2 = 0.125

def format_pose(pose):
    formatted_pose = pose
    formatted_pose.position.x = round(pose.position.x, 2)
    formatted_pose.position.y = round(pose.position.y, 2)
    formatted_pose.position.z = round(pose.position.z, 2)
    formatted_pose.orientation.x = round(pose.orientation.x, 2)
    formatted_pose.orientation.y = round(pose.orientation.y, 2)
    formatted_pose.orientation.z = round(pose.orientation.z, 2)
    formatted_pose.orientation.w = round(pose.orientation.w, 2)
    return formatted_pose

# Function to call the "add_point_gp_service" service
def call_add_point_service(x, y, z):
    rospy.wait_for_service('add_point_gp_service')
    try:
        # Create a proxy for the "add_point_gp_service" service
        add_point_gp_service = rospy.ServiceProxy('add_point_gp_service', AddPoint)
        # Call the service with the x, y, and z values
        response = add_point_gp_service(x, y, z)
        # Check the service response for success
        if response.succ:
            rospy.loginfo("Point added successfully.")
        else:
            rospy.logwarn("Failed to add point.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

# Function to call the "set_prior" service
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

# Function to publish the new pose
def publish_pose(pose):
    # Create a publisher for the pose
    pose_publisher = rospy.Publisher('new_pose', PoseArray, queue_size=10)
    pose_msg = PoseArray()
    pose_msg.poses.append(pose)
    # Publish the pose
    pose_publisher.publish(pose_msg)

# # Function to calculate the sphere center and radius (not implemented)
# def calculate_sphere(pose):
#     # Calculate the sphere center and radius based on the pose coordinates
#     # Replace this with your custom implementation
#     sphere_center = [pose.position.x, pose.position.y, pose.position.z]
#     radius = 1.0  # Example radius value
#     return sphere_center, radius

# Function to generate the sphere based on the touched point pose
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


# Function to publish the countdown marker
# Function to publish the countdown marker
def publish_countdown_marker(minutes, seconds, color, ns="countdown"):
    global countdown_marker_publisher
    if countdown_marker_publisher is None:
        rospy.logerr("Marker publisher not initialized.")
        return

    marker = Marker()
    marker.header.frame_id = "world"  # Replace "base_link" with the frame ID relevant to your application
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    #wrt base_link
    # marker.pose.position.x = 0.5
    # marker.pose.position.y = 0.0
    # marker.pose.position.z = -1.0  # Adjust the Z position based on where you want to display the countdown
    #wrt world
    marker.pose.position.x = -0.20
    marker.pose.position.y = 0.38
    marker.pose.position.z = 1.20  # Adjust the Z position based on where you want to display the countdown

    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    # Format the minutes and seconds as strings with leading zeros
    formatted_minutes = "{:02d}".format(minutes)
    formatted_seconds = "{:02d}".format(seconds)
    marker.text = "{}:{}".format(formatted_minutes, formatted_seconds)
    marker.lifetime = rospy.Duration(0)  # Keep the marker until replaced

    countdown_marker_publisher.publish(marker)

def publish_goal_marker():
    global goal_marker_publisher

    if goal_marker_publisher is None:
        print("goal_marker_publisher is None")
        return

    # Create the Marker message and set its properties (you can use your desired marker type and properties here)
    marker_goal = Marker()
    marker_goal.header.frame_id = "world"
    marker_goal.type = marker_goal.CUBE
    #Horizontal H
    marker_goal.scale.x = 0.055
    marker_goal.scale.y = 0.155
    marker_goal.scale.z = 0.08

    # #Vertical V
    # marker_goal.scale.x = 0.055
    # marker_goal.scale.y = 0.08
    # marker_goal.scale.z = 0.155

    # #BASE    B
    # marker_goal.scale.x = 0.08
    # marker_goal.scale.y = 0.155
    # marker_goal.scale.z = 0.055  



    marker_goal.pose.position.x = -0.07
    marker_goal.pose.position.y = 0.38
    marker_goal.pose.position.z = 1.01 + marker_goal.scale.z/2
    marker_goal.color.r = 0.0
    marker_goal.color.g = 1.0
    marker_goal.color.b = 0.0
    marker_goal.color.a = 0.5
    # marker_goal.lifetime = rospy.Duration.from_sec(60)
    # Publish the marker_goal
    
    goal_marker_publisher.publish(marker_goal)

def publish_plane_marker():

    global plane_marker_publisher
    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "world"  # Set the frame ID
        marker.type = Marker.TRIANGLE_LIST  # Set the marker type to triangle list

        # Set the scale of the triangles
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Set the color of the marker (white)
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        # Define the vertices of the triangles to simulate a plane
        center_x = -0.07
        center_y = 0.38
        center_z = 1.01

        # Define the vertex offsets to form the triangles
        vertices = [
            Point(-0.5, -0.5, 0),
            Point(0.5, -0.5, 0),
            Point(-0.5, 0.5, 0),
            Point(0.5, -0.5, 0),
            Point(0.5, 0.5, 0),
            Point(-0.5, 0.5, 0)
        ]

        # Apply the center position to the vertices
        for i in range(len(vertices)):
            vertices[i].x += center_x
            vertices[i].y += center_y
            vertices[i].z += center_z

        # Add the vertices to the marker's points array
        marker.points.extend(vertices)

        # Publish the marker
        plane_marker_publisher.publish(marker)

# Green timer callback function
def green_timer_callback(event):
    global GREEN_TIMER_DURATION
    global green_timer_active
    global start_time_green
    global green_timer_initialized
    global blue_timer_active
    global start_time_blue

    if not green_timer_initialized:
        green_timer = rospy.Timer(rospy.Duration(0.05), green_timer_callback)  # Start the green timer immediately
        green_timer_initialized = True

    if green_timer_active:
        if start_time_green is None:
            start_time_green = rospy.Time.now()
        
        elapsed_time = rospy.Time.now() - start_time_green
        remaining_time = max(0, GREEN_TIMER_DURATION - elapsed_time.to_sec())

        minutes = int(remaining_time) // 60
        seconds = int(remaining_time) % 60

        if remaining_time > 0:
            publish_countdown_marker(minutes, seconds, (0.0, 1.0, 0.0))  # Green color
            publish_goal_marker()

        else:
            green_timer_active = False
            start_time_blue = rospy.Time.now()  # Start the blue timer
            rospy.Timer(rospy.Duration(1), blue_timer_callback)  # Start the blue timer after 1 second
            # clone_response=clone_request()
            # if clone_response:
            #     rospy.loginfo("Clone request successful.")
            # else:
            #     rospy.loginfo("Clone request failed.")
            blue_timer_active = True

# Blue timer callback function
def blue_timer_callback(event):
    global BLUE_TIMER_DURATION
    global blue_timer_active
    global start_time_blue


    if blue_timer_active:
        if start_time_blue is None:
            start_time_blue = rospy.Time.now()

        elapsed_time = rospy.Time.now() - start_time_blue
        remaining_time = max(0, BLUE_TIMER_DURATION - elapsed_time.to_sec())

        minutes = int(remaining_time) // 60
        seconds = int(remaining_time) % 60

        if remaining_time > 0:
            publish_countdown_marker(minutes, seconds, (0.0, 0.0, 1.0))  # Blue color
            publish_goal_marker()

        else:
            blue_timer_active = False

def wrench_stamped_callback(msg):
    global stored_poses
    global first_pose_added
    global green_timer_active
    global blue_timer_active
    global sphere_center2,radius2
    global first_prior_added
    global plane_marker_publisher

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

    if blue_timer_active:
        # Here you can add any logic that you want to execute when the blue timer is active
        # For example, if you want to prevent any further processing, you can simply return here.
        return
    if not first_prior_added: 
        first_prior_added = True           
        call_set_prior_service(sphere_center2, radius2)
        publish_plane_marker()
    # The rest of the code should only execute when neither the green timer nor the blue timer is active
    if msg.wrench.force.z > threshold1:
        if link_poses is not None and len(link_poses.poses) > pose_index:
            finger_pose = format_pose(link_poses.poses[pose_index])
            distances = [np.linalg.norm(np.array([finger_pose.position.x, finger_pose.position.y, finger_pose.position.z]) - np.array([pose.position.x, pose.position.y, pose.position.z])) for pose in stored_poses]

            if not first_pose_added:
                first_pose_added = True
                pose_array = PoseArray()
                pose_array.poses = link_poses.poses
                sphere_center, radius = generate_sphere(pose_array, pose_index)
                # sphere_center.x = -0.07
                # sphere_center.y = 0.37
                # sphere_center.z = 1.08
                sphere_center.x = 0.14
                sphere_center.y = 0.38
                sphere_center.z = 1.01
                radius = 0.125
                print("sphere center, radius", sphere_center, radius)

                # Start the green timer (45 seconds)
                rospy.Timer(rospy.Duration(GREEN_TIMER_DURATION), green_timer_callback)
                green_timer_active = True

                # Call the set_prior service with the sphere parameters
                # call_set_prior_service(sphere_center, radius)

            if not distances or min(distances) > 0.01:
                stored_poses.append(finger_pose)
                rospy.loginfo("New {} pose: {}".format(finger_name, finger_pose.position))
                print("STORED POSES", len(stored_poses))

                # Call the add_point service only if the green timer is not active
                call_add_point_service(finger_pose.position.x, finger_pose.position.y, finger_pose.position.z)

                # Publish the new pose
                publish_pose(finger_pose)

        else:
            rospy.loginfo("Link poses not available.")

def reset_request():
    # Create a service proxy to call the 'reset_GP' service
    rospy.wait_for_service('reset_GP')  # Wait for the service to be available
    reset_service_proxy = rospy.ServiceProxy('reset_GP', ResetGP)

    try:
        # Call the 'reset_GP' service
        response = reset_service_proxy()
        return response

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        return None

def clone_request():
    # Create a service proxy to call the 'reset_GP' service
    rospy.wait_for_service('clone_mesh')  # Wait for the service to be available
    clone_service_proxy = rospy.ServiceProxy('clone_mesh', CloneMesh)

    try:
        # Call the 'reset_GP' service
        response = clone_service_proxy('mount_base')
        return response

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        return None
# Main function to initialize the ROS node

def main():
    global countdown_marker_publisher
    global goal_marker_publisher,plane_marker_publisher
        # Call the set_prior service with the sphere parameters
    global sphere_center
    global radius


    response = reset_request()  # Call the 'reset_GP' service

    if response:
        rospy.loginfo("Reset request successful.")
    else:
        rospy.loginfo("Reset request failed.")

    # print("sphere center, radius", sphere_center, radius)
    # call_set_prior_service(sphere_center, radius)
    


    print("newContact_detection Online")
    print(stored_poses)
    rospy.init_node('pointCloud_node')
    
    rospy.Timer(rospy.Duration(0.01), green_timer_callback)  # Start immediately
    rospy.Timer(rospy.Duration(0.01), blue_timer_callback)  # Initialize the blue timer but don't start it yet

    # Create a publisher for the countdown marker if not initialized
    if countdown_marker_publisher is None:
        countdown_marker_publisher = rospy.Publisher('countdown_marker', Marker, queue_size=1)

    # Create a publisher for the goal marker if not initialized
    if goal_marker_publisher is None:
        goal_marker_publisher = rospy.Publisher('goal_marker', Marker, queue_size=1)
    
    plane_marker_publisher = rospy.Publisher('plane_marker', Marker, queue_size=10)

    # # Publish the goal marker at position (x=0.0, y=0.5, z=0.5)
    #     rospy.sleep(1)
    #     publish_goal_marker(0.0, 0.5, 0.5)

    # Subscribe to the four topics publishing WrenchStamped messages
    rospy.Subscriber('/Crisp_TH_2HGlove', WrenchStamped, wrench_stamped_callback)
    rospy.Subscriber('/Crisp_IN_2HGlove', WrenchStamped, wrench_stamped_callback)
    rospy.Subscriber('/Crisp_MID_2HGlove', WrenchStamped, wrench_stamped_callback)
    rospy.Subscriber('/Crisp_RING_2HGlove', WrenchStamped, wrench_stamped_callback)

    
    rospy.spin()

if __name__ == '__main__':
    main()
