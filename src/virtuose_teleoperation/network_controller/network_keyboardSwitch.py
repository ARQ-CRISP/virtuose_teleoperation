#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def keyboard_input_publisher():
    rospy.init_node('keyboard_input_node', anonymous=True)
    pub = rospy.Publisher('/network_switch', Int32, queue_size=10)

    rate = rospy.Rate(100)  # 100 Hz, adjust as needed

    while not rospy.is_shutdown():
        try:
            user_input = raw_input("Enter an integer: ")  # Use raw_input instead of input in Python 2
            user_input = int(user_input)

            if 0 <= user_input <= 10:
                pub.publish(user_input)
            else:
                rospy.logwarn("Input out of range. Please enter an integer between 0 and 10.")
        except ValueError:
            rospy.logwarn("Wrong input. Please enter a valid integer.")

        rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_input_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted.")
    except Exception as e:
        rospy.logerr("An unexpected error occurred: {}".format(str(e)))