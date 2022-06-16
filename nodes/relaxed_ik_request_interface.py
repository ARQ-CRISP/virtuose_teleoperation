#! /usr/bin/env python
from __future__ import print_function, division, absolute_import
import rospy
from virtuose_teleoperation.src.virtuose_teleoperation.arm.relaxed_ik_connection import IK_Solution_Manager

if __name__ == '__main__':
    rospy.init_node('ur_relaxed_connector')
    
    IK_Solution_Manager()
    
    rospy.spin()