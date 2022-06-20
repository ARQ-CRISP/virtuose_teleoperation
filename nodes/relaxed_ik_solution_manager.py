#! /usr/bin/env python
from __future__ import print_function, division, absolute_import
import rospy
from virtuose_teleoperation.arm.relaxed_ik_connection import IK_Solution_Manager

if __name__ == '__main__':
    rospy.init_node('ur_relaxed_connector')
    
    solution_manager = IK_Solution_Manager()
    
    print('this is the current state: ', solution_manager.current_joint_state)
    
    rospy.spin()