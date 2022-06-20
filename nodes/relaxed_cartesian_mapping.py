#! /usr/bin/env python
from __future__ import print_function, division, absolute_import

import rospy
from virtuose_teleoperation.arm.cartesian_mapping import Cartesian_Mapping

if __name__ == '__main__':
    rospy.init_node('Virtuose_repub', anonymous=True)
    dummy = Cartesian_Mapping()
    rospy.spin()