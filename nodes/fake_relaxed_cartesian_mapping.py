#! /usr/bin/env python
from __future__ import print_function, division, absolute_import

import rospy
from virtuose_teleoperation.arm.FakeVirt_cartesian_mapping import FakeCartesian_Mapping

if __name__ == '__main__':
    rospy.init_node('FakeVirtuose_repub', anonymous=True)
    dummy = FakeCartesian_Mapping()
    rospy.spin()