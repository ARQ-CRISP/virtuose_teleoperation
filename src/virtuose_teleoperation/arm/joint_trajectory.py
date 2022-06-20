from __future__ import print_function, division, absolute_import
from abc import ABCMeta, abstractmethod
from collections import namedtuple

from sensor_msgs.msg import JointState
from relaxed_ik.msg import EEPoseGoals, JointAngles
import rospy

import numpy as np




class JointTrajectoryManager:
    __metaclass__ = ABCMeta
    
    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    
    ur_state = namedtuple('UR_State', ('position', 'velocity', 'effort'))