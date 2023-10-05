#!/usr/bin/env python

# devo creare una istanza del workspace perche ad ora non esiste
# poi devo decommentare questa riga nel launch 
# <node pkg="virtuose_teleoperation" type="bounday_controller.py" name="boundary_controller" output="log">

from __future__ import print_function, division, absolute_import

import rospy
# from teleoperation_ur5_allegro_leap.teleop.ur5 import Leap_Teleop_UR5
from virtuose_teleoperation.arm.workspace_bounds import WS_Bounds

def update_workspace(connection):
    global workspace_bounds
    workspace_bounds = rospy.get_param('ur5_teleop_config/workspace/', default=workspace_bounds)
    connection.workspace_marker.pose.position.x = workspace_bounds['center'][0]
    connection.workspace_marker.pose.position.y = workspace_bounds['center'][1]
    connection.workspace_marker.pose.position.z = workspace_bounds['center'][2]
    
    connection.workspace_marker.scale.x = workspace_bounds['scale'][0]
    connection.workspace_marker.scale.y = workspace_bounds['scale'][1]
    connection.workspace_marker.scale.z = workspace_bounds['scale'][2]  

if __name__ == '__main__':

    rospy.init_node('workspace_bound')
    workspace_bounds = rospy.get_param('ur5_teleop_config/workspace/', default=None)
    if workspace_bounds is None:
        rospy.loginfo('[' + rospy.get_name() + ']' + ' No Workspace Bounds set')
        workspace_bounds = {'center': [0.0, 0.4, 1.280], 'scale': [1.2] * 3}
        rospy.set_param('ur5_teleop_config/workspace/', workspace_bounds)
    else:
        rospy.loginfo('[' + rospy.get_name() + ']' + ' workspace_config: {}'.format(workspace_bounds))
    
    workspace = WS_Bounds.from_center_scale(workspace_bounds['center'], workspace_bounds['scale'])
    
    rospy.spin()