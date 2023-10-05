#! /usr/bin/env python
from __future__ import print_function, division, absolute_import
import numpy as np

import rospy
import yaml
from rospkg import RosPack

from virtuose_teleoperation.arm.relaxed_ik_connection import IK_Solution_Manager







if __name__ == '__main__':

    rospy.init_node('ur_relaxed_connector')
    relaxed_ik_path = RosPack().get_path('relaxed_ik')
    relaxed_yaml_filename = rospy.get_param('~relaxed_ik_yaml', default='ur5_allegro_info_VIRTUOSE.yaml')

    workspace_bounds = rospy.get_param('ur5_teleop_config/workspace/', default=None) # HAVE A LOOK AT WS_Bounds Class
    is_sim = rospy.get_param('~simulated', default=True)
    
    rospy.loginfo('[' + rospy.get_name() + ']' + ' ik config_file: {}'.format(relaxed_yaml_filename))
    
        
    if is_sim:
        rospy.loginfo('[' + rospy.get_name() + ']' + ' Teleoperating in a simulated environment!')
    else:
        rospy.loginfo('[' + rospy.get_name() + ']' + ' Teleoperating on the real robot!')
    yaml_file_path = relaxed_ik_path + '/src/' + '/RelaxedIK/Config/info_files/' + relaxed_yaml_filename
    with open(yaml_file_path) as f:
        yaml_file = yaml.load(f, Loader=yaml.loader.FullLoader)




    # connection = Relaxed_UR5_Connection(
    #     init_state=yaml_file['starting_config'],
    #     movegroup='ur5_arm',
    #     sim=is_sim, debug_mode=True)
    init_state = yaml_file['starting_config']
    print('init_state',init_state)
    solution_manager = IK_Solution_Manager(
        init_state,
        sim=is_sim
        # movegroup='ur5_arm' ,
        # debug_mode=True
        ) 
    
    # print('this is the current state: ', solution_manager.current_joint_state)
    
    rospy.spin()