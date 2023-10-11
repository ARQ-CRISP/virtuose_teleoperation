import torch
import numpy as np
from network import BC_MLP, BC_MLP_EE
import rospy
import numpy as np

class Policy():
    def __init__(self):
        self.BC_MLP = BC_MLP()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\BC_epoch_3009.pth"))

    def output(self, ee_data, tactile_data, allegro_joints, allegro_velocity, allegro_joints_eff):
        # Need to change it into a cuda version
        thumb_data = np.reshape(tactile_data['thumb'], -1)
        index_data = np.reshape(tactile_data['index'], -1)
        middle_data = np.reshape(tactile_data['middle'], -1)
        ring_data = np.reshape(tactile_data['ring'], -1)
        input = np.concatenate((ee_data, thumb_data, index_data, middle_data, ring_data, allegro_joints, allegro_velocity, allegro_joints_eff))
        input = torch.from_numpy(input).float()
        out = self.BC_MLP(input)
        ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out[3:].detach().numpy()
        return ee_position, AH_joint_angle


class Policy_EE():
    def __init__(self):
        self.BC_MLP = BC_MLP_EE()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\BC_EE_epoch_237.pth"))

    def output(self, ee_data, tactile_data, allegro_joints, allegro_velocity, allegro_joints_eff):
        # Need to change it into a cuda version
        thumb_data = np.reshape(tactile_data['thumb'], -1)
        index_data = np.reshape(tactile_data['index'], -1)
        middle_data = np.reshape(tactile_data['middle'], -1)
        ring_data = np.reshape(tactile_data['ring'], -1)
        input = np.concatenate((ee_data, thumb_data, index_data, middle_data, ring_data, allegro_joints, allegro_velocity, allegro_joints_eff))
        input = torch.from_numpy(ee_data).float()
        out = self.BC_MLP(input)
        # ee_position = out[0:3].detach().numpy()
        # AH_joint_angle = out[3:].detach().numpy()
        return out
