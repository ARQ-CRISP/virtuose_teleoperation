import torch
import numpy as np
from network import BC_MLP

class Policy():
    def __init__(self):
        self.BC_MLP = BC_MLP()
        self.BC_MLP.load_state_dict(torch.load("~/trained_policy_MLP/BC_epoch_3009.pth"))
    def output(self, ee_data, tactile_data, allegro_joints, allegro_velocity,  allegro_joints_eff):
        thumb_data = np.squeeze(tactile_data['thumb'])
        index_data = np.squeeze(tactile_data['index'])
        middle_data = np.squeeze(tactile_data['middle'])
        ring_data = np.squeeze(tactile_data['ring'])
        input = np.concatenate((ee_data, thumb_data, index_data, middle_data, ring_data, allegro_joints, allegro_velocity, allegro_joints_eff))
        out = self.BC_MLP(input)
        ee_position = out[:, 0:3]
        AH_joint_angle = out[:, 3:]
        return ee_position, AH_joint_angle