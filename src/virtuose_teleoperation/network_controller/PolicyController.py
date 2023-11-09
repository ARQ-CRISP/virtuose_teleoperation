import torch
import numpy as np
from network import BC_MLP, BC_MLP_EE, BC_MLP_AHEE, BC_MLP_AHEE_yes_vel_no_eff, BC_MLP_AHEE_no_vel_no_eff, BC_MLP_AHEE_yVel_nEff_yTact, BC_MLP_AHEE_no_eff_yes_ff, BC_MLP_AHEE_yes_norm, BC_MLP_AHEE_yes_norm_ouput_delta_q
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
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\BC_EE_epoch_587.pth"))

    def output(self, ee_data, tactile_data, allegro_joints, allegro_velocity, allegro_joints_eff):
        # Need to change it into a cuda version
        thumb_data = np.reshape(tactile_data['thumb'], -1)
        index_data = np.reshape(tactile_data['index'], -1)
        middle_data = np.reshape(tactile_data['middle'], -1)
        ring_data = np.reshape(tactile_data['ring'], -1)
        input = np.array((ee_data))
        input = torch.from_numpy(input).float()
        out = self.BC_MLP(input).detach().numpy()
        # ee_position = out[0:3].detach().numpy()
        # AH_joint_angle = out[3:].detach().numpy()
        return out

class PolicyBC_MLP_AHEE_yVel_nEff_yTact():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yVel_nEff_yTact()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasp_BC_AH_epoch_3009.pth"))

    def output(self, ee_data, tactile_data, allegro_joints, allegro_velocity):
        # Need to change it into a cuda version
        thumb_data = np.reshape(tactile_data['thumb'], -1)
        index_data = np.reshape(tactile_data['index'], -1)
        middle_data = np.reshape(tactile_data['middle'], -1)
        ring_data = np.reshape(tactile_data['ring'], -1)
        input = np.concatenate((ee_data, thumb_data, index_data, middle_data, ring_data, allegro_joints, allegro_velocity))

        input = torch.from_numpy(input).float()
        out = self.BC_MLP(input)
        ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out[3:].detach().numpy()
        return ee_position, AH_joint_angle

class PolicyAHEE():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\BC_AHEE_epoch_199.pth"))

    def output(self, ee_data, allegro_joints, allegro_velocity, allegro_joints_eff):
        # Need to change it into a cuda version
        input = np.concatenate((ee_data, allegro_joints, allegro_velocity, allegro_joints_eff))
        input = torch.from_numpy(input).float()
        out = self.BC_MLP(input)
        ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out[3:].detach().numpy()
        return ee_position, AH_joint_angle


   
class PolicyAHEE_yes_vel_no_eff():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_vel_no_eff()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Touch_BC_AHEE_epoch_188_yes_vel_no_eff.pth"))

    def output(self, ee_data, allegro_joints, allegro_velocity):
        # Need to change it into a cuda version
        input = np.concatenate((ee_data, allegro_joints, allegro_velocity))
        input = torch.from_numpy(input).float()
        out = self.BC_MLP(input)
        ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out[3:].detach().numpy()
        return ee_position, AH_joint_angle

    
class PolicyAHEE_no_vel_no_eff():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_no_vel_no_eff()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\BC_AHEE_joint_epoch_299.pth"))

    def output(self, ee_data, allegro_joints):
        # Need to change it into a cuda version
        input = np.concatenate((ee_data, allegro_joints))
        input = torch.from_numpy(input).float()
        out = self.BC_MLP(input)
        ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out[3:].detach().numpy()
        return ee_position, AH_joint_angle


    
class PolicyAHEE_no_eff_yes_ff():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_no_eff_yes_ff()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasp_BC_AHEE_epoch_293.pth"))

    def output(self, ee_data, allegro_joints, allegro_velocity, ff_data):
        # Need to change it into a cuda version
        thumb_ff = ff_data['thumb']
        index_ff = ff_data['index']
        middle_ff = ff_data['middle']
        ring_ff = ff_data['ring']

        input = np.concatenate((ee_data, allegro_joints, allegro_velocity, [index_ff[-1]], [middle_ff[-1]], [ring_ff[-1]], [thumb_ff[-1]]))
        input = torch.from_numpy(input).float()
        out = self.BC_MLP(input)
        ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out[3:].detach().numpy()
        return ee_position, AH_joint_angle


    
class PolicyAHEE_yes_norm():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_norm()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasp_norm_BC_AHEE_epoch_834.pth"))

    def output(self, ee_data, allegro_joints, tactile_data):
        # Norm
        thumb_data = tactile_data['thumb']
        index_data = tactile_data['index']
        middle_data = tactile_data['middle']
        ring_data = tactile_data['ring']
        thumb_norm = np.max(np.linalg.norm(thumb_data, axis=1))
        index_norm = np.max(np.linalg.norm(index_data, axis=1))
        middle_norm = np.max(np.linalg.norm(middle_data, axis=1))
        ring_norm = np.max(np.linalg.norm(ring_data, axis=1))
        # Limit the value
        limit = 20
        thumb_norm = np.where(thumb_norm < limit, 0, thumb_norm)
        index_norm = np.where(index_norm < limit, 0, index_norm)
        middle_norm = np.where(middle_norm < limit, 0, middle_norm)
        ring_norm = np.where(ring_norm < limit, 0, ring_norm)
        input = np.concatenate((ee_data, allegro_joints, [thumb_norm], [ring_norm], [middle_norm], [index_norm] ))

        input = torch.from_numpy(input).float()
        out = self.BC_MLP(input)
        ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out[3:].detach().numpy()
        return ee_position, AH_joint_angle
    
    
class PolicyAHEE_yes_norm_delta_q():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_norm()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasp_norm_BC_AHEE_delta_q_epoch_42.pth"))

    def output(self, ee_data, allegro_joints, tactile_data):
        # Norm
        thumb_data = tactile_data['thumb']
        index_data = tactile_data['index']
        middle_data = tactile_data['middle']
        ring_data = tactile_data['ring']
        thumb_norm = np.max(np.linalg.norm(thumb_data, axis=1))
        index_norm = np.max(np.linalg.norm(index_data, axis=1))
        middle_norm = np.max(np.linalg.norm(middle_data, axis=1))
        ring_norm = np.max(np.linalg.norm(ring_data, axis=1))
        # Limit the value
        limit = 20
        thumb_norm = np.where(thumb_norm < limit, 0, thumb_norm)
        index_norm = np.where(index_norm < limit, 0, index_norm)
        middle_norm = np.where(middle_norm < limit, 0, middle_norm)
        ring_norm = np.where(ring_norm < limit, 0, ring_norm)
        input = np.concatenate((ee_data, allegro_joints, [thumb_norm], [ring_norm], [middle_norm], [index_norm] ))

        input = torch.from_numpy(input).float()
        out = self.BC_MLP(input)
        # ee_position = out[0:3].detach().numpy() + ee_data[0:3]
        # AH_joint_angle = out[3:].detach().numpy() + allegro_joints
        ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out[3:].detach().numpy()
        return ee_position, AH_joint_angle
    
class PolicyAH_yes_norm_delta_q():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_norm()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasp_norm_BC_AHEE_epoch_58_latest.pth"))

    def output(self, allegro_joints, allegro_joints_delta, tactile_data):
        # Norm
        thumb_data = tactile_data['thumb']
        index_data = tactile_data['index']
        middle_data = tactile_data['middle']
        ring_data = tactile_data['ring']
        thumb_norm = np.max(np.linalg.norm(thumb_data, axis=1))
        index_norm = np.max(np.linalg.norm(index_data, axis=1))
        middle_norm = np.max(np.linalg.norm(middle_data, axis=1))
        ring_norm = np.max(np.linalg.norm(ring_data, axis=1))
        # Limit the value
        limit = 20
        thumb_norm = np.where(thumb_norm < limit, 0, thumb_norm)
        index_norm = np.where(index_norm < limit, 0, index_norm)
        middle_norm = np.where(middle_norm < limit, 0, middle_norm)
        ring_norm = np.where(ring_norm < limit, 0, ring_norm)
        input = np.concatenate((allegro_joints, allegro_joints_delta, [thumb_norm], [ring_norm], [middle_norm], [index_norm] ))

        input = torch.from_numpy(input).float()
        out = self.BC_MLP(input)
        # ee_position = out[0:3].detach().numpy() + ee_data[0:3]
        # AH_joint_angle = out[3:].detach().numpy() + allegro_joints
        # ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out.detach().numpy()
        return AH_joint_angle
    
    
class PolicyAH_yes_norm_output_delta_q():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_norm_ouput_delta_q()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasp_norm_BC_AHEE_10hz_epoch_41.pth"))

    def output(self, allegro_joints, tactile_data):
        # Norm
        thumb_data = tactile_data['thumb']
        index_data = tactile_data['index']
        middle_data = tactile_data['middle']
        ring_data = tactile_data['ring']
        thumb_norm = np.max(np.linalg.norm(thumb_data, axis=1))
        index_norm = np.max(np.linalg.norm(index_data, axis=1))
        middle_norm = np.max(np.linalg.norm(middle_data, axis=1))
        ring_norm = np.max(np.linalg.norm(ring_data, axis=1))
        # Limit the value
        limit = 20
        thumb_norm = np.where(thumb_norm < limit, 0, thumb_norm)
        index_norm = np.where(index_norm < limit, 0, index_norm)
        middle_norm = np.where(middle_norm < limit, 0, middle_norm)
        ring_norm = np.where(ring_norm < limit, 0, ring_norm)
        input = np.concatenate((allegro_joints,  [thumb_norm], [ring_norm], [middle_norm], [index_norm] ))

        input = torch.from_numpy(input).float()
        out = self.BC_MLP(input)
        # ee_position = out[0:3].detach().numpy() + ee_data[0:3]
        # AH_joint_angle = out[3:].detach().numpy() + allegro_joints
        # ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out.detach().numpy()
        return AH_joint_angle