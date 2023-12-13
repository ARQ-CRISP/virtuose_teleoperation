import torch
import numpy as np
from network import BC_MLP, BC_MLP_EE, BC_MLP_AHEE, BC_MLP_AHEE_yes_vel_no_eff, BC_MLP_AHEE_no_vel_no_eff, BC_MLP_AHEE_yVel_nEff_yTact, BC_MLP_AHEE_no_eff_yes_ff, BC_MLP_AHEE_yes_norm, BC_MLP_AHEE_yes_norm_ouput_delta_q, BC_MLP_AH_only_ouput_delta_q, BC_MLP_AH_time_series_only_ouput_delta_q, BC_MLP_EETAC, BC_MLP_EE_HISTORY_TAC, BC_MLP_AHEE_yes_norm_ouput_hglove
from network import Autoencoder, CustomNetwork
import rospy
import numpy as np
from scipy.linalg import logm, expm


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
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasp_norm_BC_10hz_lifter_AHEE_epoch_65.pth"))

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
        limit = 30
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
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasp_norm_BC_10hz_lifter_AHEE_epoch_65.pth"))

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
        # print("tactile_data",tactile_data)
        thumb_norm = np.max(np.linalg.norm(thumb_data, axis=1))
        index_norm = np.max(np.linalg.norm(index_data, axis=1))
        middle_norm = np.max(np.linalg.norm(middle_data, axis=1))
        ring_norm = np.max(np.linalg.norm(ring_data, axis=1))

        # Limit the value
        limit = 70
        thumb_norm = np.where(thumb_norm < limit, 0, thumb_norm)
        index_norm = np.where(index_norm < limit, 0, index_norm)
        middle_norm = np.where(middle_norm < limit, 0, middle_norm)
        ring_norm = np.where(ring_norm < limit, 0, ring_norm)
        input = np.concatenate((allegro_joints,  [thumb_norm], [ring_norm], [middle_norm], [index_norm] ))
        print('thumb',thumb_norm)
        print('index', index_norm)
        print('mid', middle_norm)
        print('ring', ring_norm)
        input = torch.from_numpy(input).float()
        out = self.BC_MLP(input)
        # ee_position = out[0:3].detach().numpy() + ee_data[0:3]
        # AH_joint_angle = out[3:].detach().numpy() + allegro_joints
        # ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out.detach().numpy()
        return AH_joint_angle
    
class PolicyAHFF_output_delta_q():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_norm_ouput_delta_q()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasp_proper_dataset_AHFF_epoch_12.pth"))

    def output(self, allegro_joints, force_feedback):
        # Norm
        thumb_data = force_feedback['thumb']
        index_data = force_feedback['index']
        middle_data = force_feedback['middle']
        ring_data = force_feedback['ring']
        # print("force_feedback",force_feedback)
        thumb_norm = np.linalg.norm(thumb_data)
        index_norm = np.linalg.norm(index_data)
        middle_norm = np.linalg.norm(middle_data)
        ring_norm = np.linalg.norm(ring_data)

        # Limit the value
        limit1 = 100
        limit2 = 70
        thumb_norm = np.where(thumb_norm < limit1, 0, thumb_norm)
        index_norm = np.where(index_norm < limit1, 0, index_norm)
        middle_norm = np.where(middle_norm < limit2, 0, middle_norm)
        ring_norm = np.where(ring_norm < limit1, 0, ring_norm)
        input = np.concatenate((allegro_joints,  [index_norm], [middle_norm], [ring_norm], [thumb_norm]))
        print('thumb',thumb_norm)
        print('index', index_norm)
        print('mid', middle_norm)
        print('ring', ring_norm)
        input = torch.from_numpy(input).float()
        out = self.BC_MLP(input)
        # ee_position = out[0:3].detach().numpy() + ee_data[0:3]
        # AH_joint_angle = out[3:].detach().numpy() + allegro_joints
        # ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out.detach().numpy()
        return AH_joint_angle
    
class PolicyAH_only_output_delta_q():
    def __init__(self):
        self.BC_MLP = BC_MLP_AH_only_ouput_delta_q()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasp_proper_dataset_AH_only_epoch_43.pth"))

    def output(self, allegro_joints):
        input = np.array((allegro_joints))
        input = torch.from_numpy(input).float()
        out = self.BC_MLP(input)
        # ee_position = out[0:3].detach().numpy() + ee_data[0:3]
        # AH_joint_angle = out[3:].detach().numpy() + allegro_joints
        # ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out.detach().numpy()
        return AH_joint_angle

class PolicyAH_time_series_only_output_delta_q():
    def __init__(self):
        self.BC_MLP = BC_MLP_AH_time_series_only_ouput_delta_q()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasp_proper_dataset_AH_time_series_epoch_93.pth"))

    def output(self, allegro_joints_before, allegro_joints):
        input = np.concatenate((allegro_joints_before, allegro_joints))
        input = torch.from_numpy(input).float()
        out = self.BC_MLP(input)
        # ee_position = out[0:3].detach().numpy() + ee_data[0:3]
        # AH_joint_angle = out[3:].detach().numpy() + allegro_joints
        # ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out.detach().numpy()
        return AH_joint_angle
    


class PolicyAHFF_output_delta_q_TAC():
    def __init__(self):
        self.BC_MLP = BC_MLP_EETAC()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasp_proper_dataset_AHFF_200_epoch_67.pth"))

    def output(self, allegro_joints, force_feedback):
        # Norm
        thumb_data = force_feedback['thumb']
        index_data = force_feedback['index']
        middle_data = force_feedback['middle']
        ring_data = force_feedback['ring']
        # print("force_feedback",force_feedback)
        thumb_norm = np.linalg.norm(thumb_data)
        index_norm = np.linalg.norm(index_data)
        middle_norm = np.linalg.norm(middle_data)
        ring_norm = np.linalg.norm(ring_data)

        # Limit the value
        limit1 = 100
        limit2 = 25
        thumb_norm = np.where(thumb_norm < limit1, 0, thumb_norm)
        index_norm = np.where(index_norm < limit1, 0, index_norm)
        middle_norm = np.where(middle_norm < limit2, 0, middle_norm)
        ring_norm = np.where(ring_norm < limit1, 0, ring_norm)
        input = np.concatenate((allegro_joints,  [index_norm], [middle_norm], [ring_norm], [thumb_norm]))
        print('thumb',thumb_norm)
        print('index', index_norm)
        print('mid', middle_norm)
        print('ring', ring_norm)
        input = torch.from_numpy(input).float()
        out_joint, out_tac = self.BC_MLP(input)
        # ee_position = out[0:3].detach().numpy() + ee_data[0:3]
        # AH_joint_angle = out[3:].detach().numpy() + allegro_joints
        # ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out_joint.detach().numpy()
        tactile_predict = out_tac.detach().numpy()
        return AH_joint_angle, tactile_predict
    
class PolicyAH_time_series_only_output_delta_q_TAC():
    def __init__(self):
        self.BC_MLP = BC_MLP_EE_HISTORY_TAC()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasp_proper_dataset_AHFF_400_epoch_26.pth"))

    def output(self, allegro_joints_before, allegro_joints):
        input = np.concatenate((allegro_joints_before, allegro_joints))
        input = torch.from_numpy(input).float()
        out_joint, out_tac = self.BC_MLP(input)
        # ee_position = out[0:3].detach().numpy() + ee_data[0:3]
        # AH_joint_angle = out[3:].detach().numpy() + allegro_joints
        # ee_position = out[0:3].detach().numpy()
        AH_joint_angle = out_joint.detach().numpy()
        tactile_predict = out_tac.detach().numpy()
        return AH_joint_angle, tactile_predict
    


class PolicyAHFF_output_hglove():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_norm_ouput_hglove()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasping_AHFF_epoch_97.pth"))
        #Top grasp: Grasp_proper_dataset_AHFF_out_hglove_epoch_74.pth
    def output(self, allegro_joints, force_feedback):
        # Norm
        thumb_data = force_feedback['thumb']
        index_data = force_feedback['index']
        middle_data = force_feedback['middle']
        ring_data = force_feedback['ring']
        # print("force_feedback",force_feedback)
        thumb_norm = np.linalg.norm(thumb_data)
        index_norm = np.linalg.norm(index_data)
        middle_norm = np.linalg.norm(middle_data)
        ring_norm = np.linalg.norm(ring_data)
        print('index_norm', index_norm)
        print('ring_norm', ring_norm)
        print('thumb_norm', thumb_norm)
        # Limit the value
        limit1 = 100
        limit2 = 25
        thumb_norm = np.where(thumb_norm < limit1, 0, thumb_norm)
        index_norm = np.where(index_norm < limit1, 0, index_norm)
        middle_norm = np.where(middle_norm < limit2, 0, middle_norm)
        ring_norm = np.where(ring_norm < limit1, 0, ring_norm)
        input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm]))

        input = torch.from_numpy(input).float()
        hglove_delta = self.BC_MLP(input)
        predict_joint_angle = mapping(hglove_delta.detach().numpy())

        return predict_joint_angle


class Classification_with_time_series_feature():
    def __init__(self):
        self.AE = Autoencoder()
        self.AE.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasping_AHFF_epoch_97.pth"))
        self.Classifier = CustomNetwork()
        self.Classifier.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Grasping_AHFF_epoch_97.pth"))
        #Top grasp: Grasp_proper_dataset_AHFF_out_hglove_epoch_74.pth
    def output(self,ee_state_history,joint_state_history, ff_norm_history, force_feedback_history, finger_state_history):
        #  Assume the shape of the ff is: 


        ee_time_series_data = ee_state_history
        joint_time_series_data = joint_state_history
        ff_time_series_data = force_feedback_history
        finger_time_series_data = finger_state_history
        Seg_input = self.getitem(self, ee_time_series_data, joint_time_series_data, ff_norm_history, ff_time_series_data, finger_time_series_data)
        predict_label = self.Seg_Net(Seg_input)

        return predict_label
    

    def getitem(self, ee_time_series_data, joint_time_series_data, ff_norm_history, ff_time_series_data, finger_time_series_data):
        #The time series sequence should be: t, t-1, t-2. the timestep is 0.1s.
        #For the computation of temporal feature, the timestep is 0.3s.
        #Therefore, totaly, the whole length is 0.5s
        state_input = []
        for i in range(3):
            joint_data = joint_time_series_data[i] #This should be 16
            ff_data = ff_norm_history[i]  
            ff_data[1] = ff_data[0]        
            ee_state = ee_time_series_data[i]
            state = np.expand_dims(np.concatenate((joint_data, ff_data)), axis=0)    #############No EE state right now
            state_input.append(state)
        concatenated_state_input = np.concatenate(state_input, axis=1).squeeze()

        #Calculate time-series features:
        feature_inputs = []
        for i in range(3):
            ee_state = ee_time_series_data[i]
            ff_data = ff_norm_history[i]
            ff_data[1] = ff_data[0]
            ff_full_data = ff_time_series_data[i]
            finger_state_data = finger_time_series_data[i]
            ee_state_new = ee_time_series_data[i+3]
            ff_data_new = ff_norm_history[i+3] 
            ff_data_new[1] = ff_data_new[0]
            ff_full_data_new = ff_time_series_data[i+3]
            finger_state_data_new = finger_time_series_data[i+3]
            feature_input = self.get_feature(ee_state, ff_data, ff_full_data, finger_state_data, ee_state_new, ff_data_new, ff_full_data_new, finger_state_data_new)
            
            feature_inputs.append(feature_input)
        concatenated_feature_inputs = np.concatenate(feature_inputs, axis=1).squeeze()
        Seg_input = np.concatenate((concatenated_state_input, concatenated_feature_inputs))
        return Seg_input


    def get_feature(self, ee_state,ff_data, ff_full_data, finger_state_data, ee_state_new, ff_data_new, ff_full_data_new, finger_state_data_new ):
        #Current state

        ee_angle = quaternion_to_euler(ee_state[3], ee_state[4], ee_state[5], ee_state[6])
        ff_data[1] = ff_data[0]
        #Temporal state

        ee_angle_new = quaternion_to_euler(ee_state_new[3], ee_state_new[4], ee_state_new[5], ee_state_new[6])
        ff_data_new[1] = ff_data_new[0]
        ###### 10 here represents 3 times the velocity per sceond
        ee_velocity = (ee_state_new[0:3] - ee_state[0:3]) * 10

        ee_angular_velocity = np.sin((ee_angle_new - ee_angle) * 10)

        fingertip_velocity = np.abs((finger_state_data_new - finger_state_data) * 10)

        finger_cov_data = logm(np.cov(finger_state_data.reshape(7, -1).T))
        rows, cols = np.triu_indices(finger_cov_data.shape[0], k=1)
        finger_cov_data = finger_cov_data[rows, cols]


        epsilon = 1e-10  # You might need to adjust this value
        regularized_matrix_fingertip = epsilon * np.eye(4)
        fingertip_temporal_cov =  logm(np.cov(finger_state_data_new.reshape(7, -1).T - finger_state_data.reshape(7, -1).T) + regularized_matrix_fingertip)
        rows, cols = np.triu_indices(fingertip_temporal_cov.shape[0], k=1)
        fingertip_temporal_cov = fingertip_temporal_cov[rows, cols]

        ff_data_temporal = np.abs((ff_data_new - ff_data) * 10)
        regularized_matrix_ff = epsilon * np.eye(12)
        ff_data_temporal_cov = logm(np.cov(np.stack((ff_full_data, ff_full_data_new), axis = 1)) + regularized_matrix_ff)
        rows, cols = np.triu_indices(ff_data_temporal_cov.shape[0], k=1)
        ff_data_temporal_cov = ff_data_temporal_cov[rows, cols]
        # state = np.expand_dims(np.concatenate((ee_state, joint_data[0:16],  ff_data)), axis=0) # 

        feature = np.expand_dims(np.concatenate(( ee_velocity, ee_angular_velocity, finger_state_data, fingertip_velocity, finger_cov_data,  fingertip_temporal_cov, ff_data_temporal,  ff_data_temporal_cov)), axis=0)
        return feature











def mapping(hglove_delta):
    a_in = hglove_delta[0]
    b_in = hglove_delta[1]
    c_in = hglove_delta[2]
    a_mid = hglove_delta[3]
    b_mid = hglove_delta[4]
    c_mid = hglove_delta[5]
    a_th = hglove_delta[6]
    b_th = hglove_delta[7]
    c_th = hglove_delta[8]
    joint_pose = \
            [0.1+a_in, 0.1+b_in*1.5, 0.0+b_in, 0.2+c_in*1.0,\
            0.1+a_in, 0.1+b_in*1.5, 0.0+b_in, 0.2+c_in*1.0,\
            0.1+a_mid, 0.1+b_mid*1.3, 0.0+b_mid, 0.0+c_mid*0.8,\
            1.49, -0.2-a_th*1.5, 0.0+2*b_th, -0.2+0.15*(b_th-c_th)] ##0 non so, 1 rotazione verticale, 2 rotazione verticale, 3finger pitch
            #1.3, 0.0, -0.3, 1.3] 
    return joint_pose 



    
def quaternion_to_euler(qw, qx, qy, qz):
    # Calculate Euler angles
    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    roll_x = np.arctan2(t0, t1)
    
    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
    pitch_y = np.arcsin(t2)
    
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw_z = np.arctan2(t3, t4)
    
    return np.array([roll_x, pitch_y, yaw_z])


