import torch
import numpy as np
from network import  BC_MLP_AHEE_yes_norm_ouput_hglove_ee
from network import Autoencoder, CustomNetwork
import rospy
import numpy as np
from scipy.linalg import logm, expm


class Brick_reach():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_norm_ouput_hglove_ee()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\Brick_demonstration\\Reach_skill.pth"))

    def output(self, allegro_joints, force_feedback, ee_state):
        # Norm
        thumb_data = force_feedback['thumb']
        index_data = force_feedback['index']
        middle_data = force_feedback['middle']
        ring_data = force_feedback['ring']
        # print("force_feedback",force_feedback)
        thumb_norm_data = np.linalg.norm(thumb_data)
        index_norm_data = np.linalg.norm(index_data)
        middle_norm_data = np.linalg.norm(middle_data)
        ring_norm_data = np.linalg.norm(ring_data)
        # print('index_norm', index_norm)
        # print('ring_norm', ring_norm)
        # print('thumb_norm', thumb_norm)
        # # Limit the value
        # limit1 = 100
        # limit2 = 25
        # thumb_norm = np.where(thumb_norm < limit1, 0, thumb_norm)
        # index_norm = np.where(index_norm < limit1, 0, index_norm)
        # middle_norm = np.where(middle_norm < limit2, 0, middle_norm)
        # ring_norm = np.where(ring_norm < limit1, 0, ring_norm)
        # input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm]))
        limit1 = 100
        limit2 = 50
        thumb_norm = np.where(thumb_norm_data < limit1, 0, thumb_norm_data)
        index_norm = np.where(index_norm_data < limit1, 0, index_norm_data)
        middle_norm = np.where(middle_norm_data < limit2, 0, middle_norm_data)
        ring_norm = np.where(ring_norm_data < limit1, 0, ring_norm_data)
        #Normalize:
        index_norm = self.normalize_using_sigmoid(index_norm, -50, 400)
        middle_norm = self.normalize_using_sigmoid(middle_norm, -50, 400)
        ring_norm = self.normalize_using_sigmoid(ring_norm, -50, 400)
        thumb_norm = self.normalize_using_sigmoid(thumb_norm, -50, 400)
        #Binary ff
        binary_force_feedback_data_0 = np.where(index_norm_data < limit1, 0, 1) #index
        binary_force_feedback_data_1 = np.where(middle_norm_data < limit2, 0, 1) #mid
        binary_force_feedback_data_2 = np.where(ring_norm_data < limit1, 0, 1) #ring
        binary_force_feedback_data_3 = np.where(thumb_norm_data < limit1, 0, 1) #thumb

        # binary_force_feedback_data = np.stack((binary_force_feedback_data_0, binary_force_feedback_data_1, binary_force_feedback_data_2, binary_force_feedback_data_3), axis=1)

        input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm], [binary_force_feedback_data_0], [binary_force_feedback_data_0], [binary_force_feedback_data_2], [binary_force_feedback_data_3], ee_state[0:3]))






        input = torch.from_numpy(input).float()
        output = self.BC_MLP(input)
        predict_joint_angle = mapping(output[0:9].detach().numpy())
        predict_ee_control = output[9:].detach().numpy()

        return predict_joint_angle, predict_ee_control
    def normalize_using_sigmoid(self, array, min_val, max_val, k=0.02):
        # Center the range around 0 and scale it
        b = (max_val + min_val) / 2
        # Apply the sigmoid function to the entire array
        normalized_array = 1 / (1 + np.exp(-k*(array - b)))
        return normalized_array

class Brick_flip():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_norm_ouput_hglove_ee()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\Brick_demonstration\\Flip_skill.pth"))
        #Top grasp: Grasp_proper_dataset_AHFF_out_hglove_epoch_74.pth
    def output(self, allegro_joints, force_feedback, ee_state):
        # Norm
        thumb_data = force_feedback['thumb']
        index_data = force_feedback['index']
        middle_data = force_feedback['middle']
        ring_data = force_feedback['ring']
        # print("force_feedback",force_feedback)
        thumb_norm_data = np.linalg.norm(thumb_data)
        index_norm_data = np.linalg.norm(index_data)
        middle_norm_data = np.linalg.norm(middle_data)
        ring_norm_data = np.linalg.norm(ring_data)
        # print('index_norm', index_norm)
        # print('ring_norm', ring_norm)
        # print('thumb_norm', thumb_norm)
        # # Limit the value
        # limit1 = 100
        # limit2 = 25
        # thumb_norm = np.where(thumb_norm < limit1, 0, thumb_norm)
        # index_norm = np.where(index_norm < limit1, 0, index_norm)
        # middle_norm = np.where(middle_norm < limit2, 0, middle_norm)
        # ring_norm = np.where(ring_norm < limit1, 0, ring_norm)
        # input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm]))
        limit1 = 100
        limit2 = 50
        thumb_norm = np.where(thumb_norm_data < limit1, 0, thumb_norm_data)
        index_norm = np.where(index_norm_data < limit1, 0, index_norm_data)
        middle_norm = np.where(middle_norm_data < limit2, 0, middle_norm_data)
        ring_norm = np.where(ring_norm_data < limit1, 0, ring_norm_data)
        #Normalize:
        index_norm = self.normalize_using_sigmoid(index_norm, -50, 400)
        middle_norm = self.normalize_using_sigmoid(middle_norm, -50, 400)
        ring_norm = self.normalize_using_sigmoid(ring_norm, -50, 400)
        thumb_norm = self.normalize_using_sigmoid(thumb_norm, -50, 400)
        #Binary ff
        binary_force_feedback_data_0 = np.where(index_norm_data < limit1, 0, 1) #index
        binary_force_feedback_data_1 = np.where(middle_norm_data < limit2, 0, 1) #mid
        binary_force_feedback_data_2 = np.where(ring_norm_data < limit1, 0, 1) #ring
        binary_force_feedback_data_3 = np.where(thumb_norm_data < limit1, 0, 1) #thumb

        # binary_force_feedback_data = np.stack((binary_force_feedback_data_0, binary_force_feedback_data_1, binary_force_feedback_data_2, binary_force_feedback_data_3), axis=1)

        input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm], [binary_force_feedback_data_0], [binary_force_feedback_data_0], [binary_force_feedback_data_2], [binary_force_feedback_data_3], ee_state[0:3]))






        input = torch.from_numpy(input).float()
        output = self.BC_MLP(input)
        predict_joint_angle = mapping(output[0:9].detach().numpy())
        predict_ee_control = output[9:].detach().numpy()

        return predict_joint_angle, predict_ee_control
    def normalize_using_sigmoid(self, array, min_val, max_val, k=0.02):
        # Center the range around 0 and scale it
        b = (max_val + min_val) / 2
        # Apply the sigmoid function to the entire array
        normalized_array = 1 / (1 + np.exp(-k*(array - b)))
        return normalized_array

class Brick_touch():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_norm_ouput_hglove_ee()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\Brick_demonstration\\Touch_object_skill.pth"))
        #Top grasp: Grasp_proper_dataset_AHFF_out_hglove_epoch_74.pth
    def output(self, allegro_joints, force_feedback, ee_state):
        # Norm
        thumb_data = force_feedback['thumb']
        index_data = force_feedback['index']
        middle_data = force_feedback['middle']
        ring_data = force_feedback['ring']
        # print("force_feedback",force_feedback)
        thumb_norm_data = np.linalg.norm(thumb_data)
        index_norm_data = np.linalg.norm(index_data)
        middle_norm_data = np.linalg.norm(middle_data)
        ring_norm_data = np.linalg.norm(ring_data)
        # print('index_norm', index_norm)
        # print('ring_norm', ring_norm)
        # print('thumb_norm', thumb_norm)
        # # Limit the value
        # limit1 = 100
        # limit2 = 25
        # thumb_norm = np.where(thumb_norm < limit1, 0, thumb_norm)
        # index_norm = np.where(index_norm < limit1, 0, index_norm)
        # middle_norm = np.where(middle_norm < limit2, 0, middle_norm)
        # ring_norm = np.where(ring_norm < limit1, 0, ring_norm)
        # input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm]))
        limit1 = 100
        limit2 = 50
        thumb_norm = np.where(thumb_norm_data < limit1, 0, thumb_norm_data)
        index_norm = np.where(index_norm_data < limit1, 0, index_norm_data)
        middle_norm = np.where(middle_norm_data < limit2, 0, middle_norm_data)
        ring_norm = np.where(ring_norm_data < limit1, 0, ring_norm_data)
        #Normalize:
        index_norm = self.normalize_using_sigmoid(index_norm, -50, 400)
        middle_norm = self.normalize_using_sigmoid(middle_norm, -50, 400)
        ring_norm = self.normalize_using_sigmoid(ring_norm, -50, 400)
        thumb_norm = self.normalize_using_sigmoid(thumb_norm, -50, 400)
        #Binary ff
        binary_force_feedback_data_0 = np.where(index_norm_data < limit1, 0, 1) #index
        binary_force_feedback_data_1 = np.where(middle_norm_data < limit2, 0, 1) #mid
        binary_force_feedback_data_2 = np.where(ring_norm_data < limit1, 0, 1) #ring
        binary_force_feedback_data_3 = np.where(thumb_norm_data < limit1, 0, 1) #thumb

        # binary_force_feedback_data = np.stack((binary_force_feedback_data_0, binary_force_feedback_data_1, binary_force_feedback_data_2, binary_force_feedback_data_3), axis=1)

        input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm], [binary_force_feedback_data_0], [binary_force_feedback_data_0], [binary_force_feedback_data_2], [binary_force_feedback_data_3], ee_state[0:3]))






        input = torch.from_numpy(input).float()
        output = self.BC_MLP(input)
        predict_joint_angle = mapping(output[0:9].detach().numpy())
        predict_ee_control = output[9:].detach().numpy()

        return predict_joint_angle, predict_ee_control
    def normalize_using_sigmoid(self, array, min_val, max_val, k=0.02):
        # Center the range around 0 and scale it
        b = (max_val + min_val) / 2
        # Apply the sigmoid function to the entire array
        normalized_array = 1 / (1 + np.exp(-k*(array - b)))
        return normalized_array
    
class Brick_pull():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_norm_ouput_hglove_ee()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\Brick_demonstration\\Pull_skill.pth"))
        #Top grasp: Grasp_proper_dataset_AHFF_out_hglove_epoch_74.pth
    def output(self, allegro_joints, force_feedback, ee_state):
        # Norm
        thumb_data = force_feedback['thumb']
        index_data = force_feedback['index']
        middle_data = force_feedback['middle']
        ring_data = force_feedback['ring']
        # print("force_feedback",force_feedback)
        thumb_norm_data = np.linalg.norm(thumb_data)
        index_norm_data = np.linalg.norm(index_data)
        middle_norm_data = np.linalg.norm(middle_data)
        ring_norm_data = np.linalg.norm(ring_data)
        # print('index_norm', index_norm)
        # print('ring_norm', ring_norm)
        # print('thumb_norm', thumb_norm)
        # # Limit the value
        # limit1 = 100
        # limit2 = 25
        # thumb_norm = np.where(thumb_norm < limit1, 0, thumb_norm)
        # index_norm = np.where(index_norm < limit1, 0, index_norm)
        # middle_norm = np.where(middle_norm < limit2, 0, middle_norm)
        # ring_norm = np.where(ring_norm < limit1, 0, ring_norm)
        # input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm]))
        limit1 = 100
        limit2 = 50
        thumb_norm = np.where(thumb_norm_data < limit1, 0, thumb_norm_data)
        index_norm = np.where(index_norm_data < limit1, 0, index_norm_data)
        middle_norm = np.where(middle_norm_data < limit2, 0, middle_norm_data)
        ring_norm = np.where(ring_norm_data < limit1, 0, ring_norm_data)
        #Normalize:
        index_norm = self.normalize_using_sigmoid(index_norm, -50, 400)
        middle_norm = self.normalize_using_sigmoid(middle_norm, -50, 400)
        ring_norm = self.normalize_using_sigmoid(ring_norm, -50, 400)
        thumb_norm = self.normalize_using_sigmoid(thumb_norm, -50, 400)
        #Binary ff
        binary_force_feedback_data_0 = np.where(index_norm_data < limit1, 0, 1) #index
        binary_force_feedback_data_1 = np.where(middle_norm_data < limit2, 0, 1) #mid
        binary_force_feedback_data_2 = np.where(ring_norm_data < limit1, 0, 1) #ring
        binary_force_feedback_data_3 = np.where(thumb_norm_data < limit1, 0, 1) #thumb

        # binary_force_feedback_data = np.stack((binary_force_feedback_data_0, binary_force_feedback_data_1, binary_force_feedback_data_2, binary_force_feedback_data_3), axis=1)

        input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm], [binary_force_feedback_data_0], [binary_force_feedback_data_0], [binary_force_feedback_data_2], [binary_force_feedback_data_3], ee_state[0:3]))






        input = torch.from_numpy(input).float()
        output = self.BC_MLP(input)
        predict_joint_angle = mapping(output[0:9].detach().numpy())
        predict_ee_control = output[9:].detach().numpy()

        return predict_joint_angle, predict_ee_control
    def normalize_using_sigmoid(self, array, min_val, max_val, k=0.02):
        # Center the range around 0 and scale it
        b = (max_val + min_val) / 2
        # Apply the sigmoid function to the entire array
        normalized_array = 1 / (1 + np.exp(-k*(array - b)))
        return normalized_array
    
class Brick_push():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_norm_ouput_hglove_ee()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\Brick_demonstration\\Push_skill.pth"))
        #Top grasp: Grasp_proper_dataset_AHFF_out_hglove_epoch_74.pth
    def output(self, allegro_joints, force_feedback, ee_state):
        # Norm
        thumb_data = force_feedback['thumb']
        index_data = force_feedback['index']
        middle_data = force_feedback['middle']
        ring_data = force_feedback['ring']
        # print("force_feedback",force_feedback)
        thumb_norm_data = np.linalg.norm(thumb_data)
        index_norm_data = np.linalg.norm(index_data)
        middle_norm_data = np.linalg.norm(middle_data)
        ring_norm_data = np.linalg.norm(ring_data)
        # print('index_norm', index_norm)
        # print('ring_norm', ring_norm)
        # print('thumb_norm', thumb_norm)
        # # Limit the value
        # limit1 = 100
        # limit2 = 25
        # thumb_norm = np.where(thumb_norm < limit1, 0, thumb_norm)
        # index_norm = np.where(index_norm < limit1, 0, index_norm)
        # middle_norm = np.where(middle_norm < limit2, 0, middle_norm)
        # ring_norm = np.where(ring_norm < limit1, 0, ring_norm)
        # input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm]))
        limit1 = 100
        limit2 = 50
        thumb_norm = np.where(thumb_norm_data < limit1, 0, thumb_norm_data)
        index_norm = np.where(index_norm_data < limit1, 0, index_norm_data)
        middle_norm = np.where(middle_norm_data < limit2, 0, middle_norm_data)
        ring_norm = np.where(ring_norm_data < limit1, 0, ring_norm_data)
        #Normalize:
        index_norm = self.normalize_using_sigmoid(index_norm, -50, 400)
        middle_norm = self.normalize_using_sigmoid(middle_norm, -50, 400)
        ring_norm = self.normalize_using_sigmoid(ring_norm, -50, 400)
        thumb_norm = self.normalize_using_sigmoid(thumb_norm, -50, 400)
        #Binary ff
        binary_force_feedback_data_0 = np.where(index_norm_data < limit1, 0, 1) #index
        binary_force_feedback_data_1 = np.where(middle_norm_data < limit2, 0, 1) #mid
        binary_force_feedback_data_2 = np.where(ring_norm_data < limit1, 0, 1) #ring
        binary_force_feedback_data_3 = np.where(thumb_norm_data < limit1, 0, 1) #thumb

        # binary_force_feedback_data = np.stack((binary_force_feedback_data_0, binary_force_feedback_data_1, binary_force_feedback_data_2, binary_force_feedback_data_3), axis=1)

        input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm], [binary_force_feedback_data_0], [binary_force_feedback_data_0], [binary_force_feedback_data_2], [binary_force_feedback_data_3], ee_state[0:3]))






        input = torch.from_numpy(input).float()
        output = self.BC_MLP(input)
        predict_joint_angle = mapping(output[0:9].detach().numpy())
        predict_ee_control = output[9:].detach().numpy()

        return predict_joint_angle, predict_ee_control
    def normalize_using_sigmoid(self, array, min_val, max_val, k=0.02):
        # Center the range around 0 and scale it
        b = (max_val + min_val) / 2
        # Apply the sigmoid function to the entire array
        normalized_array = 1 / (1 + np.exp(-k*(array - b)))
        return normalized_array
    
class Brick_pre_grasp():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_norm_ouput_hglove_ee()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\Brick_demonstration\\Center_grasp_skill.pth"))
        #Top grasp: Grasp_proper_dataset_AHFF_out_hglove_epoch_74.pth
    def output(self, allegro_joints, force_feedback, ee_state):
        # Norm
        thumb_data = force_feedback['thumb']
        index_data = force_feedback['index']
        middle_data = force_feedback['middle']
        ring_data = force_feedback['ring']
        # print("force_feedback",force_feedback)
        thumb_norm_data = np.linalg.norm(thumb_data)
        index_norm_data = np.linalg.norm(index_data)
        middle_norm_data = np.linalg.norm(middle_data)
        ring_norm_data = np.linalg.norm(ring_data)
        # print('index_norm', index_norm)
        # print('ring_norm', ring_norm)
        # print('thumb_norm', thumb_norm)
        # # Limit the value
        # limit1 = 100
        # limit2 = 25
        # thumb_norm = np.where(thumb_norm < limit1, 0, thumb_norm)
        # index_norm = np.where(index_norm < limit1, 0, index_norm)
        # middle_norm = np.where(middle_norm < limit2, 0, middle_norm)
        # ring_norm = np.where(ring_norm < limit1, 0, ring_norm)
        # input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm]))
        limit1 = 100
        limit2 = 50
        thumb_norm = np.where(thumb_norm_data < limit1, 0, thumb_norm_data)
        index_norm = np.where(index_norm_data < limit1, 0, index_norm_data)
        middle_norm = np.where(middle_norm_data < limit2, 0, middle_norm_data)
        ring_norm = np.where(ring_norm_data < limit1, 0, ring_norm_data)
        #Normalize:
        index_norm = self.normalize_using_sigmoid(index_norm, -50, 400)
        middle_norm = self.normalize_using_sigmoid(middle_norm, -50, 400)
        ring_norm = self.normalize_using_sigmoid(ring_norm, -50, 400)
        thumb_norm = self.normalize_using_sigmoid(thumb_norm, -50, 400)
        #Binary ff
        binary_force_feedback_data_0 = np.where(index_norm_data < limit1, 0, 1) #index
        binary_force_feedback_data_1 = np.where(middle_norm_data < limit2, 0, 1) #mid
        binary_force_feedback_data_2 = np.where(ring_norm_data < limit1, 0, 1) #ring
        binary_force_feedback_data_3 = np.where(thumb_norm_data < limit1, 0, 1) #thumb

        # binary_force_feedback_data = np.stack((binary_force_feedback_data_0, binary_force_feedback_data_1, binary_force_feedback_data_2, binary_force_feedback_data_3), axis=1)

        input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm], [binary_force_feedback_data_0], [binary_force_feedback_data_0], [binary_force_feedback_data_2], [binary_force_feedback_data_3], ee_state[0:3]))






        input = torch.from_numpy(input).float()
        output = self.BC_MLP(input)
        predict_joint_angle = mapping(output[0:9].detach().numpy())
        predict_ee_control = output[9:].detach().numpy()

        return predict_joint_angle, predict_ee_control
    def normalize_using_sigmoid(self, array, min_val, max_val, k=0.02):
        # Center the range around 0 and scale it
        b = (max_val + min_val) / 2
        # Apply the sigmoid function to the entire array
        normalized_array = 1 / (1 + np.exp(-k*(array - b)))
        return normalized_array
    
class Brick_grasp():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_norm_ouput_hglove_ee()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\Brick_demonstration\\Grasp_skill.pth"))
        #Top grasp: Grasp_proper_dataset_AHFF_out_hglove_epoch_74.pth
    def output(self, allegro_joints, force_feedback, ee_state):
        # Norm
        thumb_data = force_feedback['thumb']
        index_data = force_feedback['index']
        middle_data = force_feedback['middle']
        ring_data = force_feedback['ring']
        # print("force_feedback",force_feedback)
        thumb_norm_data = np.linalg.norm(thumb_data)
        index_norm_data = np.linalg.norm(index_data)
        middle_norm_data = np.linalg.norm(middle_data)
        ring_norm_data = np.linalg.norm(ring_data)
        # print('index_norm', index_norm)
        # print('ring_norm', ring_norm)
        # print('thumb_norm', thumb_norm)
        # # Limit the value
        # limit1 = 100
        # limit2 = 25
        # thumb_norm = np.where(thumb_norm < limit1, 0, thumb_norm)
        # index_norm = np.where(index_norm < limit1, 0, index_norm)
        # middle_norm = np.where(middle_norm < limit2, 0, middle_norm)
        # ring_norm = np.where(ring_norm < limit1, 0, ring_norm)
        # input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm]))
        limit1 = 100
        limit2 = 50
        thumb_norm = np.where(thumb_norm_data < limit1, 0, thumb_norm_data)
        index_norm = np.where(index_norm_data < limit1, 0, index_norm_data)
        middle_norm = np.where(middle_norm_data < limit2, 0, middle_norm_data)
        ring_norm = np.where(ring_norm_data < limit1, 0, ring_norm_data)
        #Normalize:
        index_norm = self.normalize_using_sigmoid(index_norm, -50, 400)
        middle_norm = self.normalize_using_sigmoid(middle_norm, -50, 400)
        ring_norm = self.normalize_using_sigmoid(ring_norm, -50, 400)
        thumb_norm = self.normalize_using_sigmoid(thumb_norm, -50, 400)
        #Binary ff
        binary_force_feedback_data_0 = np.where(index_norm_data < limit1, 0, 1) #index
        binary_force_feedback_data_1 = np.where(middle_norm_data < limit2, 0, 1) #mid
        binary_force_feedback_data_2 = np.where(ring_norm_data < limit1, 0, 1) #ring
        binary_force_feedback_data_3 = np.where(thumb_norm_data < limit1, 0, 1) #thumb

        # binary_force_feedback_data = np.stack((binary_force_feedback_data_0, binary_force_feedback_data_1, binary_force_feedback_data_2, binary_force_feedback_data_3), axis=1)

        input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm], [binary_force_feedback_data_0], [binary_force_feedback_data_0], [binary_force_feedback_data_2], [binary_force_feedback_data_3], ee_state[0:3]))






        input = torch.from_numpy(input).float()
        output = self.BC_MLP(input)
        predict_joint_angle = mapping(output[0:9].detach().numpy())
        predict_ee_control = output[9:].detach().numpy()

        return predict_joint_angle, predict_ee_control
    def normalize_using_sigmoid(self, array, min_val, max_val, k=0.02):
        # Center the range around 0 and scale it
        b = (max_val + min_val) / 2
        # Apply the sigmoid function to the entire array
        normalized_array = 1 / (1 + np.exp(-k*(array - b)))
        return normalized_array
    
class Brick_lift():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_norm_ouput_hglove_ee()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\Brick_demonstration\\Lift_with_grasp_skill.pth"))
        #Top grasp: Grasp_proper_dataset_AHFF_out_hglove_epoch_74.pth
    def output(self, allegro_joints, force_feedback, ee_state):
        # Norm
        thumb_data = force_feedback['thumb']
        index_data = force_feedback['index']
        middle_data = force_feedback['middle']
        ring_data = force_feedback['ring']
        # print("force_feedback",force_feedback)
        thumb_norm_data = np.linalg.norm(thumb_data)
        index_norm_data = np.linalg.norm(index_data)
        middle_norm_data = np.linalg.norm(middle_data)
        ring_norm_data = np.linalg.norm(ring_data)
        # print('index_norm', index_norm)
        # print('ring_norm', ring_norm)
        # print('thumb_norm', thumb_norm)
        # # Limit the value
        # limit1 = 100
        # limit2 = 25
        # thumb_norm = np.where(thumb_norm < limit1, 0, thumb_norm)
        # index_norm = np.where(index_norm < limit1, 0, index_norm)
        # middle_norm = np.where(middle_norm < limit2, 0, middle_norm)
        # ring_norm = np.where(ring_norm < limit1, 0, ring_norm)
        # input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm]))
        limit1 = 100
        limit2 = 50
        thumb_norm = np.where(thumb_norm_data < limit1, 0, thumb_norm_data)
        index_norm = np.where(index_norm_data < limit1, 0, index_norm_data)
        middle_norm = np.where(middle_norm_data < limit2, 0, middle_norm_data)
        ring_norm = np.where(ring_norm_data < limit1, 0, ring_norm_data)
        #Normalize:
        index_norm = self.normalize_using_sigmoid(index_norm, -50, 400)
        middle_norm = self.normalize_using_sigmoid(middle_norm, -50, 400)
        ring_norm = self.normalize_using_sigmoid(ring_norm, -50, 400)
        thumb_norm = self.normalize_using_sigmoid(thumb_norm, -50, 400)
        #Binary ff
        binary_force_feedback_data_0 = np.where(index_norm_data < limit1, 0, 1) #index
        binary_force_feedback_data_1 = np.where(middle_norm_data < limit2, 0, 1) #mid
        binary_force_feedback_data_2 = np.where(ring_norm_data < limit1, 0, 1) #ring
        binary_force_feedback_data_3 = np.where(thumb_norm_data < limit1, 0, 1) #thumb

        # binary_force_feedback_data = np.stack((binary_force_feedback_data_0, binary_force_feedback_data_1, binary_force_feedback_data_2, binary_force_feedback_data_3), axis=1)

        input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm], [binary_force_feedback_data_0], [binary_force_feedback_data_0], [binary_force_feedback_data_2], [binary_force_feedback_data_3], ee_state[0:3]))






        input = torch.from_numpy(input).float()
        output = self.BC_MLP(input)
        predict_joint_angle = mapping(output[0:9].detach().numpy())
        predict_ee_control = output[9:].detach().numpy()

        return predict_joint_angle, predict_ee_control
    def normalize_using_sigmoid(self, array, min_val, max_val, k=0.02):
        # Center the range around 0 and scale it
        b = (max_val + min_val) / 2
        # Apply the sigmoid function to the entire array
        normalized_array = 1 / (1 + np.exp(-k*(array - b)))
        return normalized_array
    


class Brick_release():
    def __init__(self):
        self.BC_MLP = BC_MLP_AHEE_yes_norm_ouput_hglove_ee()
        self.BC_MLP.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\Brick_demonstration\\Release_skill.pth"))
        #Top grasp: Grasp_proper_dataset_AHFF_out_hglove_epoch_74.pth
    def output(self, allegro_joints, force_feedback, ee_state):
        # Norm
        thumb_data = force_feedback['thumb']
        index_data = force_feedback['index']
        middle_data = force_feedback['middle']
        ring_data = force_feedback['ring']
        # print("force_feedback",force_feedback)
        thumb_norm_data = np.linalg.norm(thumb_data)
        index_norm_data = np.linalg.norm(index_data)
        middle_norm_data = np.linalg.norm(middle_data)
        ring_norm_data = np.linalg.norm(ring_data)
        # print('index_norm', index_norm)
        # print('ring_norm', ring_norm)
        # print('thumb_norm', thumb_norm)
        # # Limit the value
        # limit1 = 100
        # limit2 = 25
        # thumb_norm = np.where(thumb_norm < limit1, 0, thumb_norm)
        # index_norm = np.where(index_norm < limit1, 0, index_norm)
        # middle_norm = np.where(middle_norm < limit2, 0, middle_norm)
        # ring_norm = np.where(ring_norm < limit1, 0, ring_norm)
        # input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm]))
        limit1 = 100
        limit2 = 50
        thumb_norm = np.where(thumb_norm_data < limit1, 0, thumb_norm_data)
        index_norm = np.where(index_norm_data < limit1, 0, index_norm_data)
        middle_norm = np.where(middle_norm_data < limit2, 0, middle_norm_data)
        ring_norm = np.where(ring_norm_data < limit1, 0, ring_norm_data)
        #Normalize:
        index_norm = self.normalize_using_sigmoid(index_norm, -50, 400)
        middle_norm = self.normalize_using_sigmoid(middle_norm, -50, 400)
        ring_norm = self.normalize_using_sigmoid(ring_norm, -50, 400)
        thumb_norm = self.normalize_using_sigmoid(thumb_norm, -50, 400)
        #Binary ff
        binary_force_feedback_data_0 = np.where(index_norm_data < limit1, 0, 1) #index
        binary_force_feedback_data_1 = np.where(middle_norm_data < limit2, 0, 1) #mid
        binary_force_feedback_data_2 = np.where(ring_norm_data < limit1, 0, 1) #ring
        binary_force_feedback_data_3 = np.where(thumb_norm_data < limit1, 0, 1) #thumb

        # binary_force_feedback_data = np.stack((binary_force_feedback_data_0, binary_force_feedback_data_1, binary_force_feedback_data_2, binary_force_feedback_data_3), axis=1)

        input = np.concatenate((allegro_joints,  [index_norm], [index_norm], [ring_norm], [thumb_norm], [binary_force_feedback_data_0], [binary_force_feedback_data_0], [binary_force_feedback_data_2], [binary_force_feedback_data_3], ee_state[0:3]))






        input = torch.from_numpy(input).float()
        output = self.BC_MLP(input)
        predict_joint_angle = mapping(output[0:9].detach().numpy())
        predict_ee_control = output[9:].detach().numpy()

        return predict_joint_angle, predict_ee_control
    def normalize_using_sigmoid(self, array, min_val, max_val, k=0.02):
        # Center the range around 0 and scale it
        b = (max_val + min_val) / 2
        # Apply the sigmoid function to the entire array
        normalized_array = 1 / (1 + np.exp(-k*(array - b)))
        return normalized_array



class Classification_with_time_series_feature():
    def __init__(self):
        self.AE = Autoencoder(input_size=144*3+20*3, bottleneck_size=256).float()
        self.AE.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\AE_256_new_skill_epoch_856.pth"))
        self.AE.eval()
        self.Classifier = CustomNetwork().float()
        self.Classifier.load_state_dict(torch.load("C:\\catkin_ws\\src\\virtuose_teleoperation\\src\\virtuose_teleoperation\\network_controller\\trained_policy_MLP\\Seg_256_new_skill_epoch_856.pth"))
        self.Classifier.eval()
        #Top grasp: Grasp_proper_dataset_AHFF_out_hglove_epoch_74.pth
    def output(self,ee_state_history,joint_state_history, ff_norm_history, force_feedback_history, finger_state_history):
        #  Assume the shape of the ff is: 


        ee_time_series_data = np.array(ee_state_history)
        joint_time_series_data = np.array(joint_state_history)
        ff_norm_history = np.array(ff_norm_history)
        ff_time_series_data = np.array(force_feedback_history)
        finger_time_series_data = np.array(finger_state_history)

        feature = self.getitem(ee_time_series_data, joint_time_series_data, ff_norm_history, ff_time_series_data, finger_time_series_data)
        AE_input = torch.tensor(feature).float()
        latent_feature = self.AE.get_latent_vector(AE_input)
        predict_label, _, _ = self.Classifier(latent_feature)
        
        return predict_label.argmax().cpu().detach()
    

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
        feature = np.concatenate((concatenated_state_input, concatenated_feature_inputs))
        return feature


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


