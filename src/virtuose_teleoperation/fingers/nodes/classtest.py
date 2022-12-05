#! /usr/bin/env python
import rospy
from virtuose.msg import out_virtuose_pose
# msg=out_virtuose_pose
# msg.virtuose_pose.rotation.x
import actionlib
import numpy as np
from allegro_hand_kdl.msg import PoseControlAction, PoseControlGoal, PoseControlResult, PoseControlFeedback
from geometry_msgs.msg import Pose
import tf.transformations

#this file is the conversion of pose_action_client_hglove_old to a Class Structure
class PoseActionClient(object):


    def __init__(self, init_pose=None,):
        self.initial_pose_flag=False
        self.q1_inv=[0,0,0,1]
        self.q2=[0,0,0,1]
        self.initial_pose_msg=out_virtuose_pose()
        self.delta_pose_msg=out_virtuose_pose()
        n_sin = np.arange(50)
        dt = 0.001
        x_sin = np.sin(2 * np.pi * 50 * n_sin * dt)
        self.x_sin=x_sin/15
        #print(x_sin)
        self.h_thumb_x = 0.0
        self.h_thumb_y = 0.0
        self.h_thumb_z = 0.0
        self.quat_thumb_delta_x=0.0
        self.quat_thumb_delta_y=0.0
        self.quat_thumb_delta_z=0.0
        self.quat_thumb_delta_w=0.0

    # this is a program that take as input ros messages from hglove to control allegro hand
        self.out_thumb_subscriber =  rospy.Subscriber("/out_thumb_pose", out_virtuose_pose, self.__OnOutVirtuosePoseReceived)
        self.delta_pose_pub = rospy.Publisher("/delta_pose_thumb", out_virtuose_pose, queue_size=5)

        cartesian_controller=True
        joints_controller=False

        client =\
            actionlib.SimpleActionClient('pose_control_action', PoseControlAction)
        gg=1 
                                                
            ###
                                                #### CARTESIAN CONTROLLER ###
            ###
        if cartesian_controller==True:

            client.wait_for_server()
            k=0
            
            while not rospy.is_shutdown(): #finger 3=thumb  2 = pinkie 1= middle 0=index?
                # create a cartesian pose for testing
                # 'relax' pose from cartesian_poses.yaml
                finger_poses = []
                print("CARTESIAN CONTROLLER cycle ", k )
                gg=gg*(-1)
                dx=0.01*gg
                h_thumb_x = self.x_sin[k]

                                                # position [x, y, z]  +  quaternion [x, y, z, w]
                ##relax configuration
                finger_poses.append(self.list_to_pose([0.0935, 0.0873, 0.1425, 0.9597, 0.2028, 0.1940, 0.0015])) #finger 0
                finger_poses.append(self.list_to_pose([round(0.1064,2), round(0.0092,2), round(0.1627,2), round(0.9020,2), round(0.0393,2), round(0.4294,2), round(-0.0178,2)])) #finger 1
                finger_poses.append(self.list_to_pose([0.0689, -0.0519, 0.1396,0.9860, -0.0378, 0.1579, -0.0373]))  #finger 2 mignolo
                finger_poses.append(self.list_to_pose([round(0.0687+h_thumb_x ,2) , round(0.1170+h_thumb_x ,2), round(0.0563+h_thumb_x ,2) , 0.1961 + self.quat_thumb_delta_x, 0.0134+self.quat_thumb_delta_y, 0.4522+self.quat_thumb_delta_z, 0.8699+self.quat_thumb_delta_z])) #finger 3
            

                # send the pose to the server
                goal = PoseControlGoal(cartesian_pose=finger_poses)
                client.send_goal(goal, feedback_cb=self.feedbackCallback)
                # to test the feedback mechanism:
                
                self.feedbacks_received = False
                k=k+1
                if k==50:
                    k=0
                #client.wait_for_result()
                rospy.sleep(0.2)

            #client.wait_for_result()

            print("pose_action_client: finished cartesian pose execution")
            print("feedbacks received: "+ str(feedbacks_received))

            rospy.sleep(2.0)
            ###
                                                #### JOINT CONTROLLER ###
            ###
        if joints_controller==True:
            client.wait_for_server()
            print("JOINT CONTROLLER")
            k=0
            while not rospy.is_shutdown(): #finger 3=thumb  2 = pinkie 1= middle 0=index?
                # create a joint pose for testing
                # 'point1' pose from joint_poses.yaml
                gg=gg*(-1)
                dx=0#0.2*gg
                #h_thumb_x = x_sin[k]
                #print(h_thumb_x)

                joint_pose = \
                    [0.06+dx, 0.50+dx, 0.13+dx, -0.14+dx,\
                    0.0, 1.32, 1.72, 0.0,\
                    0.0+dx, 1.32+dx, 1.72+dx, 0.0+dx,\
                    0.72+dx, 0.01+dx, 0.40+dx, 1.5+dx]

                #print("joint_pose",joint_pose)
                # send the pose to the server
                goal = PoseControlGoal(joint_pose=joint_pose)
                client.send_goal(goal, feedback_cb=self.feedbackCallback)
                # to test the feedback mechanism:
                feedbacks_received = False
                #k=k+1
            # if k==50:
            #    k=0
                rospy.sleep(0.2)#/////############### SOPRA LO 0.2(5hz) FUNZIONA ABBASTANZA BENE, best at 0.5(2hz)# ovviamente questo e' perche gli sto chiedendo di fare 0.4 radianti istantaneamente

                #client.wait_for_result()

            print("pose_action_client: finished joint pose execution")
            print("feedbacks received: "+ str(feedbacks_received))


    def __OnOutVirtuosePoseReceived(self,out_thumb_msg):
        
        #msg=out_virtuose_pose()
        #msg.virtuose_pose.translation.x

        
        out_thumb_msg.virtuose_pose.translation.x

        #save the initial pose
        if self.initial_pose_flag==False:
            self.initial_pose_msg=out_thumb_msg
            self.initial_pose_flag=True
            print("INITIAL POSE",self.initial_pose_msg)

        #calculate delta pose of current pose wrt initial pose
        elif self.initial_pose_flag ==True:

            self.q1_inv[0] = self.initial_pose_msg.virtuose_pose.rotation.x
            self.q1_inv[1] = self.initial_pose_msg.virtuose_pose.rotation.y
            self.q1_inv[2] = self.initial_pose_msg.virtuose_pose.rotation.z
            self.q1_inv[3] = -(self.initial_pose_msg.virtuose_pose.rotation.w) # Negate for inverse http://wiki.ros.org/tf2/Tutorials/Quaternions#Relative_rotations

            self.q2[0] = out_thumb_msg.virtuose_pose.rotation.x
            self.q2[1] = out_thumb_msg.virtuose_pose.rotation.y
            self.q2[2] = out_thumb_msg.virtuose_pose.rotation.z
            self.q2[3] = out_thumb_msg.virtuose_pose.rotation.w

            rotation=tf.transformations.quaternion_multiply(self.q2, self.q1_inv)


            self.delta_pose_msg.virtuose_pose.translation.x = round(self.initial_pose_msg.virtuose_pose.translation.x - out_thumb_msg.virtuose_pose.translation.x,2)
            self.delta_pose_msg.virtuose_pose.translation.y = round(self.initial_pose_msg.virtuose_pose.translation.y - out_thumb_msg.virtuose_pose.translation.y,2)
            self.delta_pose_msg.virtuose_pose.translation.z = round(self.initial_pose_msg.virtuose_pose.translation.z - out_thumb_msg.virtuose_pose.translation.z,2)
            self.delta_pose_msg.virtuose_pose.rotation.x=round(rotation[0],2)
            self.delta_pose_msg.virtuose_pose.rotation.y=round(rotation[1],2)
            self.delta_pose_msg.virtuose_pose.rotation.z=round(rotation[2],2)
            self.delta_pose_msg.virtuose_pose.rotation.w=round(rotation[3],2)

            #print("delta_pose_msg",self.delta_pose_msg)
            self.delta_pose_pub.publish(self.delta_pose_msg)
    

    def feedbackCallback(self,feedback):
        
        self.feedbacks_received = True

        rospy.logdebug("pose_action_client: got feedback:")
        rospy.logdebug(feedback)


    def list_to_pose(self,list):
        pose = Pose()
        pose.position.x = list[0]
        pose.position.y = list[1]
        pose.position.z = list[2]
        pose.orientation.x = list[3]
        pose.orientation.y = list[4]
        pose.orientation.z = list[5]
        pose.orientation.w = list[6]
        return pose

if __name__ == '__main__':
    try:
        rospy.init_node('pose_action_client')
        inputmanager = PoseActionClient(None)
        rospy.spin()
    except rospy.ROSInterruptException:
        print("pose_action_client: interrupted by signal")
        print("pose_action_client: terminating")