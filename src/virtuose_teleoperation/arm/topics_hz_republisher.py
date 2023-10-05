#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped 
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped,WrenchStamped
from std_msgs.msg import String, Float64, Header
from visualization_msgs.msg import Marker
from relaxed_ik.msg import EEPoseGoals, JointAngles
import rospy
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from rospy import Time
from tf2_msgs.msg import TFMessage
from virtuose.msg import out_virtuose_pose,out_virtuose_physical_pose,in_virtuose_force
class CustomTopicPublisher:

    frequencies = [10, 10, 10,10,
    10,  
    10, 10, 10, 10,
    10, 10, 10,
    10, 10, 10,
    10, 
    10,10, 10]

    def __init__(self):
        rospy.init_node('custom_topic_publisher', anonymous=True)
        self.pubs = []
        self.subs = []

        for i, topic in enumerate(['/Crisp_IN_2HGlove', '/Crisp_MID_2HGlove', '/Crisp_RING_2HGlove','/Crisp_TH_2HGlove', 
        '/allegroHand_0/joint_states', 
        '/in_index_force','/in_middle_force', '/in_thumb_force', '/in_virtuose_force',
        '/out_index_pose', '/out_middle_pose', '/out_thumb_pose',
        '/delta_pose_index', '/delta_pose_middle', '/delta_pose_thumb',
        '/out_virtuose_physical_pose',
        '/relaxed_ik/ee_pose_goals','/relaxed_ik/joint_angle_solutions', '/ur5/tf']):

            msg_type = [WrenchStamped, WrenchStamped, WrenchStamped,WrenchStamped,
                JointState,
                in_virtuose_force,in_virtuose_force,in_virtuose_force,in_virtuose_force,
                out_virtuose_pose,out_virtuose_pose,out_virtuose_pose,
                out_virtuose_pose,out_virtuose_pose,out_virtuose_pose,
                out_virtuose_physical_pose,
                EEPoseGoals,JointAngles,TFMessage][i]
        
        
        

        # for i, topic in enumerate(['/relaxed_ik/ee_pose_goals']):
        #     msg_type = [EEPoseGoals][i]
            pub_topic = 'zz_customFreq_' + topic  
            self.pubs.append(rospy.Publisher(pub_topic, msg_type, queue_size=10))
            self.subs.append(rospy.Subscriber(topic, msg_type, self.callback, callback_args=(i,)))
            self.last_pub_times = [Time.now()] * len(self.pubs)

    def callback(self, data, cb_args):
        idx = cb_args[0]
        # Filter the messages according to the desired frequency
        if (Time.now() - self.last_pub_times[idx]).to_sec() >= 1.0/self.frequencies[idx]:
            self.pubs[idx].publish(data)
            self.last_pub_times[idx] = Time.now()

if __name__ == '__main__':
    try:
        CustomTopicPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
