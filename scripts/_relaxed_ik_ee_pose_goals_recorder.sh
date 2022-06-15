SUBJECT='_relaxed_ik_ee_pose_goals'
BAGNAME='experiment_recordings_'${SUBJECT}
rosbag record -O ~/.ros/rosbags/${BAGNAME}.bag /relaxed_ik/ee_pose_goals
