SUBJECT='out_virtuose_physical_pose'
BAGNAME='experiment_recordings_'${SUBJECT}
rosbag record -O ~/.ros/rosbags/${BAGNAME}.bag /out_virtuose_physical_pose
