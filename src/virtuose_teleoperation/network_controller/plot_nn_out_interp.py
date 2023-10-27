import os
import pandas as pd
import matplotlib.pyplot as plt

# Define the folder path
folder_path = r'C:\catkin_ws'

# File paths
nn_joints_pose_file = os.path.join(folder_path, 'nn_joints_pose.csv')
nn_joints_pose_interpolation_file = os.path.join(folder_path, 'nn_joints_pose_interpolation.csv')

# Read data from CSV files
nn_joints_pose_df = pd.read_csv(nn_joints_pose_file)
nn_joints_pose_interpolation_df = pd.read_csv(nn_joints_pose_interpolation_file)

# Get the column names
columns_nn_joints_pose = nn_joints_pose_df.columns
columns_nn_joints_pose_interpolation = nn_joints_pose_interpolation_df.columns

# Plot corresponding columns from the two dataframes
for col_nn_joints_pose, col_nn_joints_pose_interpolation in zip(columns_nn_joints_pose, columns_nn_joints_pose_interpolation):
    plt.figure(figsize=(8, 6))
    plt.plot(nn_joints_pose_df[col_nn_joints_pose], nn_joints_pose_interpolation_df[col_nn_joints_pose_interpolation], marker='o', linestyle='-', color='b')
    plt.xlabel(f'Column: {col_nn_joints_pose} (nn_joints_pose.csv)')
    plt.ylabel(f'Column: {col_nn_joints_pose_interpolation} (nn_joints_pose_interpolation.csv)')
    plt.title(f'Plotting {col_nn_joints_pose} vs {col_nn_joints_pose_interpolation}')
    plt.grid(True)
    plt.show()
