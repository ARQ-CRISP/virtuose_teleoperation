import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split, cross_val_score
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report, accuracy_score, confusion_matrix
import joblib
import os
import matplotlib.pyplot as plt
import seaborn as sns

def load_data(base_path):
    all_data = pd.DataFrame()
    subfolders = ['brick', 'no_contact', 'rice', 'tom', 'us', 'ys']
    for subfolder in subfolders:
        subfolder_path = os.path.join(base_path, 'processed', subfolder)
        if os.path.isdir(subfolder_path):
            for csv_file in os.listdir(subfolder_path):
                if csv_file.endswith('.csv'):
                    file_path = os.path.join(subfolder_path, csv_file)
                    df = pd.read_csv(file_path)
                    df['label'] = subfolder
                    all_data = pd.concat([all_data, df], ignore_index=True)
    return all_data

# Load the extracted data
base_path = '/home/hair/stiffness_estimation'
data = load_data(base_path)

# Handle missing force data (if any)
data.fillna(0, inplace=True)

# Features for training
features = [
    # 'index_position_x', 'index_position_y', 
    # 'index_position_z',
    # 'index_orientation_x', 'index_orientation_y', 'index_orientation_z', 'index_orientation_w',
    # 'middle_position_x', 'middle_position_y', 
    # 'middle_position_z',
    # 'middle_orientation_x', 'middle_orientation_y', 'middle_orientation_z', 'middle_orientation_w',
    # 'ring_position_x', 'ring_position_y', 
    # 'ring_position_z',
    # 'ring_orientation_x', 'ring_orientation_y', 'ring_orientation_z', 'ring_orientation_w',
    # 'thumb_position_x', 'thumb_position_y', 
    # 'thumb_position_z',
    # 'thumb_orientation_x', 'thumb_orientation_y', 'thumb_orientation_z', 'thumb_orientation_w',
    # 'force_x_index', 'force_y_index', 
    'force_z_index',
    # 'force_x_middle', 'force_y_middle', 
    # 'force_z_middle',
    # 'force_x_ring', 'force_y_ring', 
    'force_z_ring',
    # 'force_x_thumb', 'force_y_thumb', 
    'force_z_thumb',
    # 'torque_norm_index', 'torque_norm_middle', 'torque_norm_ring', 'torque_norm_thumb',
    # 'stiffness_index', 'stiffness_middle', 'stiffness_ring', 'stiffness_thumb'
]

# Preprocessing
X = data[features]
y = data['label']

# Check the distribution of the labels
print("Label distribution in the complete dataset:")
print(y.value_counts())

# Split the data with stratification
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42, stratify=y)

# Check the distribution of the labels in the training and test sets
print("Label distribution in the training set:")
print(y_train.value_counts())
print("Label distribution in the test set:")
print(y_test.value_counts())

# Train the classifier
clf = RandomForestClassifier(n_estimators=100, random_state=42)
clf.fit(X_train, y_train)

# Test the classifier
y_pred = clf.predict(X_test)
print(classification_report(y_test, y_pred))
print("Accuracy:", accuracy_score(y_test, y_pred))

# Save the model
joblib.dump(clf, 'stiffness_classifier.pkl')

# Plot the normalized confusion matrix for the standard model
conf_matrix = confusion_matrix(y_test, y_pred)
conf_matrix_normalized = conf_matrix.astype('float') / conf_matrix.sum(axis=1)[:, np.newaxis]
plt.figure(figsize=(10, 6))
sns.heatmap(conf_matrix_normalized, annot=True, fmt='.2f', cmap='Blues', xticklabels=np.unique(y), yticklabels=np.unique(y))
plt.title('Normalized Confusion Matrix - Standard Model')
plt.xlabel('Predicted Label')
plt.ylabel('True Label')
plt.show()

# Add noise to the training data
noise_factor = 0.02
X_train_noisy = X_train + noise_factor * np.random.normal(size=X_train.shape)

# Train the classifier with noisy data
clf_noisy = RandomForestClassifier(n_estimators=100, random_state=42)
clf_noisy.fit(X_train_noisy, y_train)

# Test the classifier with noisy data
y_pred_noisy = clf_noisy.predict(X_test)
print("Classification report with noisy training data:")
print(classification_report(y_test, y_pred_noisy))
print("Accuracy with noisy training data:", accuracy_score(y_test, y_pred_noisy))

# Save the noisy model
joblib.dump(clf_noisy, 'stiffness_classifier_noisy.pkl')

# Plot the normalized confusion matrix for the noisy model
conf_matrix_noisy = confusion_matrix(y_test, y_pred_noisy)
conf_matrix_noisy_normalized = conf_matrix_noisy.astype('float') / conf_matrix_noisy.sum(axis=1)[:, np.newaxis]
plt.figure(figsize=(10, 6))
sns.heatmap(conf_matrix_noisy_normalized, annot=True, fmt='.2f', cmap='Blues', xticklabels=np.unique(y), yticklabels=np.unique(y))
plt.title('Normalized Confusion Matrix - Noisy Model')
plt.xlabel('Predicted Label')
plt.ylabel('True Label')
plt.show()

# Perform K-Fold Cross-Validation
scores = cross_val_score(clf, X, y, cv=5)
print("Cross-Validation Scores:", scores)
print("Mean Accuracy:", scores.mean())
