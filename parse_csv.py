#!/usr/bin/env python

import numpy as np
import transforms3d as td
import pandas as pd
import datetime
from sklearn.decomposition import PCA
import re
import argparse


class CSVParser(object):
    def __init__(self):
        self.conditions = []
        self.calibration_conditions = []
        self.df = None
        self.calibration_df = None

    def file_to_df(self, filename):
        df = pd.read_csv(filename, low_memory=False)
        df.drop(columns = ['Unnamed: 0'], inplace = True)
        df.sort_values(by=['time'], inplace = True, ignore_index = True)
        df['trakstar0'] = df['trakstar0'].apply(self.parse_transform)
        df['trakstar1'] = df['trakstar1'].apply(self.parse_transform)
        df['trakstar2'] = df['trakstar2'].apply(self.parse_transform)
        df['trakstar_01'] = [np.dot(np.linalg.inv(t1), t2) for t1, t2 in zip(df['trakstar0'], df['trakstar1'])]
        df['trakstar_02'] = [np.dot(np.linalg.inv(t1), t2) for t1, t2 in zip(df['trakstar0'], df['trakstar2'])]
        df['trakstar_02_x'] = [t[:3, 3][0] for t in df['trakstar_02']]
        df['trakstar_02_y'] = [t[:3, 3][1] for t in df['trakstar_02']]
        df['trakstar_02_z'] = [t[:3, 3][2] for t in df['trakstar_02']]
        df['time_unitless'] = df.index
        df["mcp_angle"] = np.empty(len(df.index))
        df["pip_angle"] = np.empty(len(df.index))
        
        self.df = df

    def parse_conditions(self):
                
        conditions = list(self.df['condition'].unique())
        for condition in conditions:
            if 'calibration' in condition:
                self.calibration_conditions.append(condition)

        self.conditions = set(conditions) - set(self.calibration_conditions)

    def parse_transform(self, transform_str):
        # Split the string using regex
        result = re.split(r'[:\n]', transform_str)

        # Remove empty strings from the result
        result = [item.strip() for item in result if item.strip()]
        x = float(result[2])
        y = float(result[4])
        z = float(result[6])
        rx = float(result[9])
        ry = float(result[11])
        rz = float(result[13])
        rw = float(result[15])
        # Display the result
        return td.affines.compose([x,y,z], td.quaternions.quat2mat([rw, rx, ry, rz]), np.ones(3))

    def relative_transform(self, t1, t2):
        return np.dot(np.linalg.inv(t1), t2)

    def get_calibration_pca(self, calibration):
        '''
        input: dataframe & column for calibration
        '''
        data = self.df[self.df['condition'] == calibration]['trakstar_02']
        n = len(data)
        points = np.zeros((n, 3))

        for i, transform in enumerate(data):
            translation = transform[:3, 3]
            points[i, :3] = translation

        pca = PCA()
        pca.fit(points)

        return pca

    def twist_rotation_about_axis(self, transform, axis):
        '''
        Returns the angle theta in degrees about the twist axis. 
        '''
        # quaternion convention is [w, x, y, z]
        transform_quat = td.quaternions.mat2quat(transform[:3, :3])
        transform_axes = np.array([transform_quat[1], transform_quat[2], transform_quat[3]])
        
        dot_product = np.dot(transform_axes, axis)
        dp_sign = np.sign(dot_product)
        axis_norm = np.linalg.norm(axis)
        projection = (dot_product / axis_norm**2) * axis 
        
        # define twist in quaternion form and normalize
        twist = np.array([transform_quat[0], projection[0], projection[1], projection[2]])
        twist /= td.quaternions.qnorm(twist)
        
        # catching singularities
        threshold = 1e-6
        if td.quaternions.qnorm(twist) < threshold:
            print("Singularity in twist")
            return
        elif td.quaternions.qnorm(transform_quat) < threshold:
            print("Singularity in rotation")
            return
        
        twist = dp_sign * twist
        twist_axis, twist_theta = td.quaternions.quat2axangle(twist)

        if not np.allclose(axis, twist_axis):
            print("Axis of rotation is not the given axis. Something went wrong.")
            print("twist axis: %s"%(twist_axis))
            print("pca axis  : %s"%(axis))
        
        return dp_sign*twist_theta*(180/np.pi)

    def calculate_relative_angle(self, ref_transform, target_transform, axis):
        
        net_transform = np.dot(np.linalg.inv(ref_transform), target_transform)
        target_theta = self.twist_rotation_about_axis(net_transform, axis)
        theta_sign = np.sign(target_theta)
        delta = abs(target_theta)
        theta = min(delta, 360 - delta)

        return theta_sign * theta

    def get_calibration_df(self):
        
        calibration_data = []

        for condition in self.calibration_conditions:
            PCA = self.get_calibration_pca(condition)
            pca_axes = PCA.components_
            pca_variance = PCA.explained_variance_ratio_
            mcp_ref_rotation = self.df[self.df['condition'] == condition]['trakstar_01'].iloc[0][:3, :3]
            pip_ref_rotation = self.df[self.df['condition'] == condition]['trakstar_02'].iloc[0][:3, :3]
            calibration_row = {'condition':condition, 
                            'mcp_ref_rotation': mcp_ref_rotation, 
                            'pip_ref_rotation': pip_ref_rotation, 
                            'PCA_axis1':pca_axes[0], 
                            'PCA_axis2':pca_axes[1], 
                            'PCA_axis3':pca_axes[2], 
                            'variance': pca_variance}

            calibration_data.append(calibration_row)
        
        calibration_df = pd.DataFrame(calibration_data)
        self.calibration_df = calibration_df

    def calculate_joint_angles(self, df, calibration_df, calibration_condition, condition):
        
        joint_axis = calibration_df[calibration_df['condition']==calibration_condition].loc[0, 'PCA_axis3']
        # pick the first transform of the dataset as the neutral pose
        mcp_ref_rotation = calibration_df[calibration_df['condition']==calibration_condition].loc[0, 'mcp_ref_rotation']
        pip_ref_rotation = calibration_df[calibration_df['condition']==calibration_condition].loc[0, 'pip_ref_rotation']

        # Rotate joint axis into frame
        mcp_axis = np.dot(np.linalg.inv(mcp_ref_rotation), joint_axis)
        pip_axis = np.dot(np.linalg.inv(pip_ref_rotation), joint_axis)
        print("Conditions: %s"%condition)
        print("\nPCA axis of least variation: %s"%(joint_axis))
        print("                   MCP axis: %s \n                   PIP axis: %s"%(mcp_axis, pip_axis))
        print("----------------------------\n")

        # Calculate the relative angle using the pre-defined reference transform and joint axis
        mcp_angle = [-1*self.calculate_relative_angle(mcp_ref_rotation, t[:3, :3], mcp_axis) for t in df[df['condition'] == condition]['trakstar_01']]
        finger_angle = [-1*self.calculate_relative_angle(pip_ref_rotation, t[:3, :3], pip_axis) for t in df[df['condition'] == condition]['trakstar_02']]
        # Find PIP angle using the difference 
        pip_angle = np.array(np.array([finger_angle]) - np.array([mcp_angle]))[0]

        indices = list(df[df['condition'] == condition].index)

        df.loc[indices, 'mcp_angle'] = mcp_angle
        df.loc[indices, 'pip_angle'] = pip_angle
        
        return df

    def get_calibrated_joint_angles(self, multiple_calibration = True):

        for condition in self.conditions: 
            if multiple_calibration == False:
                df = self.calculate_joint_angles(self.df, self.calibration_df, 'calibration', condition)
            else:
                df = self.calculate_joint_angles(self.df, self.calibration_df, condition + '_calibration', condition)

        df.drop(columns=["trakstar0", "trakstar1", "trakstar2", "trakstar_01", "trakstar_02"], inplace = True)

        self.df = df

def main(filename):
    csv_file = CSVParser()
    csv_file.file_to_df(filename)
    csv_file.parse_conditions()
    csv_file.get_calibrated_joint_angles(multiple_calibration = False)

