# Import libraries
import numpy as np
import transforms3d as td
import pandas as pd
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import datetime
from sklearn.decomposition import PCA
import re
from sklearn.linear_model import LinearRegression


class PatientData(object):
    def __init__(self, filename, hand_opens):

        self.file_to_df(filename)
        self.parse_conditions()
        self.calibration_dict = {'passive_start': 'passive_start_calibration', 'active_rorcr': 'active_calibration', 'active_cube' : 'active_calibration', 'passive_end':'passive_end_calibration'}
        self.get_calibration_df()
        self.get_calibrated_joint_angles()

        self.opens_dfs = self.make_condition_df()
        self.stiffnesses = self.make_condition_df()
        self.intercepts = self.make_condition_df()
        self.fit  = self.make_condition_df()
        self.max_emg = self.make_condition_df()
        self.avg_fit = self.make_condition_df()
        self.hand_opens = hand_opens

        self.splice_condition()
        self.get_stats()
        self.calculate_avg_fit()
        
    def file_to_df(self, filename):

        df = pd.read_csv(filename, low_memory=False, index_col=0)
        
        # Remove NaN rows from dataframe
        df = df[~df['motor_position'].isnull()]
        df = df[~df['futek'].isnull()]
        #df = df[~df['emg0'].isnull()]
        df.reset_index(inplace = True, drop = True)

        # Convert motor_position to mm
        df['motor_position'] = df['motor_position'] * 0.007 # encoder to mm

        # Sort df by time
        df['date_time'] = pd.to_datetime(df['time'], format='%Y-%m-%d %H:%M:%S.%f')
        df.drop(columns=['raw_time', 'time'], inplace = True)
        df.sort_values(by=['date_time'], inplace = True, ignore_index = True)
        # Calculate time differences
        df['time_diff'] = df['date_time'].diff().dt.total_seconds()
        df['time'] = df['time_diff'].cumsum()
        df.loc[0, 'time'] = 0
        df.loc[0, 'time_diff'] = 0

        # Re-order columns because I want the time column to be on the left >:)
        cols = df.columns.tolist()
        new_col_order = [cols[0]] + cols[-3:] + cols[1:-3]
        df = df[new_col_order]

        # Parse transforms and create relative transforms
        df['trakstar0'] = df['trakstar0'].apply(self.parse_transform)
        df['trakstar1'] = df['trakstar1'].apply(self.parse_transform)
        df['trakstar2'] = df['trakstar2'].apply(self.parse_transform)
        df['trakstar_01'] = [self.relative_transform(t1, t2) for t1, t2 in zip(df['trakstar0'], df['trakstar1'])]
        df['trakstar_02'] = [self.relative_transform(t1, t2) for t1, t2 in zip(df['trakstar0'], df['trakstar2'])]

        # define df attribute
        self.df = df

    def parse_conditions(self):
        '''
        Get all conditions from dataset.
        '''
        conditions = list(self.df['condition'].unique())
        calibration_conditions = []
        for condition in conditions:
            if 'calibration' in condition:
                calibration_conditions.append(condition)

        # define conditions attributes
        self.calibration_conditions = calibration_conditions
        self.conditions = [condition for condition in conditions if condition not in calibration_conditions]

    def parse_transform(self, transform_str):
        '''
        Hard-coded parser to convert tf.msg into numpy array.
        '''
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

    def get_calibration_pca(self, calibration, calibration_transform = 'trakstar_02'):
        '''
        input: dataframe & column for calibration
        Calculates PCA for the set of points for the calibration sweep. 
        Uses the transform from the hand to the finger (trakstar_02) to define the plane.
        Change `calibration_transform` to use a different column for the PCA plane.
        '''
        data = self.df[self.df['condition'] == calibration][calibration_transform]
        n = len(data)
        points = np.zeros((n, 3))

        for i, transform in enumerate(data):
            translation = transform[:3, 3]
            points[i, :3] = translation

        pca = PCA()
        pca.fit(points)

        return pca
    
    def get_calibration_df(self, mcp_ref_transform = 'trakstar_01', pip_ref_transform = 'trakstar_02', ref_transform_num = 0):
        '''
        Creates the calibration dataframe by calculating PCA for each condition
        '''
        calibration_data = []

        for condition in self.calibration_conditions:
            PCA = self.get_calibration_pca(condition)
            pca_axes = PCA.components_
            pca_variance = PCA.explained_variance_ratio_
            mcp_ref_rotation = self.df[self.df['condition'] == condition][mcp_ref_transform].iloc[ref_transform_num][:3, :3]
            # note--pip_ref_rotation is actually calculating the finger angle, which is then used to calculate the pip angle
            pip_ref_rotation = self.df[self.df['condition'] == condition][pip_ref_transform].iloc[ref_transform_num][:3, :3]
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
        
        return twist_theta*(180/np.pi)*dp_sign

    def calculate_relative_angle(self, ref_transform, target_transform, axis):
        
        net_transform = self.relative_transform(ref_transform, target_transform)
        target_theta = self.twist_rotation_about_axis(net_transform, axis)
        theta_sign = np.sign(target_theta)
        delta = abs(target_theta)
        theta = min(delta, 360 - delta)

        return theta * theta_sign

    def calculate_joint_angles(self, calibration_condition, condition, pca_axis = 'PCA_axis3'):
        
        joint_axis = self.calibration_df[self.calibration_df['condition']==calibration_condition][pca_axis].iloc[0]

        # pick the first transform of the dataset as the neutral pose
        mcp_ref_rotation = self.calibration_df[self.calibration_df['condition']==calibration_condition]['mcp_ref_rotation'].iloc[0]
        pip_ref_rotation = self.calibration_df[self.calibration_df['condition']==calibration_condition]['pip_ref_rotation'].iloc[0]
        # Rotate joint axis into frame
        mcp_axis = self.relative_transform(mcp_ref_rotation, joint_axis)
        pip_axis = self.relative_transform(pip_ref_rotation, joint_axis)
        print("Condition: %s"%condition)
        print("\nVariance ratio: %s"%self.calibration_df[self.calibration_df['condition']==calibration_condition]['variance'].iloc[0])
        #print(joint_axis)
        #print(mcp_axis)
        #print(pip_axis)
        print("----------------------------\n")

        # Calculate the relative angle using the pre-defined reference transform and joint axis
        mcp_angle = [self.calculate_relative_angle(mcp_ref_rotation, t[:3, :3], mcp_axis) for t in self.df[self.df['condition'] == condition]['trakstar_01']]
        finger_angle = [self.calculate_relative_angle(pip_ref_rotation, t[:3, :3], pip_axis) for t in self.df[self.df['condition'] == condition]['trakstar_02']]
        # Find PIP angle using the difference 
        pip_angle = np.array(np.array([finger_angle]) - np.array([mcp_angle]))[0]
        indices = list(self.df[self.df['condition'] == condition].index)

        self.df.loc[indices, 'mcp_angle'] = mcp_angle
        self.df.loc[indices, 'pip_angle'] = pip_angle
        self.df.loc[indices, 'finger_angle'] = finger_angle

    def get_calibrated_joint_angles(self):
        for condition in self.conditions: 
            self.calculate_joint_angles(self.calibration_dict[condition], condition)
    
    def make_condition_df(self):
        return {c: [] for c in self.conditions}
    
    def splice_condition(self):
        '''
        Create the dataframe slice for each condition.
        Also calls the splice_opens and calculate_opens as it iterates over the conditions
        '''
        self.full_condition_dfs = {}
        self.condition_dfs = {}
        for condition in self.conditions:
            # Get slice from main dataframe
            df_slice = self.df[self.df['condition'] == condition].copy()
            df_slice.reset_index(inplace=True, drop = True)
            # Calculate correct time 
            df_slice.loc[0, 'time_diff'] = 0
            df_slice['time_condition'] = df_slice['time_diff'].cumsum()
            # reorder columns to my liking hehe
            cols = df_slice.columns.to_list()
            cols = cols[:4] + [cols[-1]] + cols[4:-1]
            df_slice = df_slice[cols]
            # add dataframe slice to the dictionary
            self.full_condition_dfs[condition] = df_slice
            
            # Splice out opens! 
            self.splice_opens(condition)
            # Calculate best fit per hand open
            self.calculate_opens(condition)

            # Concat the hand open dfs to make the condition_df
            condition_df = pd.concat(self.opens_dfs[condition], axis=0)
            condition_df.reset_index(inplace=True)
            condition_df.rename(columns={'level_0' : 'open_index', 'index' : 'full_index'}, inplace=True)
            self.condition_dfs[condition] = condition_df
    
    def splice_opens(self, condition):
        '''
        Create dataframe slices from the manually spliced hand_opens dict
        '''
        for i, j in self.hand_opens[condition]:
            # Get hand opening dataframe slice
            df_slice = self.full_condition_dfs[condition].loc[np.r_[i:j]].copy()
            df_slice.reset_index(inplace=True, drop=False)
            # Calculate correct time 
            df_slice.loc[0, 'time_diff'] = 0
            df_slice['time_open'] = df_slice['time_diff'].cumsum()
            # reorder columns to my liking hehe
            cols = df_slice.columns.to_list()
            cols = cols[:6] + [cols[-1]] + cols[6:-1]
            df_slice = df_slice[cols]
            self.opens_dfs[condition].append(df_slice)
    
    def calculate_opens(self, condition):
        '''
        Calculate the best fit line for every hand opening.
        '''
        for i, df in enumerate(self.opens_dfs[condition]):
            # Calculate best fit
            df_, m, b = self.calculate_fit(df.copy())
            self.opens_dfs[condition][i] = df_
            
            self.stiffnesses[condition].append(m)
            self.intercepts[condition].append(b)
            self.fit[condition].append((m, b))

            # Get max emg value
            self.max_emg[condition].append(self.get_max_emg(df.copy()))

    def calculate_fit(self, df):
        '''
        Calculate the best fit line for a single hand opening.
        '''
        x_start = df.loc[0,'motor_position']
        y_start = df.loc[0,'futek']

        df['displacement'] = df['motor_position'] - x_start
        df['net_force'] = df['futek'] - y_start

        x_vals = df['displacement'].values.reshape(-1, 1)
        y_vals = df['net_force'].values

        model = LinearRegression()
        model.fit(x_vals, y_vals)
        m = model.coef_[0]
        b = model.intercept_

        return df, m, b

    def get_max_emg(self, df):
        '''
        Calculates the max emg value reached during one hand opening
        '''
        emg_vals = df[['emg' + str(i) for i in range(8)]].values
        emg_vals = emg_vals[~np.isnan(emg_vals)]

        return emg_vals.mean()

    def get_stats(self):
        dfs = []

        for c in self.conditions:
            dfs.append(pd.DataFrame(self.stiffnesses[c]).describe().rename(columns = {0 : c}))

        self.stats = pd.concat(dfs, axis = 1)
        
    def calculate_avg_fit(self):
        for c in self.conditions:
            m = np.mean(self.stiffnesses[c])
            b = np.mean(self.intercepts[c])
            self.avg_fit[c] = (m, b)
