#!/usr/bin/env python

import rosbag
import pandas as pd
import glob
from datetime import datetime
import rospy
import transforms3d as td
import argparse
import numpy as np
import re

class RosbagParser(object):
    def __init__(self):
        self.futek_list = []
        self.trakstar_list = []

    def rosbag_parser(self, filepath):
        try:
            files = glob.glob(filepath + '/*')
            bag = rosbag.Bag(files[0])
        except IOError:
            print("Failed to open files")
            return False

        for i, file in enumerate(files): 
            bag = rosbag.Bag(file)
            print("File %s of %s total files."%(i+1, len(files)))
            topics = ['/tf', '/futek']
            condition = re.findall("\d[_]([a-z_+]+)[_]\d", file)
            
            for topic, msg, t in bag.read_messages(topics=topics):

                if topic == '/tf':
                    ros_time = rospy.Time(t.secs, t.nsecs)
                    timestamp = ros_time.to_sec()
                    date_time = datetime.utcfromtimestamp(round(timestamp, 2))
                    formatted_time = date_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-4]
                    
                    T_0 = self.to_affine(msg.transforms[0])
                    T_1 = self.to_affine(msg.transforms[1])
                    T_2 = self.to_affine(msg.transforms[2])
            
                    self.trakstar_list.append(
                        {'condition': condition, 
                         'raw_time': t,
                         'time': formatted_time,
                         'sensor_0': T_0,
                         'sensor_1': T_1,
                         'sensor_2': T_2}
                    )
                elif topic == '/futek':
                    ros_time = rospy.Time(t.secs, t.nsecs)
                    timestamp = ros_time.to_sec()
                    date_time = datetime.utcfromtimestamp(round(timestamp, 2))
                    formatted_time = date_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-4]
                    
                    force = msg.load
                    self.futek_list.append(
                        {'condition': condition, 
                         'raw_time': t,
                         'time': formatted_time,
                         'force': force}
                    )
        print("Finished parsing data.")

    def save_data(self, path, filename):
        print("saving data")
        trakstar_df = pd.DataFrame(self.trakstar_list)
        futek_df = pd.DataFrame(self.futek_list)
        df = trakstar_df.merge(futek_df, on='time', how='left')
        df.to_csv(path + '/' + filename + '.csv')
        print("Saved data parsing file.")

    def to_affine(self, t):
        '''
        Returns the 4x4 homogenous transform from the Transform message. 
        '''
        T = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        rotation = [t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z]
        R = td.quaternions.quat2mat(rotation)
        Z = np.ones(3)

        return td.affines.compose(T, R, Z)

def main(args):
    ros_bag = RosbagParser()
    ros_bag.rosbag_parser(args.filepath)
    ros_bag.save_data(args.filepath, args.outfile)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--filepath', type=str)
    parser.add_argument('--outfile', type=str, default = 'data')
    main(parser.parse_args())
