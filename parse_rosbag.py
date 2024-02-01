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
import functools as ft
from pathlib import Path  

class RosbagParser(object):
    def __init__(self):
        self.topic_data = {}

    def filter_time(self, t):
        ros_time = rospy.Time(t.secs, t.nsecs)
        timestamp = ros_time.to_sec()
        date_time = datetime.utcfromtimestamp(round(timestamp, 2))
        formatted_time = date_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-4]

        return timestamp, formatted_time

    def rosbag_parser(self, filepath, folder, topics, file):
        try:
            if len(folder) > 0:
                folder = folder + '/'

            self.total_path = filepath + folder
            if len(file) > 0:
                files = glob.glob(self.total_path + file)
            else:
                files = glob.glob(self.total_path + '*')

            self.topics = [topic_name for topic_name in topics.split(' ')]

        except IOError:
            print("Failed to open files")
            return False

        for topic in self.topics:
            self.topic_data[topic] = []

        for i, file in enumerate(files): 
            bag = rosbag.Bag(file)
            
            print("Parsed file %s of %s total files."%(i+1, len(files)))
            file_name = file[len(filepath + folder):]

            condition = re.findall("(?<=\d_)(.*?)(?=_2024)", file_name)[0]
            if len(condition) == 0:
                condition = "not_recorded"

            for topic, msg, t in bag.read_messages(topics=self.topics):

                if topic == '/tf':
                    tsec, formatted_time = self.filter_time(t)
                    self.topic_data[topic].append(
                        {'condition': condition, 
                         'raw_time': tsec,
                         'time': formatted_time,
                         'trakstar0': str(msg.transforms[0].transform),
                         'trakstar1': str(msg.transforms[1].transform),
                         'trakstar2': str(msg.transforms[2].transform)}
                    )
                elif topic == '/futek':
                    tsec, formatted_time = self.filter_time(t)
                    force = msg.load
                    self.topic_data[topic].append(
                        {'condition': condition, 
                         'raw_time': tsec,
                         'time': formatted_time,
                         'futek': force}
                    )
                elif topic == '/arduino_DCmotor/feedback':
                    tsec, formatted_time = self.filter_time(t)
                    motor_pos = msg.position1
                    self.topic_data[topic].append(
                        {'condition': condition, 
                         'raw_time': tsec,
                         'time': formatted_time,
                         'motor_position': motor_pos}
                    )
                elif topic == '/arduino_DCmotor/button':
                    tsec, formatted_time = self.filter_time(t)
                    button = msg.data
                    self.topic_data[topic].append(
                        {'condition': condition, 
                         'raw_time': tsec,
                         'time': formatted_time,
                         'button': button}
                    )
                elif topic == '/hand_event':
                    tsec, formatted_time = self.filter_time(t)
                    hand_event = msg.data
                    self.topic_data[topic].append(
                        {'condition': condition, 
                         'raw_time': tsec,
                         'time': formatted_time,
                         'hand_event': hand_event}
                    )

        print("Finished parsing data.")

    def save_data(self, name):
        print("Saving data...")
        if len(name) > 0:
            name = name + '_'

        if '/tf' in self.topics and len(self.topics) > 1:
            trakstar_df = pd.DataFrame(self.topic_data['/tf'])
            subtopics = self.topics.copy()
            subtopics.remove('/tf')

            for topic in subtopics:
                df = pd.DataFrame(self.topic_data[topic])
                df.drop(columns = ['condition', 'raw_time'], inplace = True)
                trakstar_df = trakstar_df.merge(df, on='time', how='left')

            filepath = Path(self.total_path + name + 'compiled_data.csv')  
            filepath.parent.mkdir(parents=False, exist_ok=True)  
            trakstar_df.to_csv(filepath)

            print("Saved data parsing file.")

        else:
            for i, topic in enumerate(self.topics):
                df = pd.DataFrame(self.topic_data[topic])
                tp = re.sub("/", "_", topic[1:])
                filepath = Path('~/' + self.total_path + name + tp + "_data.csv")  
                filepath.parent.mkdir(parents=True, exist_ok=True)  
                df.to_csv(filepath) 
            
                print("Saved %s : %s of %s topics parsed."%(topic, i+1, len(self.topics)))

def main(args):
    ros_bag = RosbagParser()
    ros_bag.rosbag_parser(args.filepath, args.folder, args.topics, args.file)
    ros_bag.save_data(args.name)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--filepath', type=str, default = 'hand_orthosis_ws/src/trakstar_ros/collected_data/rosbag/')
    parser.add_argument('--name', type=str, default = '')
    parser.add_argument('--folder', type=str, default = '')
    parser.add_argument('--file', type=str, default = '')
    parser.add_argument('--topics', type=str, default = "/tf /futek /arduino_DCmotor/button /arduino_DCmotor/feedback")

    main(parser.parse_args())
