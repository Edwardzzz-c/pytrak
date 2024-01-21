#!/usr/bin/env python

import transforms3d as td
import numpy as np
import pickle
import argparse
from sklearn.decomposition import PCA

class Calibrator(object):

	def __init__(self):
		self.calibration_info = {}
		pass

	def load_file(self, filename):
		'''
		Loads in the calibration poses, stored as lists: 
			neutral_poses  : list of 10 neutral poses
			sweeping_poses : 40 transforms representing the full range of motion
			test_poses     : transforms at pre-specified angles
		'''
		try:
			file = open(filename, 'rb')
			self.neutral_poses = pickle.load(file)
			self.sweeping_poses = pickle.load(file)

		except IOError:
			print("Failed to read file")
			return False
		return True
	
	def get_pca_least_var_axis(self, data):
		n = len(data)
		points = np.zeros((n, 3))
		
		for i, transform in enumerate(data):
			translation = transform[:3, 3]
			points[i, :3] = translation

		pca = PCA()
		pca.fit(points)
		print("Variance explained: %s"%(pca.explained_variance_ratio_))
		print("Plane normal: \n %s"%(pca.components_))
		print("Axis of least variation: %s"%(pca.components_[2,:]))
		return pca.components_[2,:]
	
	def save_calibration_information(self, outfile_name):
		'''
		Saves calibration information dictionary, `calibration_info`, into a file. 
		'''
		sensor_2_sweeping_poses = self.sweeping_poses["sensor_2"]
		print(len(sensor_2_sweeping_poses))
		joint_axis = self.get_pca_least_var_axis(sensor_2_sweeping_poses)

		# re-orient joint-axis to get correct direction from Sensor 0 to Sensor 1
		#joint_axis = -1*joint_axis
		
		self.calibration_info["joint_axis"] = joint_axis
		self.calibration_info["sensor_1_neutral_pose"] = self.neutral_poses["sensor_1"][2]
		self.calibration_info["sensor_2_neutral_pose"] = self.neutral_poses["sensor_2"][2]

		outfile = open(outfile_name, 'wb')
		pickle.dump(self.calibration_info, outfile)
		outfile.close()
		print("Saved calibration.")
											   
def main(args):
	calibrator = Calibrator()
	if not calibrator.load_file(args.file):
		exit()
	calibrator.save_calibration_information(args.outfile)

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--file', type=str, default = 'calibration_poses')
	parser.add_argument('--outfile', type=str, default = 'calibration_info')
	main(parser.parse_args())

