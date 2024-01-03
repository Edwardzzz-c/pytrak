#!/usr/bin/env python3

import transforms3d as td
import numpy as np
import pickle
import argparse
from sklearn.decomposition import PCA

class Calibrator(object):

	def __init__(self):
		self.calibration_information = {}
		pass

	def load_file(self, filename):
		'''
		Loads in the calibration poses, stored as lists: 
			neutral_poses  : list of 10 neutral poses
			sweeping_poses : 40 transforms representing the full range of motion
			data_poses     : transforms at pre-specified angles
		'''
		try:
			file = open(filename, 'rb')
			self.neutral_poses = pickle.load(file)
			self.sweeping_poses = pickle.load(file)
			self.data_poses = pickle.load(file)
		except IOError:
			print("Failed to read file")
			return False
		return True
	
	def find_finger_plane_normal(self):
		'''
		Calculates the plane of rotation using PCA from the sweeping_poses. 
		'''
		n = len(self.sweeping_poses)
		sweeping_translations = np.zeros((n, 3))

		for i, transform in enumerate(self.sweeping_poses):
			translation = transform[0:3, 3]
			sweeping_translations[i, 0:3] = translation
		
		pca = PCA()
		pca.fit(sweeping_translations)
		print("Variance explained: %s"%(pca.explained_variance_ratio_))
		print("Plane normal: %s"%(pca.components_[2, :]))
		print('')
		self.finger_plane_normal = pca.components_[2, :]

		return True

	def project_transforms_to_plane(self):
		xaxis = self.neutral_poses[0][0:3, 3]
		xaxis = xaxis/np.linalg.norm(xaxis)
		zaxis = self.finger_plane_normal
		yaxis = np.cross(zaxis, xaxis)
		yaxis = yaxis/np.linalg.norm(yaxis)
		xaxis = np.cross(yaxis, zaxis)
		xaxis = xaxis/np.linalg.norm(xaxis)

		R = np.vstack((xaxis, yaxis, zaxis)).T
		plane_rotation = td.affines.compose(np.zeros(3), R, np.ones(3))

		self.neutral_poses_rotated = []
		self.sweeping_poses_rotated = []
		self.data_poses_rotated = []
		total_z = 0
		
		for t in self.neutral_poses:
			self.neutral_poses_rotated.append(np.dot(np.linalg.inv(plane_rotation), t))
		for t in self.data_poses:
			t_rot = np.dot(np.linalg.inv(plane_rotation), t)
			self.data_poses_rotated.append(t_rot)
		for t in self.sweeping_poses:
			rotated_transform = np.dot(np.linalg.inv(plane_rotation), t)
			self.sweeping_poses_rotated.append(rotated_transform)
			total_z += rotated_transform[2, 3]
		
		mean_z = total_z / len(self.sweeping_poses)
		plane_translation = td.affines.compose((0,0,-mean_z), np.eye(3), np.ones(3))

		projected_sweeping_poses = []
		for t in self.sweeping_poses_rotated:
			projected_sweeping_poses.append(np.dot(plane_translation,t))

		R_neutral = self.neutral_poses_rotated[0]

		for t in self.sweeping_poses_rotated:
			theta1 = td.euler.mat2euler(np.dot(np.linalg.inv(R_neutral), t), axes = 'szxy')[0]*(180/np.pi)
			print(theta1)
			print("---")
	
		self.calibration_information["plane_rotation"] = np.linalg.inv(plane_rotation)
		self.calibration_information["plane_translation"] = np.linalg.inv(plane_translation)
		self.calibration_information["neutral_transform"] = self.neutral_poses_rotated[0]

		return True
	
	def save_calibration_information(self, outfile_name):
		'''
		
		'''
		outfile = open(outfile_name, 'wb')
		pickle.dump(self.calibration_information, outfile)
		outfile.close()
		print("Saved calibration")
											   
def main(args):
	calibrator = Calibrator()
	if not calibrator.load_file(args.file):
		exit()
	if not calibrator.find_finger_plane_normal():
		exit()
	if not calibrator.project_transforms_to_plane():
		exit()
	calibrator.save_calibration_information(args.outfile)

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--file', type=str, default = 'calibration_poses')
	parser.add_argument('--outfile', type=str, default = 'calibration_information')
	main(parser.parse_args())

