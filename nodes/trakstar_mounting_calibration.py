#!/usr/bin/env python

import transforms3d as td
import numpy as np
import pickle
import argparse
from sklearn.decomposition import PCA

class Calibrator(object):

	def __init__(self):
		self.calibration_information = {}
		self.normal_plane = {}
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
	
	def find_finger_plane_normal(self, joint):
		'''
		Calculates the plane of rotation using PCA from the sweeping_poses. 
		'''

		n = len(self.sweeping_poses[joint])
		sweeping_translations = np.zeros((n, 3))

		for i, transform in enumerate(self.sweeping_poses[joint]):
			translation = transform[0:3, 3]
			sweeping_translations[i, 0:3] = translation
		
		pca = PCA()
		pca.fit(sweeping_translations)
		print(joint)
		print("Variance explained: %s"%(pca.explained_variance_ratio_))
		print("Plane normal: %s"%(pca.components_[2, :]))
		print("--")
		self.normal_plane[joint] = pca.components_[2, :]

	def rotate_to_plane(self, joint):
		'''
			calibration_information : dictionary with plane_rotation, plane_translation, and neutral_transform
		'''

		# Take the translation component of the first neutral pose 
		xaxis = self.neutral_poses[joint][0][0:3, 3]
		xaxis = xaxis/np.linalg.norm(xaxis)
		# Obtain the finger_plane_normal previously calculated from PCA
		zaxis = self.normal_plane[joint]
		# Calculate y-axis by taking the cross product 
		yaxis = np.cross(zaxis, xaxis)
		yaxis = yaxis/np.linalg.norm(yaxis)
		# Re-define the x-axis with the resulting y-axis 
		xaxis = np.cross(yaxis, zaxis)
		xaxis = xaxis/np.linalg.norm(xaxis)

		# Define R, the rotation matrix with the new coordinate axes 
		R = np.vstack((xaxis, yaxis, zaxis)).T
		plane_rotation = td.affines.compose(np.zeros(3), R, np.ones(3))

		return plane_rotation

	def translate_to_plane(self, joint): 

		# Rotate the neutral and sweeping poses to align with the plane
		plane_rotation = self.rotate_to_plane(joint)
		sweeping_poses_rotated = []
		total_z = 0
		
		for t in self.sweeping_poses[joint]:
			rotated_transform = np.dot(np.linalg.inv(plane_rotation), t)
			sweeping_poses_rotated.append(rotated_transform)
			
			# After rotating the sweeping pose to align with the plane, get the z-axis value of the transform 
			total_z += rotated_transform[2, 3]
		
		# Define the translation into the plane as the mean z-axis value of all the sweeping_poses
		mean_z = total_z / len(self.sweeping_poses[joint])
		plane_translation = td.affines.compose((0,0,-mean_z), np.eye(3), np.ones(3))

		return plane_translation
	
	def rotate_neutral_pose(self, joint):

		plane_rotation = self.rotate_to_plane(joint)
		neutral_poses_rotated = []
		for t in self.neutral_poses[joint]:
			rotated_transform = np.dot(np.linalg.inv(plane_rotation), t)
			neutral_poses_rotated.append(rotated_transform)
		
		return neutral_poses_rotated

	def project_transforms_to_plane(self, joint):
		self.find_finger_plane_normal(joint)
		plane_rotation = self.rotate_to_plane(joint)
		plane_translation = self.translate_to_plane(joint)
		neutral_poses_rotated = self.rotate_neutral_pose(joint)
		# Save all calibration information to a dictionary `calibration_information`
		joint_calibration_information = {}
		joint_calibration_information["plane_rotation"] = np.linalg.inv(plane_rotation)
		joint_calibration_information["plane_translation"] = np.linalg.inv(plane_translation)
		joint_calibration_information["neutral_transform"] = neutral_poses_rotated[0]

		return joint_calibration_information
	
	def store_joint_transforms(self):
		
		self.calibration_information["mcp"] = self.project_transforms_to_plane("mcp")
		self.calibration_information["dip"] = self.project_transforms_to_plane("dip")

		return True
	
	def save_calibration_information(self, outfile_name):
		'''
		Saves calibration information dictionary, `calibration_information`, into a file. 
		'''
		outfile = open(outfile_name, 'wb')
		pickle.dump(self.calibration_information, outfile)
		outfile.close()
		print("Saved calibration.")
											   
def main(args):
	calibrator = Calibrator()
	if not calibrator.load_file(args.file):
		exit()
	if not calibrator.store_joint_transforms():
		exit()
	calibrator.save_calibration_information(args.outfile)

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--file', type=str, default = 'calibration_poses')
	parser.add_argument('--outfile', type=str, default = 'calibration_information')
	main(parser.parse_args())

