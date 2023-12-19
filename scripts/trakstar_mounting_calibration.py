#!/usr/bin/env python3

import transforms3d as td
import numpy as np
import pickle
import argparse
from sklearn.decomposition import PCA

class Calibrator(object):

	def __init__(self):
		pass

	def load_file(self, filename):

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
		plane_transform = td.affines.compose(np.zeros(3), R, np.ones(3))

		self.neutral_poses_rotated = []
		self.sweeping_poses_rotated = []
		self.data_poses_rotated = []
		total_z = 0
		
		for t in self.neutral_poses:
			self.neutral_poses_rotated.append(np.dot(np.linalg.inv(plane_transform), t))
		for t in self.data_poses:
			t_rot = np.dot(np.linalg.inv(plane_transform), t)
			self.data_poses_rotated.append(t_rot)
		for t in self.sweeping_poses:
			rotated_transform = np.dot(np.linalg.inv(plane_transform), t)
			self.sweeping_poses_rotated.append(rotated_transform)
			total_z += rotated_transform[2, 3]
		
		mean_z = total_z / len(self.sweeping_poses)

		projected_sweeping_poses = []
		for t in self.sweeping_poses_rotated:
			t[2, 3] -= mean_z
			projected_sweeping_poses.append(t)

		R_neutral = self.neutral_poses_rotated[0]

		for t in self.data_poses_rotated:
			print(td.euler.mat2euler(np.dot(np.linalg.inv(R_neutral), t), axes = 'szxy')[0]*(180/np.pi))
			ax1, angle_1, pt1 = td.axangles.aff2axangle(np.dot(np.linalg.inv(R_neutral), t))
			print(angle_1*(180/np.pi))
			print("--")
											   
def main(args):
	calibrator = Calibrator()
	if not calibrator.load_file(args.file):
		exit()
	if not calibrator.find_finger_plane_normal():
		exit()
	if not calibrator.project_transforms_to_plane():
		exit()

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--file', type=str, default = 'calibration_poses')
	main(parser.parse_args())

