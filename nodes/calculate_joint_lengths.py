#!/usr/bin/env python

import transforms3d as td
import numpy as np
import pickle
import argparse
import matplotlib.pyplot as plt
import circle_fit as cf
import matplotlib 

class JointLengths(object):

	def __init__(self):
		self.lengths = {}
		pass

	def load_file(self, filename):
		'''
		Loads in the sensor information, stored as lists: 
			x  
			y 
			theta     
			total_length
		'''
		try:
			file = open(filename, 'rb')
			self.sensor_information = pickle.load(file)
			self.calibration_information = pickle.load(file)
		except IOError:
			print("Failed to read file")
			return False
		return True

	def circlefit(self, joint):
		x = []#self.sensor_information["x"]
		y = []#self.sensor_information["y"]
		for t in self.sensor_information[joint]["transforms"]:
			pt = np.linalg.inv(t)
			x.append(pt[0, 3])
			y.append(pt[1, 3])

		a = np.linspace(0, 2*np.pi, 100)
		print(np.shape(np.array([x, y]).T))
		xc, yc, r, _ = cf.least_squares_circle(np.array([x, y]).T)
		cx = xc + r*np.cos(a)
		cy = yc + r*np.sin(a)
		print(xc, yc)
		plt.plot(cx, cy)
		print(r*1000)
		plt.plot(x, y, 'o', markersize=1, label="sensor")
		plt.axis('equal')
		plt.show()
		return True

	def calculate_joint_lengths(self, joint):
		x = [] #self.sensor_information["x"]
		y = [] #self.sensor_information["y"]
		for t in self.sensor_information[joint]["transforms"]:
			pt = np.linalg.inv(t)
			x.append(pt[0, 3])
			y.append(pt[1, 3])
		LHS = []
		RHS = []

		if len(x) != len(y):
			print("Unequal number of elements in lists x, y")
			exit()

		for i in range(len(x)):
			theta = -self.sensor_information[joint]["theta"][i]
			lhs = np.zeros((2, 4))
			lhs[0, 0] = 1
			lhs[0, 2] = np.cos(theta)
			lhs[0, 3] = -np.sin(theta)
			lhs[1, 1] = 1
			lhs[1, 2] = np.sin(theta)
			lhs[1, 3] = np.cos(theta)
		
			rhs = np.array([[x[i]], [y[i]]])

			LHS.append(lhs)
			RHS.append(rhs)

		t = np.linalg.lstsq(np.vstack(LHS), np.vstack(RHS), rcond = None)

		t1_x = t[0][0]
		t1_y = t[0][1]
		t2_x = t[0][2]
		t2_y = t[0][3]

		t1 = np.sqrt((t1_x)**2 + (t1_y)**2)*1000
		t2 = np.sqrt((t2_x)**2 + (t2_y)**2)*1000
	
		self.lengths[joint] = [t1, t2]
		return True

	def save_joint_length_information(self, outfile_name):
		'''
		Saves calibration information dictionary, `calibration_information`, into a file. 
		'''
		outfile = open(outfile_name, 'wb')
		pickle.dump(self.lengths, outfile)
		outfile.close()
		print("Saved joint lengths.")

def main(args):
	jointlengths = JointLengths()
	if not jointlengths.load_file(args.file):
		exit()
	if not jointlengths.calculate_joint_lengths("mcp"):
		exit()
	if not jointlengths.calculate_joint_lengths("dip"):
		exit()
	print(jointlengths.lengths)
	jointlengths.save_joint_length_information(args.outfile)

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--file', type=str, default = 'sensor_information')
	parser.add_argument('--outfile', type=str, default = 'joint_lengths')
	main(parser.parse_args())
