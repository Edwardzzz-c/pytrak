#!/usr/bin/env python3

import transforms3d as td
import numpy as np
import pickle
import argparse
from sklearn.decomposition import PCA

class CalibrationInformation(object):

	def __init__(self):
		self.plane_projection_transform = None
		self.neutral_transform = None
