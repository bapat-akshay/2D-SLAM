import numpy as np
import cv2 as cv
import math
import pickle
import matplotlib.pyplot as plt


class SLAMAlg:
	def __init__(self, room, agent):
		self.room = room
		self.agent = agent
		self.map_ = np.zeros((2*self.agent.range, 2*self.agent.range), np.uint8)

	def expandMap(self, direction):
		numR = self.map_.shape[0]
		numC = self.map_.shape[1]
		if direction == "rowup":
			new = np.zeros((numR*2, numC), np.uint8)
			new[numR:, :] = self.map_
			self.agent.tfMat[0,2] += self.map_.shape[0]
			self.map_ = new

		elif direction == "rowdown":
			new = np.zeros((numR*2, numC), np.uint8)
			new[:numR, :] = self.map_
			self.map_ = new

		elif direction == "colright":
			new = np.zeros((numR, numC*2), np.uint8)
			new[:, numC:] = self.map_
			self.map_ = new

		elif direction == "colleft":
			new = np.zeros((numR, numC*2), np.uint8)
			new[:, :numC] = self.map_
			self.agent.tfMat[1,2] += self.map_.shape[1]
			self.map_ = new


	def checkMapLimits(self, r, c):
		limitExceeded = False
		direction = ""

		if r < 0:
			limitExceeded = True
			direction = "rowup"

		elif r >= self.map_.shape[0]:
			limitExceeded = True
			direction = "rowdown"

		if c < 0:
			limitExceeded = True
			direction = "colleft"

		elif c >= self.map_.shape[1]:
			limitExceeded = True
			direction = "colright"
		
		return (limitExceeded, direction)


	# Transform points from newObs and stitch together to form a map of the perceived environment
	def stitchMap(self, tfMat, newObs):

		for point in newObs:
			point_tf = tfMat @ np.array([point[0], point[1], 1]).T
			point_tf[0] += 500
			point_tf[1] += 500

			ret = self.checkMapLimits(point_tf[0], point_tf[1])
			if ret[0]:
				self.expandMap(ret[1])

			self.map_[round(point_tf[0]), round(point_tf[1])] = 255

