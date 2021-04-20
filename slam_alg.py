import numpy as np
import cv2 as cv
import math
import pickle


class SLAMAlg:
	def __init__(self, room, agent):
		self.room = room
		self.agent = agent
		self.map_ = np.zeros((self.agent.range, self.agent.range), np.uint8)

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
			self.tfMat[1,2] += self.map_.shape[1]
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


	def stitchMap(self, newObs):
		for point in newObs:
			point_tf = np.dot(self.agent.tfMat, np.array([point[0], point[1], 1]).T)
			point.tf /= point.tf[2]

			ret = self.checkMapLimits(point_tf[0], point_tf[1])
			if ret[0]:
				self.expandMap(ret[1])

			self.map_[point_tf[0], point_tf[1]] = 255

