import numpy as np
import cv2 as cv
import math
import pickle
import matplotlib.pyplot as plt


def rotComponent(mat):
	mat[0][2] = 0
	mat[1][2] = 0

	return mat


def arrayMin(arr1, arr2):
	if np.linalg.norm(arr1) < np.linalg.norm(arr2):
		return arr1
	else:
		return arr2


class SLAMAlg:
	def __init__(self, room, agent):
		self.room = room
		self.agent = agent
		self.map_ = np.zeros((4*self.agent.range, 4*self.agent.range), np.uint8)

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


	def unravelTFMat(self, tfMat):
		theta = math.acos(tfMat[0][0])
		assert theta == math.acos(tfMat[1][1])

		return (tfMat[0][2], tfMat[1][2], theta)


	def optimizeGraph(self, constraints, expectedPoses, observedPoses):
		# constraints[i] = [a, b, Tba, T0a, Sigma]

		itr = 1
		pwr = 2
		while(itr <= 20):

			# Update M
			if (itr == pwr or itr == 1):
				pwr *= pwr
				M = np.zeros((len(constraints), 3))
				gamma = np.array([math.inf, math.inf, math.inf])

				for con in constraints:
					R = rotComponent(con[3])
					W = np.linalg.inv(R @ con[4] @ R.T)

					for i in range(con[0], con[1]):
						#print(M)
						M[i,:] += np.diagonal(W)
						gamma = arrayMin(gamma, np.diagonal(W))

			# Optimize graph using modified stochastic gradient descent
			for i, con in enumerate(constraints):
				if i == 0:
					continue

				R = rotComponent(con[3])
				res = observedPoses[i] - expectedPoses[i]
				res[2] %= 2*math.pi

				d = 2 * np.linalg.inv(R.T @ con[4] @ R) @ res

				for j in range(3):
					alpha = 1/(gamma[j]*itr)
					weight = sum([1/M[i][j] for i in range(con[0], con[1])])
					beta = (con[1] - con[0]) * d[j] * alpha

					if abs(beta) > abs(res[j]):
						beta = res[j]

					dpose = 0

					for i in range(con[0]+1, len(constraints)):
						if i >= con[0]+1 and i <= con[1]:
							dpose += beta/M[i][j]/weight
						expectedPoses[i][j] += dpose

			itr += 1