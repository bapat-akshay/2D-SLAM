import numpy as np
import slam_env
import cv2 as cv
import math
from random import gauss
import pickle


def dist(p1x, p1y, p2x, p2y):
	return np.sqrt((p1x-p2x)**2 + (p1y-p2y)**2)


class Agent:
	def __init__(self, room, spawnR=270, spawnC=680, spawnH=math.pi/2):
		self.room = room

		# Spawn pose
		self.spawnR = spawnR
		self.spawnC = spawnC
		self.spawnH = spawnH

		# Current pose
		self.row = spawnR
		self.col = spawnC
		self.heading = spawnH					# Radians

		# Previous pose
		self.prevrow = self.row
		self.prevcol = self.col
		self.prevheading = self.heading

		# Agent appearance parameters
		self.theta = 20*math.pi/180
		self.edgeLen = 15
		self.edgeThickness = 2

		# Sensor parameters
		self.fov = 2*math.pi/3
		self.stddev = 0.01
		self.covarMat = self.stddev**2 * np.eye(3)
		self.range = 500
		self.angularRes = 0.01
		self.linearRes = 0.1

		self.PC = []
		self.observations = [[]]
		self.PCmap = np.zeros((self.room.height, self.room.length), dtype=np.uint8)

		self.tfMat = np.eye(3)
		self.rotMat = np.eye(2)
		self.transVec = np.array([0, 0])


	def reset(self):
		print("Resetting robot")
		self.__init__(self.room, self.spawnR, self.spawnC, self.spawnH)


	def refresh(self):
		# Erase previous position
		cv.line(self.room.map, (self.prevcol, self.prevrow), (self.prevcol + int(self.edgeLen*math.cos(self.prevheading-self.theta)), \
			self.prevrow - int(self.edgeLen*math.sin(self.prevheading - self.theta))), (0,0,0), self.edgeThickness)
		cv.line(self.room.map, (self.prevcol, self.prevrow), (self.prevcol + int(self.edgeLen*math.cos(self.prevheading+self.theta)), \
			self.prevrow - int(self.edgeLen*math.sin(self.prevheading + self.theta))), (0,0,0), self.edgeThickness)

		# Draw current position
		cv.line(self.room.map, (self.col, self.row), (self.col + int(self.edgeLen*math.cos(self.heading-self.theta)), \
			self.row - int(self.edgeLen*math.sin(self.heading - self.theta))), (255,255,0), self.edgeThickness)
		cv.line(self.room.map, (self.col, self.row), (self.col + int(self.edgeLen*math.cos(self.heading+self.theta)), \
			self.row - int(self.edgeLen*math.sin(self.heading + self.theta))), (255,255,0), self.edgeThickness)


	def move(self, dR, dC, dH):
		self.prevrow = self.row
		self.prevcol = self.col
		self.prevheading = self.heading

		self.row = self.prevrow + dR
		self.col = self.prevcol + dC
		self.heading = self.prevheading + dH


	def moveToLoc(self, R, C, H):
		self.prevrow = self.row
		self.prevcol = self.col
		self.prevheading = self.heading

		self.row = R
		self.col = C
		self.heading = H


	def explore(self):
		radius = 200
		center = [self.room.height//2, self.room.length//2]
		points = []
		angle = 0
		while angle < 2*math.pi:
			points.append([center[0] - int(radius*math.sin(angle)), center[1] + int(radius*math.cos(angle)), angle + math.pi/2])
			angle += self.angularRes*5

		return points


	def explore2(self):
		start = [100, 800, math.pi]
		points = []
		end = 400
		while start[1] >= end:
			points.append(start.copy())
			start[1] -= 10

		return points


	# Grab point cloud and save to observations[]
	def grabPC(self):
		theta = self.heading - self.fov/2
		currPC = []
		while theta <= self.heading + self.fov/2:
			r = self.row
			c = self.col
			x = 0

			# Traverse scanner "ray" till either a wall is encountered or out of range
			while (not np.array_equal(self.room.map[round(r)][round(c)], (255,255,255))) and (dist(r, c, self.row, self.col) < self.range):
				r = self.row - (x*math.sin(theta))
				c = self.col + (x*math.cos(theta))

				# Coordinates from the pov of the agent
				robs = -(x*math.cos(theta - self.heading))
				cobs = -(x*math.sin(theta - self.heading))
				x += self.linearRes

				if r < 0 or c < 0 or r >= self.room.height or c >= self.room.length:
					break

			if r < 0 or c < 0 or r >= self.room.height or c >= self.room.length:
				break

			# Add Gaussian noise to sensor perception
			if np.array_equal(self.room.map[round(r)][round(c)], (255,255,255)):
				r = (gauss(r, self.stddev))
				c = (gauss(c, self.stddev))

				if r >= 0 and c >= 0 and r < self.room.height and c < self.room.length:
					currPC.append([gauss(robs, self.stddev), gauss(cobs, self.stddev)])
				else:
					break

			theta += self.angularRes

		self.observations.append(currPC)


	# Returns (noisy) estimate of change in R, C, H and estimated geometric transformation matrix associated with the motion
	def deadReckon(self, trueDeadReckon=False):
		
		if trueDeadReckon:
			sd = 0
		else:
			sd = self.stddev

		currTF = np.zeros((3,3))
		heading = gauss(self.heading - self.prevheading, sd)

		# Translation vector in absolute coordinates
		t = np.array([gauss(self.row, sd), gauss(self.col, sd), 1])

		# Translation vector in coordinates wrt frame i-1
		t2 = np.linalg.inv(self.tfMat) @ t.T

		currTF[0][0] = math.cos(heading)
		currTF[0][1] = -math.sin(heading)
		currTF[1][1] = math.cos(heading)
		currTF[1][0] = math.sin(heading)
		currTF[0][2] = t2[0]
		currTF[1][2] = t2[1]
		currTF[2][2] = 1

		self.tfMat = self.tfMat @ currTF

		return currTF, np.array([self.row, self.col, self.heading]), np.array([t[0], t[1], heading + self.prevheading])


	def saveObservations(self, string=""):
		pickle.dump(self.observations, open("agentObs" + string + ".p", "wb"))


	def loadObservations(self, string=""):
		return pickle.load(open("agentObs" + string + ".p", "rb"))



def main():
	pass


if __name__ == "__main__":
	main()