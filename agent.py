import numpy as np
import slam_env
import cv2 as cv
import math
from random import gauss
import pickle


def dist(p1x, p1y, p2x, p2y):
	return np.sqrt((p1x-p2x)**2 + (p1y-p2y)**2)


class Agent:
	def __init__(self, room):
		self.room = room

		# Current pose
		self.row = 2*self.room.height//4
		self.col = 2*self.room.length//4
		self.heading = math.pi						# Radians

		# Previous pose
		self.prevrow = self.row
		self.prevcol = self.col
		self.prevheading = self.heading

		# Spawn coordinates
		self.spawnR = 2*self.room.height//4
		self.spawnC = 2*self.room.length//4
		self.spawnheading = math.pi

		# Agent appearance parameters
		self.theta = 20*math.pi/180
		self.edgeLen = 15
		self.edgeThickness = 2

		# Sensor parameters
		self.fov = 2*math.pi/3
		self.stddev = 1
		self.range = 300
		self.angularRes = 0.01
		self.linearRes = 0.1

		self.PC = []
		self.observations = [[]]
		self.PCmap = np.zeros((self.room.height, self.room.length), dtype=np.uint8)

		self.tfMat = np.eye(3)


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


	# Grab point cloud and save to observations[]
	def grabPC(self):
		theta = self.heading - self.fov/2
		currPC = []
		while theta <= self.heading + self.fov/2:
			r = self.row
			c = self.col
			x = 0

			# Traverse scanner "ray" till either a wall is encountered or out of range
			while (not np.array_equal(self.room.map[r][c], (255,255,255))) and (dist(r, c, self.row, self.col) < self.range):
				r = self.row - int(x*math.sin(theta))
				c = self.col + int(x*math.cos(theta))

				# Coordinates from the pov of the agent
				cobs = (self.range - int(x*math.sin(theta - self.heading)))
				robs = 300 - int(x*math.cos(theta - self.heading))
				x += self.linearRes

				if r < 0 or c < 0 or r >= self.room.height or c >= self.room.length:
					break

			if r < 0 or c < 0 or r >= self.room.height or c >= self.room.length:
				break

			# Add Gaussian noise to sensor perception
			if np.array_equal(self.room.map[r][c], (255,255,255)):
				r = int(gauss(r, self.stddev))
				c = int(gauss(c, self.stddev))
				if r >= 0 and c >= 0 and r < self.room.height and c < self.room.length:
					# self.PC.append([r, c])
					# self.PCmap[r][c] = 255
					# self.room.map[r][c][:] = (0,0,255)
					currPC.append([robs, cobs])
				else:
					break

			theta += self.angularRes

		self.observations.append(currPC)


	# Returns (noisy) estimate of change in R, C, H and estimated geometric transformation matrix associated with the motion
	def deadReckon(self):
		mat = np.zeros((3,3))
		mat[0][0] = gauss(math.cos(self.heading - self.prevheading), self.stddev)
		mat[0][1] = gauss(math.sin(self.heading - self.prevheading), self.stddev)
		mat[1][1] = gauss(math.cos(self.heading - self.prevheading), self.stddev)
		mat[1][0] = gauss(-math.sin(self.heading - self.prevheading), self.stddev)

		mat[2][2] = 1
		mat[1][2] = gauss(self.col - self.prevcol, self.stddev)
		mat[0][2] = gauss(self.row - self.prevrow, self.stddev)

		heading = gauss(self.heading - self.prevheading, self.stddev)
		self.tfMat = mat

		return mat[0][2], mat[1][2], heading, mat


	def saveObservations(self):
		pickle.dump(self.observations, open("agentObs.p", "wb"))


	def loadObservations(self):
		return pickle.load(open("agentObs.p", "rb"))




def main():
	# temp = np.zeros((500,500,3), dtype=np.uint8)
	# temp = cv.circle(temp, (250,250), 2, (0,0,255), -1)
	# count = 0
	# while count < 20:
	# 	x = int(gauss(250, 2))
	# 	y = int(gauss(250, 2))
	# 	temp = cv.circle(temp, (x,y), 1, (255,255,255), -1)
	# 	count += 1
	# cv.imshow("test", temp)
	# cv.waitKey(0)



	length = 960
	height = 9*length//16
	thickness = 5
	room = slam_env.createMap(length, height, thickness)
	room.addBox(200, 400, 140, 160)
	room.addCircle(100, 100, 50)
	room.addCircle(100, 200, 40)
	room.addCircle(200, 100, 30)
	room.addCircle(500, 50, 40)
	room.addBox(400, 900, 140, 60)
	sensor = Agent(room)
	sensor.refresh()
	#room.display()

	for i, p in enumerate(sensor.explore()):
		sensor.moveToLoc(p[0], p[1], p[2])
		sensor.refresh()
		sensor.grabPC()
		room.display()

		obs = np.zeros((600, 600), dtype=np.uint8)
		for point in sensor.observations[i]:
			print(point)
			obs[point[0], point[1]] = 255
		cv.imshow("Agent observation", obs)
		cv.waitKey(1000)

		sensor.tfMat = np.dot(sensor.tfMat, sensor.deadReckon()[3])
		Restimate = sensor.deadReckon()[0]
		Cestimate = sensor.deadReckon()[1]



if __name__ == "__main__":
	main()