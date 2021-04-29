import numpy as np
import cv2 as cv


def dist(p1x, p1y, p2x, p2y):
	return np.sqrt((p1x-p2x)**2 + (p1y-p2y)**2)


class Map:
	def __init__(self, length, height, thickness):
		self.length = length
		self.height = height
		self.wallThickness = thickness
		self.map = np.zeros((self.height, self.length, 3), dtype=np.uint8)

		# Walls
		for r in range(self.height):
			for c in range(self.length):
				if (r >= 0 and r < self.wallThickness) or (c >= 0 and c <self.wallThickness) or \
				(r >= self.height-self.wallThickness and r < self.height) or \
				(c >= self.length-self.wallThickness and c < self.length):
					self.map[r][c][:] = (255, 255, 255)

	def addCircle(self, posR, posC, radius):
		self.map = cv.circle(self.map, (posC, posR), radius, (255, 255, 255), self.wallThickness)


	def addBox(self, CornerR, CornerC, height, length):
		self.map = cv.rectangle(self.map, (CornerC, CornerR), (CornerC+length, CornerR+height), (255, 255, 255), self.wallThickness)


	def display(self):
		cv.imshow("SLAM Environment", self.map)


def createMap(length, height, thickness):
	return Map(length, height, thickness)


def main():
	length = 960
	height = 9*length//16
	thickness = 5

	room = Map(length, height, thickness)
	room.addBox(200, 300, 100, 50)
	room.addCircle(100, 100, 50)
	room.display()


if __name__ == "__main__":
	main()
