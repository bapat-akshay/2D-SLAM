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
		# for r in range(posR-radius-self.wallThickness, posR+radius+self.wallThickness+1):
		# 	for c in range(posC-radius-self.wallThickness, posC+radius+self.wallThickness+1):
		# 		if (dist(r, c, posR, posC) >= radius - self.wallThickness/2) and \
		# 		(dist(r, c, posR, posC) <= radius + self.wallThickness/2):
		# 			self.map[r][c] = 255


	def addBox(self, CornerR, CornerC, height, length):
		self.map = cv.rectangle(self.map, (CornerC, CornerR), (CornerC+length, CornerR+height), (255, 255, 255), self.wallThickness)
		# for r in range(CornerR-self.wallThickness//2, CornerR+height+self.wallThickness//2+1):
		# 	for c in range(CornerC-self.wallThickness//2, CornerC+length+self.wallThickness//2+1):
		# 		if (r >= CornerR-np.ceil(self.wallThickness)/2 and r <= CornerR+np.floor(self.wallThickness)/2) or \
		# 		(c >= CornerC-np.ceil(self.wallThickness)/2 and c <= CornerC+np.floor(self.wallThickness)/2) or \
		# 		(r >= CornerR+height-np.ceil(self.wallThickness)/2 and r <= CornerR+height+np.floor(self.wallThickness)/2) or \
		# 		(c >= CornerC+length-np.ceil(self.wallThickness)/2 and c <= CornerC+length+np.floor(self.wallThickness)/2):
		# 			self.map[r][c] = 255


	def display(self):
		cv.imshow("SLAM Environment", self.map)
		cv.waitKey(10)


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
