import agent
import slam_alg
import slam_env

import numpy as np
import cv2 as cv
import pickle
from tqdm import tqdm


def main():
	length = 960
	height = 540
	thickness = 5
	room = slam_env.createMap(length, height, thickness)
	room.addBox(200, 400, 140, 160)
	room.addCircle(100, 100, 50)
	room.addCircle(100, 200, 40)
	room.addCircle(200, 100, 30)
	room.addCircle(500, 50, 40)
	room.addBox(400, 900, 140, 60)

	robot = agent.Agent(room)

	for i, p in enumerate(tqdm(robot.explore())):
		robot.moveToLoc(p[0], p[1], p[2])
		robot.refresh()
		robot.grabPC()
		#room.display()

		# obs = np.zeros((600, 600), dtype=np.uint8)
		# for point in robot.observations[i]:
		# 	obs[point[0], point[1]] = 255
		# cv.imshow("Agent observation", obs)
		# cv.waitKey(1000)

		# robot.tfMat = np.dot(robot.tfMat, robot.deadReckon()[3])
		# Restimate = robot.deadReckon()[0]
		# Cestimate = robot.deadReckon()[1]

	robot.saveObservations("2")


if __name__ == "__main__":
	main()