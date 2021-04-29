# Display perceived environment at the end of the program using matplotlib scatter plot


import agent
import slam_alg
import slam_env

import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv


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
	observations = robot.loadObservations("2")
	obs_tfX = []
	obs_tfY = []
	slam = slam_alg.SLAMAlg(room, robot)

	for i, p in enumerate(robot.explore()):
		robot.moveToLoc(p[0], p[1], p[2])
		robot.refresh()

		robot.deadReckon()
		tfMat = robot.tfMat

		for point in observations[i+1]:
			point_tf = np.dot(tfMat, np.array([point[0], point[1], 1]).T)
			obs_tfX.append(point_tf[0])
			obs_tfY.append(point_tf[1])

		room.display()
		cv.waitKey(0)

	plt.scatter(obs_tfX, obs_tfY)
	plt.show()




if __name__ == "__main__":
	main()