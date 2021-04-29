# Display perceived environment as it is being observed, using OpenCV


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
	observations = robot.loadObservations("1")
	obs_tf = []
	slam = slam_alg.SLAMAlg(room, robot)

	for i, p in enumerate(robot.explore()):
		robot.moveToLoc(p[0], p[1], p[2])
		robot.refresh()

		if i == 0:
			tfMat = np.eye(3)
		else:
			robot.deadReckon()
			tfMat = robot.tfMat

		slam.stitchMap(tfMat, observations[i+1])

		room.display()
		cv.imshow("slam", slam.map_)
		cv.waitKey(0)



if __name__ == "__main__":
	main()