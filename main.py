# Display perceived environment as it is being observed, using OpenCV


import agent
import slam_alg
import slam_env
import math
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import icp


CHECK_EVERY = 5
SCENE_DIST_THRESHOLD = 50
SCENE_ANGLE_THRESHOLD = 30*(math.pi/180)


def unravelTFMat(tfMat):
		theta = math.acos(tfMat[0][0])
		assert np.round(theta, 5) == np.round(math.acos(tfMat[1][1]), 5)

		return (tfMat[0][2], tfMat[1][2], theta)


def checkSceneVisited(memory, obs):
	for scene in memory:

		# Ensure point clouds have same size
		if len(scene) < len(obs):
			obsTrim = obs[:len(scene)]
			sceneTrim = scene
		elif len(scene) > len(obs):
			obsTrim = obs
			sceneTrim = scene[:len(obs)]
		else:
			obsTrim = obs
			sceneTrim = scene

		mat, distance, iters = icp.icp(np.array(sceneTrim), np.array(obsTrim))
		res = unravelTFMat(mat)
		
		return (res[2] < SCENE_ANGLE_THRESHOLD) and (math.sqrt(res[0]**2 + res[1]**2) < SCENE_DIST_THRESHOLD)



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

	constraints = []
	expectedPoses = {}
	observedPoses = {}
	robot = agent.Agent(room)
	observations = robot.loadObservations("2")
	memory = []
	obs_tf = []
	slam = slam_alg.SLAMAlg(room, robot)

	for i, p in enumerate(robot.explore()):
		robot.moveToLoc(p[0], p[1], p[2])
		robot.refresh()

		if i == 0:
			tfMat = np.eye(3)
		else:
			currTF, expPose, obsPose = robot.deadReckon()
			expectedPoses[i] = expPose
			observedPoses[i] = obsPose
			tfMat = robot.tfMat
			# currTF transforms from the ith frame to the (i-1)th frame
			constraints.append([i-1, i, currTF, tfMat, robot.covarMat])
			expectedPoses[i] = expPose
			observedPoses[i] = obsPose

		# Logic to check if scene has been visited earlier
		# Based on ICP evaluation of scenes, followed by comparison of resultant tf matrix
		# Food for thought: what if after loop closure, the vehicle continues to move on the same path?
		# Can you prevent subsequent redundant loop closure calls?

		if i > 10 and checkSceneVisited(memory, observations[i+1]):
			# Add constraint corresponding to loop closure
			slam.optimizeGraph(constraints, expectedPoses, observedPoses)

		if i % CHECK_EVERY == 0:
			memory.append(observations[i+1])

	robot.reset()

	# for i, pose in enumerate(robot.explore()):
	for i, pose in observedPoses.items():
		robot.moveToLoc(int(pose[0]), int(pose[1]), pose[2])
		robot.refresh()

		if i == 0:
			tfMat = np.eye(3)
		else:
			currTF, expPose, obsPose = robot.deadReckon(trueDeadReckon=True)
			tfMat = robot.tfMat

		slam.stitchMap(tfMat, observations[i+1])

		room.display()
		cv.imshow("slam", slam.map_)
		cv.waitKey(0)



if __name__ == "__main__":
	main()