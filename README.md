
# 2D-SLAM: Algorithm for a Simulated Two-Dimensional World

## Simulation
The simulated environment consists of a room with certain features (circular and rectangular objects). A robot explores this room and gathers observations in the form of simulated point clouds. The robot's perception and odometry are erroneous, and this algorithm uses pose graph optimization to minimize the effects of the error in odometry.

![Simulation Environment](https://user-images.githubusercontent.com/47930459/119280152-067ee980-bbfe-11eb-88f2-f6e995ddd6ab.jpg)
This images illustrates the room which the robot explores. Everything in white represents features of the room, while the robot is the V-shaped light blue colored object.

## Pose Graph Optimization
![SLAM Without Pose Graph Optimization](https://user-images.githubusercontent.com/47930459/119280336-f6b3d500-bbfe-11eb-9cbb-31e903d8b8cb.jpg)
Without pose graph optimization, faulty odometry results in incorrect SLAM.






![SLAM With Pose Graph Optimization](https://user-images.githubusercontent.com/47930459/119280357-092e0e80-bbff-11eb-946f-888204d67bf6.jpg)
Using pose graph optimization, the algorithm corrects all previous observations as soon as it detects that it has reached a location it has already visited before, resulting in a much more accurate SLAM solution.

## Scholarly References:
- "A Tutorial on Graph-Based SLAM", Giorgio Grisetti, Rainer Kummerle, Cyrill Stachniss, and Wolfram Burgard, _Department of Computer Science, University of Freiburg_
- "Fast Iterative Alignment of Pose Graphs with Poor Initial Estimates", Edwin Olson, John Leonard, and Seth Teller, _International Conference on Robotics and Automation (ICRA), pp. 2262-2269, May 2006_

## Observation (.p) Files Reference

- "1" for zero noise observation, circular motion
- "2" for standard deviation of 2, circular motion
- "4" for zero noise observation, linear motion
