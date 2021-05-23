![Room](https://user-images.githubusercontent.com/47930459/119280152-067ee980-bbfe-11eb-88f2-f6e995ddd6ab.jpg)
# 2D-SLAM: Algorithm for a Simulated Two-Dimensional World

## Simulation
The simulated environment consists of a room with certain features (circular and rectangular objects). A robot explores this room and gathers observations in the form of simulated point clouds. The robot's perception and odometry are erroneous, and this algorithm uses pose graph optimization to minimize the effects of the error in odometry.

![alt text](https://github.com/bapat-akshay/2D-SLAM/edit/main/Room.PNG)




.p files:

- "1" for zero noise observation, circular motion
- "2" for standard deviation of 2, circular motion
- "4" for zero noise observation, linear motion
