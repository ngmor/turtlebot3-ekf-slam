# ME495 Sensing, Navigation and Machine Learning For Robotics
* Nicolas Morales
* Winter 2023

## Demo Video
[EKF SLAM with Landmark Detection and Unknown Data Association on Real Robot](https://user-images.githubusercontent.com/113186159/225492968-baeebdf7-f481-4faf-a2ae-2aefef27c33b.mp4)

# Package List
This repository consists of several ROS packages
- [`nusim`](nusim) - contains simulator for the NUTurtle world.
- [`nuslam`](nuslam) - contains a node for performing Extended Kalman Filter SLAM with the NUTurtle. **Demo videos for this EKF SLAM (Task L) are embedded in this package's README.**
- [`nuturtle_control`](nuturtle_control) - contains nodes for interfacing with and controlling the NUTurtle. **A demo video for this package (Task F) is embedded in this package's README.**
- [`nuturtle_description`](nuturtle_description) - contains URDF and configuration parameters for the NUTurtle.


Other include packages:
- [`turtlelib`](turtlelib) - a C++ library with classes for 2D kinematics and odometry for differential drive robots.
- [`turtlelib_ros`](turtlelib_ros) - a C++ library to handle interactions between the turtlelib library and ROS.

# Dependencies
Dependencies for this package are listed in the [`turtle.repos`](turtle.repos) file. To import all dependencies, clone this repository as `nuturtle` into the `src` directory in your workspace root. Then from the workspace root directory, run the command:
```
vcs import src < src/nuturtle/turtle.repos
```
