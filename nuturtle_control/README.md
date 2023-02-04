# NUTurtle Control
A package with nodes for interfacing with and controlling the NUTurtle.
* `ros2 launch nuturtle_control start_robot.launch.xml` to launch the control/interface nodes with several options.

## Launch File Details
* `ros2 launch nuturtle_control start_robot.launch.xml --show-args` to show arguments for launch file that launches the simulation.

```
'cmd_src':
    Source for cmd_vel messages. Valid choices are: ['teleop', 'circle', 'none']
    (default: 'teleop')

'robot':
    Simulation or other options for running robot nodes. Valid choices are: ['nusim', 'localhost', 'none']
    (default: 'nusim')

'use_rviz':
    Start RVIZ to visualize robot. Valid choices are: ['true', 'false']
    (default: 'true')
```

## Nodes
See source code for comments elaborating on the parameters, publishers, subscribers, servers, and clients of each node.
* [`circle`](src/circle.cpp) - Publishes cmd_vel commands to move a differential drive robot in a circle.
* [`odometry`](src/odometry.cpp) - Calculates and publishes odometry from input joint states.
* [`turtle_control`](src/turtle_control.cpp) - Converts turtlebot sensor data into joint states and commanded twists into wheel commands.

## Collaboration
I worked with the following people on this package:
* Liz Metzger
* Katie Hughes
* Kevin Nella
* Ava Zahedi