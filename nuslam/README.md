# NUSLAM
A package implementing Extended Kalman Filter SLAM for the NUTurtle with a fake sensor publishing relative location of landmarks.

`ros2 launch nuslam slam.launch.xml` to launch a simulation with EKF SLAM.

## Demo Screenshot
![SLAM Demo](images/demo.png)

## Launch File Details
`ros2 launch nuslam slam.launch.xml --show-args` to show arguments for launch file that launches the EKF SLAM.

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

## Parameter Details
* `kalman` - parameters to control the Extended Kalman Filter
    * `process_noise` - parameters to control the Q_bar, process (movement) noise matrix
        * `theta` - Kalman filter process noise for theta coordinate.
        * `x` - Kalman filter process noise for x coordinate.
        * `y` - Kalman filter process noise for y coordinate.
    * `sensor_noise` - Kalman filter sensor noise.

## Collaboration
I worked with the following people on this package:
* Katie Hughes