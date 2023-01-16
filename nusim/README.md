# NUSim 
Simulation package for the nuturtle.
* `ros2 launch nusim nusim.launch.xml` to launch the simulation.

![](images/nusim1.png)

## Launch File Details
* `ros2 launch nusim nusim.launch.xml --show-args` to show arguments for launch file that launches the simulation.

```
Currently there are no arguments for this launch file.
```

## Parameter Details
* `basic_world.yaml` - basic world configuration parameters.
    * `x0` - initial x position of robot in meters.
    * `y0` - initial y position of robot in meters.
    * `theta0` - initial angle rotation of robot in radians.
    * `obstacles` - parameters for the creation of cylindrical obstacles in the world.
        * `x` - list of x starting positions of obstacles in meters. Arbitrary length, but must match length of `y`.
        * `y` - list of y starting positions of obstacles in meters. Arbitray length, but must match length of `x`.
        * `r` - radius of all cylinder obstacles in meters. Single value applies to all obstacles.

## Collaboration
I worked alone on this package.