# Multi-Robot Simulator
In this project, your task is to extend the 2D Robot simulator illustrated during the course (located at `source/rp_02b_cpp_inheritance_`) by:

-   Implementing **ROS** Support
-   Implementing a configuration system by file
-   Complete the CMakeLists provided by us

## ROS Support
We expect every robot to publish and subscribe to their own topics.

i.e. For a configuration in which we have two robots (robot\_0, robot\_1) and one lidar mounted on robot\_0, We expect the following topics:
```txt
/robot_0/odom       <- Odometry topic for robot_0
/robot_0/cmd_vel    <- Control topic for robot_0
/robot_0/base_scan  <- Lidar topic for lidar sensor attached to robot_0
/robot_1/odom
/robot_1/cmd_vel
```
__Topics must be responsive__ meaning that:
    
    1. Publishing on /robot_N/cmd_vel should alter the velocity of robot_N
    2. Reading /robot_N/odom should yield the current pose of robot_N in world
    3. Reading /robot_N/scan should return the Lidar Scan of robot_N

You can use __RViz__ to visualize the quality of (2) and (3).

### TIPS

The simulator functionalities are already provided by us. Our suggestion is to build on top of the provided classes for integrating support over ROS.

Remember that you can call an overriden method from the derived method:
```cpp
class Foo {
    public:
    virtual void func() {
        cerr << "Base func " << endl;
    }
};
class Bar : public Foo {
    public:
    void func() override {
        cerr << "Derived func calling...";
        Foo::func();
    }
};
```

Calling `Bar::func()` yields:
```txt
Derived func calling...Base func
```

You could extend this example to implement a new robot and/or lidar objects which, implements ROS routines on top of the simulation routines.

On the other hand, you are free to follow your own path. If you think refactoring the provided code to support ROS is a better idea, go for it :)

The ROS wiki provides a lot of examples and tips, be sure to check it out if you feel stuck

## Configuration System

Instead of building the simulation environment by changing your main executable, We wish to configure it via a configuration file, provided a command line argument.

i.e. Given the `cappero_1r.json` configuration file, specifiying the cappero\_diag map and the presence of a single robot with a lidar mounted on top
```json
{
    "map": "cappero_laser_odom_diag_2020-05-06-16-26-03.png",
    "items": [
        {
            "id": 0,
            "type": "robot",
            "frame_id": "frame_robot0",
            "namespace": "robot_0",
            "radius": 0.5,
            "max_rv" : 0.1,
            "max_tv" : 0.5,
            "pose": [
                20,
                8,
                0
            ],
            "parent": -1
        },
        {
            "id": 1,
            "type": "lidar",
            "frame_id": "frame_lidar0",
            "namespace": "robot_0",
            "fov": 6.18,
            "max_range": 10.0,
            "num_beams": 360,
            "pose": [
                0,
                0,
                0
            ],
            "parent": 0
        }
    ]
}
```
We launch our simulator node as follows:
```sh
rosrun mrsim mrsim_node cappero_1r.json
```

### TIPS

You are free to use any serializing/deserializing library that you can find, however it might be easier to use the `json` or a simple custom format.

Regarding __JSON__, feel free to use `jsoncpp` library which provides an easy interface to parse the format. You can find a lot of examples in their [repo](https://github.com/open-source-parsers/jsoncpp).

For custom formats, you might use something easy like:
```txt
w <path_to_map>
r <id> <frame_id> <namespace> <radius> <max_rv> <max_tv> <pose> <parent>
l <id> <frame_id> <namespace> <fov> <max_range> <num_beams> <pose> <parent>
```
in which the first character represent the entity and its line contains the parameter of itself.
For instance you could rewrite `cappero_1r.json` as:
```txt
w cappero_laser_odom_diag_2020-05-06-16-26-03.png
r 0 frame_robot0 robot_0 0.5 0.1 0.5 20 8 0 -1
l 1 frame_lidar0 robot_0 6.18 10 360 0 0 0 0
```

The choice is yours, stick with the idea you prefer, or implement your own one! 




