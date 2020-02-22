# yak_ros

This is an example ROS frontend node for [the Yak TSDF package](https://github.com/ros-industrial/yak). This package depends on Yak, so follow the Yak installation instructions to make sure its prerequisites are satisfied before building `yak_ros`.

## Installation

### ROS Melodic

These instructions assume that you have a ROS1 Catkin workspace at `~/catkin_ws`. Modify as needed to match your environment.

Download dependendencies that need to be built from source:

```
cd ~/catkin_ws/src
git clone https://github.com/ros-industrial/yak_ros.git
wstool init .
wstool merge -t . yak_ros/dependencies.rosinstall
wstool update -t .
```

Use Catkin to build the workspace:

```
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

### ROS2 (Dashing and newer)

(WIP)

## Demo

This package contains a simulated image generator node and a launch file to provide a self-contained demonstration of TSDF reconstruction from streaming depth images. Currently the demo is only supported for ROS1.

The demo depends on [the gl_depth_sim package](https://github.com/Jmeyer1292/gl_depth_sim) to provide simulated depth images. Dependencies for the demo are listed separately in `demo_dependencies.rosinstall`.

```
cd ~/catkin_ws/src
wstool merge -t . yak_ros/demo_dependencies.rosinstall
wstool update -t .
```

The demo node depends on external packages that aren't required by the core `yak_ros` node, so compilation of the demo node is skipped by default. To build the demo, pass the `BUILD_DEMO` flag to catkin as `True`:

```
catkin build --cmake-args -DBUILD_DEMO=True
```

After re-sourcing the workspace, start the demo from the provided launch file:

```
roslaunch yak_ros demo.launch
```
