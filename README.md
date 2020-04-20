# yak_ros

Example ROS frontend node for [the Yak TSDF package](https://github.com/ros-industrial/yak).

## Demo

This package contains a simulated image generator node and a launch file to provide a self-contained demonstration of TSDF reconstruction from streaming depth images. Currently the demo is only supported for ROS1.

The demo depends on [the gl_depth_sim package](https://github.com/Jmeyer1292/gl_depth_sim) to provide simulated depth images.

The demo node depends on external packages that aren't required by the core `yak_ros` node, so compilationo of the demo node is skipped by default. To build the demo, pass the `BUILD_DEMO` flag to catkin as `True`:

```
catkin build --cmake-args -DBUILD_DEMO=True
```

After re-sourcing the workspace, start the demo from the provided launch file:

```
roslaunch yak_ros demo.launch
```


## Usage

The yak_ros node provides 3 services and is a subscriber to 2 topics, which can be used to interact with it.


#### Topics

1 `/yak_ros/input_depth_image`: Listener for published depth images.
2. `/yak_ros/input_point_cloud`: Listener for published point clouds.


#### Services

1. `/yak_ros/generate_mesh`: Used to generate the output mesh to the provided path.
2. `/yak_ros/reset_tsdf`: Resets the TSDF Volume (= delete all provided depth images / point clouds)
3. `/yak_ros/update_params`: Used to update the TSDF Volume Params.


#### Visualization

A bounding box around the area of the TSDF Volume is published under the `/tsdf_node/visualization_marker` marker and can be viewed in `rviz`.
