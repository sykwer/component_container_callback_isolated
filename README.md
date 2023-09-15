# component_container_callback_isolated
A component container that assigns a dedicated thread for each callback group.

## Build and Install
```
$ mkdir -p /path/to/project/directory
$ cd /path/to/project/directory
$ git clone https://github.com/sykwer/component_container_callback_isolated.git src/
$ source /opt/ros/humble/setup.bash
$ colcon build
$ source install/setup.bash
```

## How to Use
Refer to the [launch directory](https://github.com/sykwer/component_container_callback_isolated/tree/main/launch).

## Quick Demo
Demonstration of launching the component container and loading the sample node at the same time.
```
$ ros2 launch rclcpp_component_container_callback_isolated sample_node.launch.xml
```
Demonstration of loading the sample node into the pre-launched component container afterwards.
```
$ ros2 run rclcpp_component_container_callback_isolated component_container_callback_isolated --ros-args --remap __node:=sample_container
$ ros2 launch rclcpp_component_container_callback_isolated load_sample_node.launch.xml
```
