# Introduction

This is a demo repo to show how to utilize object detection of OpenVINO in ROS 2.

# Build

* You should install [ros2_openvino_toolkit](https://github.com/intel/ros2_openvino_toolkit) from Intel first.
* Source ROS 2 dashing and ros2_openvino_toolkit.
* After source ros2_openvino_toolkit, build code
```
mkdir -p ~/ros2_openvino_example_ws/src
cd ~/ros2_openvino_example_ws/src
git clone https://github.com/Adlink-ROS/vino_integration_demo
cd ~/ros2_openvino_example_ws
colcon build --symlink-install
```

# Usage
## detected_objects to cmd_vel

The ROS 2 node here will transform topic `/detected_objects` from OpenVINO into `/cmd_vel`.

While seeing person, the robot will turn right. While seeing chair, the robot will turn left.

```
# Run your robot first.
# Source ROS 2 dashing and ros2_openvino_toolkit.
cd ~/ros2_openvino_example_ws/src
source install/local_setup.bash
ros2 run vino_integration vino_integration
```

## detection_objects to string

The ROS 2 node will transform topic `/detected_objects` to String type.

This will be used while bridging ROS 1 since bridge can't transmit topic `/detected_objects`.

```
# Source ROS 2 dashing and ros2_openvino_toolkit
cd ~/ros2_openvino_example_ws/src
source install/local_setup.bash
ros2 run vino_integration vino_transform
```
