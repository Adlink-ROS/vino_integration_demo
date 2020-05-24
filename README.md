# Introduction

This is a demo repo to show how to utilize object detection of OpenVINO in ROS 1.

# Build

* You should install [ros_openvino_toolkit](https://github.com/intel/ros_openvino_toolkit) from Intel first.
* Source ROS 1 melodic and ros_openvino_toolkit.
* After source ros_openvino_toolkit, build code
```
mkdir -p ~/ros_openvino_example_ws/src
cd ~/ros_openvino_example_ws/src
git clone https://github.com/Adlink-ROS/vino_integration_demo -b melodic-devel
cd ~/ros_openvino_example_ws
catkin_make
```

# Usage
## detected_objects to cmd_vel

The ROS node here will transform topic `/detected_objects` from OpenVINO into `/cmd_vel`.

While seeing person, the robot will turn right. While seeing chair, the robot will turn left.

```
# Run your robot first.
# Source ROS 1 melodic and ros_openvino_toolkit.
cd ~/ros1_openvino_example_ws
source devel/setup.bash
rosrun vino_integration vino_integration
```
