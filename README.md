# Introduction
This is a demo to show how to utilize object detection of OpenVINO in ROS 2.

The ROS 2 node here will transform topic `/detected_objects` from OpenVINO into `/cmd_vel`.

While seeing person, the robot will turn right. While seeing chair, the robot will turn left.

# Usage
* You should install [ros2_openvino_toolkit](https://github.com/intel/ros2_openvino_toolkit) from Intel.
* After source ros2_openvino_toolkit, build code
```
git clone https://github.com/Adlink-ROS/vino_integration_demo
cd ~/vino_integration_demo
colcon build --symlink-install
```
* Run, and your robot will be controlled by the result of object detection.
```
source install/local_setup.bash
ros2 run vino_integration vino_integration
```
