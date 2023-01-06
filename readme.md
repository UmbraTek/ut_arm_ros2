## Introduction

This repository contains the 3D models of arm series and demo packages  for ROS2 development and simulations.Developing and testing environment: Ubuntu 22.04 + ROS Humble. 

## Getting Started

1. [Install ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

2. Make sure that `colcon`, its extensions and `vcs` are installed: 

   ```
   $ sudo apt install python3-colcon-common-extensions python3-vcstool
   ```

3. Create a new ROS2 workspace:

   ```
   $ mkdir -p ~/colcon_ws/src
   ```

4. Pull relevant packages, install dependencies, compile, and source the workspace by using:

   ```
   $ cd ~/colcon_ws/src
   $ git clone -b main https://github.com/UmbraTek/ut_arm_ros2.git 
   $ rosdep update
   $ cd ~/colcon_ws/
   $ rosdep install --ignore-src --from-paths src --rosdistro humble -y
   $ colcon build
   $ source install/setup.sh
   ```

## Arm Description

Description files for all series of umbratek robotic arms, including UTRA, OPTI robotic arms.

1. Launch UTRA 6-550 rviz :

   ```
   $ ros2 launch arm_description view_utra6.launch.py arm_type:=utra6_550
   ```

2. Launch UTRA 6-850 rviz :

   ```
   $ ros2 launch arm_description view_utra6.launch.py arm_type:=utra6_550
   ```

3. Launch UTRA 6-1000 rviz :

   ```
   $ ros2 launch arm_description view_utra6.launch.py arm_type:=utra6_850
   ```

4. Launch UTRA 6-1500 rviz :

   ```
   $ ros2 launch arm_description view_utra6.launch.py arm_type:=utra6_1500
   ```

5. Launch OPTI 6-800 rviz :

   ```
   $ ros2 launch arm_description view_opti6.launch.py arm_type:=opti6_800
   ```

6. Launch OPTI 7-800 rviz :

   ```
   $ ros2 launch arm_description view_opti7.launch.py arm_type:=opti7_800
   ```

## Arm Controller

This package is a ros2 wrapper for "ut_arm_sdk" with functions implemented as ros service or ros topic. Communication with the real robotic arm hardware in ros2 is based on the services and topics provided in this section. By default, all services and topics are under utarm/ (unless the 'arm_ns' parameter is specified), such that the full name of "joint_states" is actually "utarm/joint_states".

The API manual can be found in the following two documents:

[ARM SDK WIKI](https://umbratek.com/wiki/en/#!utra/utra_api_python.md)

[ARM SDK Programming Guide](https://github.com/UmbraTek/ut_sdk_python/blob/master/utapi/utra/readme.md)



1. Run controller

   ```
   $ ros2 launch arm_controller utarm_api_server.launch.py arm_ip:="192.168.1.23"
   ```

2. Test the topic of the controller, showing the current status in real time

   ```
   $ ros2 launch arm_controller arm_controller_test.launch.py
   ```

3. Test the controller's services

   > Gets the current state

   ```
   $ ros2 service call /utarm/apisrv/get_motion_status  ut_msg/srv/GetInt16
   $ ros2 service call /utarm/apisrv/get_motion_mode  ut_msg/srv/GetInt16
   $ ros2 service call /utarm/apisrv/get_motion_enable  ut_msg/srv/GetInt16
   $ ros2 service call /utarm/apisrv/get_error_code  ut_msg/srv/GetUint16s
   $ ros2 service call /utarm/apisrv/get_servo_msg  ut_msg/srv/GetUint16s
   $ ros2 service call /utarm/apisrv/get_joint_actual_pos  ut_msg/srv/GetFloat32s
   ```

   > Set motion mode

   ```
   $ ros2 service call /utarm/apisrv/set_motion_mode  ut_msg/srv/SetInt16 "{data: 0}"
   ```

   > set motion status

   ```
   $ ros2 service call /utarm/apisrv/set_motion_status  ut_msg/srv/SetInt16 "{data: 0}"
   ```

   > P2P motion 

   ```
   $ ros2 service call /utarm/apisrv/moveto_joint_p2p  ut_msg/srv/MovetoJointP2p "{joints: [0, 0, 0, 0, 0, 0], speed: 50, acc: 100}"  
   ```

   > linear motion

   ```
   $ ros2 service call /utarm/apisrv/moveto_cartesian_line  ut_msg/srv/MovetoCartesianLine "{pose: [-, -, -, -, -, -], speed: 50, acc: 100}"
   ```

   > linear-blending motion

   ```
   ros2 service call /utarm/apisrv/moveto_cartesian_lineb  ut_msg/srv/MovetoCartesianLineb "{pose: [-, -, -, -, -, -], speed: 50, acc: 100, radii: 0}"
   ```

   > Servo motion

   ```
   ros2 service call /utarm/apisrv/moveto_servo_joint  ut_msg/srv/MovetoServoJoint "{axiz: 6, num: 1, frames: [-, -, -, -, -, -], time: 50, plan_delay: 1}"
   ```

   > Delay motion

   ```
   ros2 service call /utarm/apisrv/plan_sleep  ut_msg/srv/SetFloat32 "{data: 1}"
   ```

