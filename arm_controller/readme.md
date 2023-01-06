# Communication with arm through the rosservice command

ros2 launch arm_description view_utra6.launch.py arm_type:=utra6_550

ros2 launch arm_controller utarm_api_server.launch.py arm_ip:="192.168.1.23"
ros2 launch arm_controller arm_controller_test.launch.py


ros2 service call /utarm/apisrv/get_motion_status  ut_msg/srv/GetInt16
ros2 service call /utarm/apisrv/get_motion_mode  ut_msg/srv/GetInt16
ros2 service call /utarm/apisrv/get_motion_enable  ut_msg/srv/GetInt16
ros2 service call /utarm/apisrv/get_error_code  ut_msg/srv/GetUint16s
ros2 service call /utarm/apisrv/get_servo_msg  ut_msg/srv/GetUint16s
ros2 service call /utarm/apisrv/get_joint_actual_pos  ut_msg/srv/GetFloat32s


ros2 service call /utarm/apisrv/set_motion_mode  ut_msg/srv/SetInt16 "{data: 0}"
ros2 service call /utarm/apisrv/set_motion_status  ut_msg/srv/SetInt16 "{data: 0}"
ros2 service call /utarm/apisrv/moveto_joint_p2p  ut_msg/srv/MovetoJointP2p "{joints: [0, 0, 0, 0, 0, 0], speed: 50, acc: 100}"   

ros2 service call /utarm/apisrv/moveto_cartesian_line  ut_msg/srv/MovetoCartesianLine "{pose: [-, -, -, -, -, -], speed: 50, acc: 100}"
ros2 service call /utarm/apisrv/moveto_cartesian_lineb  ut_msg/srv/MovetoCartesianLineb "{pose: [-, -, -, -, -, -], speed: 50, acc: 100, radii: 0}"
ros2 service call /utarm/apisrv/moveto_servo_joint  ut_msg/srv/MovetoServoJoint "{axiz: 6, num: 1, frames: [-, -, -, -, -, -], time: 50, plan_delay: 1}"
ros2 service call /utarm/apisrv/plan_sleep  ut_msg/srv/SetFloat32 "{data: 1}"

