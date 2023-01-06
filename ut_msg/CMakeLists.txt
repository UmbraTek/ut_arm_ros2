cmake_minimum_required(VERSION 3.8)
project(ut_msg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/CheckConnect.srv"
  "srv/Connect.srv"
  "srv/Disconnect.srv"
  "srv/GetFloat32.srv"
  "srv/GetFloat32s.srv"
  "srv/GetGripperState.srv"
  "srv/GetInt16.srv"
  "srv/GetUint16s.srv"
  "srv/MovetoCartesianLine.srv"
  "srv/MovetoCartesianLineb.srv"
  "srv/MovetoJointP2p.srv"
  "srv/MovetoServoJoint.srv"
  "srv/SetEnable.srv"
  "srv/SetFloat32.srv"
  "srv/SetGripperState.srv"
  "srv/SetInt16.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
