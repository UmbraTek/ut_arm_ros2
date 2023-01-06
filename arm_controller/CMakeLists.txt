cmake_minimum_required(VERSION 3.5)
project(arm_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic
    -Wno-sign-compare
    -Wno-unused-parameter
    -Wno-unused-variable
  )
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ut_msg REQUIRED)
find_package(ut_api REQUIRED)

include_directories(
  include
  ${ut_api_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
)

add_executable(utarm_report_publisher src/utarm_report_publisher.cpp)
ament_target_dependencies(utarm_report_publisher rclcpp ut_msg ut_api)
target_link_libraries(utarm_report_publisher ${ament_LIBRARIES} ${ut_api_LIBRARIES})

add_executable(utarm_api_server src/utarm_api_server.cpp)
ament_target_dependencies(utarm_api_server rclcpp ut_msg ut_api)
target_link_libraries(utarm_api_server ${ament_LIBRARIES} ${ut_api_LIBRARIES})

add_executable(arm_controller_test src/arm_controller_test.cpp)
ament_target_dependencies(arm_controller_test rclcpp ut_msg ut_api)
target_link_libraries(arm_controller_test ${ament_LIBRARIES} ${ut_api_LIBRARIES})

install(TARGETS
  utarm_report_publisher
  utarm_api_server
  arm_controller_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
