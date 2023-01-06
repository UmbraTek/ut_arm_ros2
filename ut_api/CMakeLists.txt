cmake_minimum_required(VERSION 3.5)
project(ut_api)

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
    -Wno-unused-parameter
    -Wno-unused-variable
  )
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  include/ut_api
  ${ut_api_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/common/network.cpp
  src/common/preempt_rt.cpp
  src/common/socket_serial.cpp
  src/common/socket_tcp.cpp
  src/base/servo_api_base.cpp

  src/base/arm_api_base.cpp
  src/base/arm_report_status.cpp
  src/base/arm_report_config.cpp
  src/utra/utra_api_tcp.cpp
  src/utra/utra_report_status.cpp
  src/utra/utra_report_config.cpp
  src/utra/utra_flxie_api.cpp
)

target_link_libraries(${PROJECT_NAME} ${ament_LIBRARIES} rt pthread)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME}/)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
