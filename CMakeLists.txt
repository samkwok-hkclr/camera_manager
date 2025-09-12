cmake_minimum_required(VERSION 3.8)
project(camera_manager)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-dev)
endif()

cmake_policy(SET CMP0074 NEW)
set(PCL_ROOT /usr)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(robot_controller_msgs REQUIRED)
find_package(robotic_platform_msgs REQUIRED)

include_directories(
  include
)

add_executable(camera_manager
  src/main.cpp
  src/camera_manager.cpp
  src/algorithm_client/algo_cli.cpp
  src/data_saver.cpp
)
target_include_directories(camera_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(camera_manager
  rclcpp
  sensor_msgs
  moveit_msgs

  cv_bridge
  OpenCV
  pcl_conversions
  PCL
  
  robot_controller_msgs
  robotic_platform_msgs
)
target_compile_features(camera_manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  camera_manager
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
