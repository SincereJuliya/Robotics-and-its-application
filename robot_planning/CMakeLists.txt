cmake_minimum_required(VERSION 3.8)
project(robot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)

add_executable(borders src/listenerBorders.cpp)
add_executable(gates src/listenerGates.cpp)
add_executable(obstacles src/listenerObstacles.cpp)
add_executable(dataRetriver src/dataRetriver.cpp)

ament_target_dependencies(borders rclcpp std_msgs geometry_msgs)
ament_target_dependencies(gates rclcpp std_msgs geometry_msgs)
ament_target_dependencies(obstacles rclcpp std_msgs geometry_msgs obstacles_msgs)
ament_target_dependencies(dataRetriver rclcpp std_msgs geometry_msgs obstacles_msgs)

install(TARGETS
  borders
  gates
  obstacles
  dataRetriver
  DESTINATION lib/${PROJECT_NAME})

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
