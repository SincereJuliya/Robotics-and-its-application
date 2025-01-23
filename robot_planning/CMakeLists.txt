cmake_minimum_required(VERSION 3.8)
project(robot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(dataRetriver src/dataRetriver.cpp src/obstacles.cpp src/point.cpp)
add_executable(cellDecomposition src/cellDecomposition.cpp src/obstacles.cpp src/point.cpp src/graph.cpp)
add_executable(followPath src/followPath.cpp)

target_include_directories(dataRetriver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(cellDecomposition PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(dataRetriver rclcpp std_msgs geometry_msgs obstacles_msgs)
ament_target_dependencies(cellDecomposition rclcpp std_msgs geometry_msgs obstacles_msgs)
ament_target_dependencies(followPath rclcpp rclcpp_components rclcpp_action nav2_msgs std_msgs nav_msgs)

install(TARGETS
  dataRetriver
  cellDecomposition
  followPath
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
