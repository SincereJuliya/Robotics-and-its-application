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
find_package(graph_for_task_planner_msg REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Map generator node (supports both Sample- and Cell-based generators)
add_executable(mapGeneratorNode
  src/MapGeneratorNode.cpp
  src/graph.cpp
  src/point.cpp
  src/obstacles.cpp
  src/IMapGenerator.cpp 
  src/SampleBasedMapGenerator.cpp
  src/CellDecompositionMapGenerator.cpp
)
add_executable(taskPlannerNode 
  src/TaskPlannerNode.cpp 
  src/graph.cpp 
  src/point.cpp 
  src/IPathPlanner.cpp
  src/AStar.cpp
)
add_executable(followPath src/followPath.cpp)

ament_target_dependencies(mapGeneratorNode
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  obstacles_msgs
  graph_for_task_planner_msg
  nav_msgs
  nav2_msgs
)
ament_target_dependencies(taskPlannerNode rclcpp std_msgs geometry_msgs obstacles_msgs graph_for_task_planner_msg)
ament_target_dependencies(followPath rclcpp rclcpp_components rclcpp_action nav2_msgs std_msgs nav_msgs)

install(TARGETS
  mapGeneratorNode
  taskPlannerNode
  followPath
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
