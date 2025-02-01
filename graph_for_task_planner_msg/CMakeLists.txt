cmake_minimum_required(VERSION 3.8)
project(graph_for_task_planner_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Graph.msg"
  "msg/Edge.msg"
  "msg/Point.msg"
)

# Ensure this is last!
ament_package()
