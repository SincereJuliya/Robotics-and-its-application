# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.31

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/seventeen/.local/lib/python3.10/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/seventeen/.local/lib/python3.10/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/seventeen/Robotics-and-its-application/src/robot_planning

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/seventeen/Robotics-and-its-application/build/robot_planning

# Include any dependencies generated for this target.
include CMakeFiles/borders.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/borders.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/borders.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/borders.dir/flags.make

CMakeFiles/borders.dir/codegen:
.PHONY : CMakeFiles/borders.dir/codegen

CMakeFiles/borders.dir/src/listenerBorders.cpp.o: CMakeFiles/borders.dir/flags.make
CMakeFiles/borders.dir/src/listenerBorders.cpp.o: /home/seventeen/Robotics-and-its-application/src/robot_planning/src/listenerBorders.cpp
CMakeFiles/borders.dir/src/listenerBorders.cpp.o: CMakeFiles/borders.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/seventeen/Robotics-and-its-application/build/robot_planning/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/borders.dir/src/listenerBorders.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/borders.dir/src/listenerBorders.cpp.o -MF CMakeFiles/borders.dir/src/listenerBorders.cpp.o.d -o CMakeFiles/borders.dir/src/listenerBorders.cpp.o -c /home/seventeen/Robotics-and-its-application/src/robot_planning/src/listenerBorders.cpp

CMakeFiles/borders.dir/src/listenerBorders.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/borders.dir/src/listenerBorders.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/seventeen/Robotics-and-its-application/src/robot_planning/src/listenerBorders.cpp > CMakeFiles/borders.dir/src/listenerBorders.cpp.i

CMakeFiles/borders.dir/src/listenerBorders.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/borders.dir/src/listenerBorders.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/seventeen/Robotics-and-its-application/src/robot_planning/src/listenerBorders.cpp -o CMakeFiles/borders.dir/src/listenerBorders.cpp.s

# Object files for target borders
borders_OBJECTS = \
"CMakeFiles/borders.dir/src/listenerBorders.cpp.o"

# External object files for target borders
borders_EXTERNAL_OBJECTS =

borders: CMakeFiles/borders.dir/src/listenerBorders.cpp.o
borders: CMakeFiles/borders.dir/build.make
borders: /opt/ros/humble/lib/librclcpp.so
borders: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_c.so
borders: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
borders: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_cpp.so
borders: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
borders: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
borders: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_py.so
borders: /opt/ros/humble/lib/liblibstatistics_collector.so
borders: /opt/ros/humble/lib/librcl.so
borders: /opt/ros/humble/lib/librmw_implementation.so
borders: /opt/ros/humble/lib/libament_index_cpp.so
borders: /opt/ros/humble/lib/librcl_logging_spdlog.so
borders: /opt/ros/humble/lib/librcl_logging_interface.so
borders: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so
borders: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
borders: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so
borders: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
borders: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_cpp.so
borders: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_py.so
borders: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_c.so
borders: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_c.so
borders: /opt/ros/humble/lib/librcl_yaml_param_parser.so
borders: /opt/ros/humble/lib/libyaml.so
borders: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.so
borders: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.so
borders: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
borders: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
borders: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
borders: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_py.so
borders: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_c.so
borders: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_c.so
borders: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_c.so
borders: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_cpp.so
borders: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
borders: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
borders: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
borders: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_py.so
borders: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_c.so
borders: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_c.so
borders: /opt/ros/humble/lib/libtracetools.so
borders: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so
borders: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.so
borders: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_c.so
borders: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
borders: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
borders: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_cpp.so
borders: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.so
borders: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_cpp.so
borders: /opt/ros/humble/lib/libfastcdr.so.1.0.24
borders: /opt/ros/humble/lib/librmw.so
borders: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
borders: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
borders: /opt/ros/humble/lib/librosidl_typesupport_introspection_cpp.so
borders: /opt/ros/humble/lib/librosidl_typesupport_introspection_c.so
borders: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_cpp.so
borders: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
borders: /opt/ros/humble/lib/librosidl_typesupport_cpp.so
borders: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_py.so
borders: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_c.so
borders: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_c.so
borders: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_c.so
borders: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_c.so
borders: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_py.so
borders: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
borders: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_c.so
borders: /opt/ros/humble/lib/librosidl_typesupport_c.so
borders: /opt/ros/humble/lib/librcpputils.so
borders: /opt/ros/humble/lib/librosidl_runtime_c.so
borders: /opt/ros/humble/lib/librcutils.so
borders: /usr/lib/x86_64-linux-gnu/libpython3.10.so
borders: CMakeFiles/borders.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/seventeen/Robotics-and-its-application/build/robot_planning/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable borders"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/borders.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/borders.dir/build: borders
.PHONY : CMakeFiles/borders.dir/build

CMakeFiles/borders.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/borders.dir/cmake_clean.cmake
.PHONY : CMakeFiles/borders.dir/clean

CMakeFiles/borders.dir/depend:
	cd /home/seventeen/Robotics-and-its-application/build/robot_planning && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/seventeen/Robotics-and-its-application/src/robot_planning /home/seventeen/Robotics-and-its-application/src/robot_planning /home/seventeen/Robotics-and-its-application/build/robot_planning /home/seventeen/Robotics-and-its-application/build/robot_planning /home/seventeen/Robotics-and-its-application/build/robot_planning/CMakeFiles/borders.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/borders.dir/depend
