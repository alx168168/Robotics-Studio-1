# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/student/ros2_ws/src/lab

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/ros2_ws/src/lab/build

# Include any dependencies generated for this target.
include CMakeFiles/lab4.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/lab4.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/lab4.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lab4.dir/flags.make

CMakeFiles/lab4.dir/src/lab4.cpp.o: CMakeFiles/lab4.dir/flags.make
CMakeFiles/lab4.dir/src/lab4.cpp.o: ../src/lab4.cpp
CMakeFiles/lab4.dir/src/lab4.cpp.o: CMakeFiles/lab4.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/ros2_ws/src/lab/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/lab4.dir/src/lab4.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/lab4.dir/src/lab4.cpp.o -MF CMakeFiles/lab4.dir/src/lab4.cpp.o.d -o CMakeFiles/lab4.dir/src/lab4.cpp.o -c /home/student/ros2_ws/src/lab/src/lab4.cpp

CMakeFiles/lab4.dir/src/lab4.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lab4.dir/src/lab4.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/ros2_ws/src/lab/src/lab4.cpp > CMakeFiles/lab4.dir/src/lab4.cpp.i

CMakeFiles/lab4.dir/src/lab4.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lab4.dir/src/lab4.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/ros2_ws/src/lab/src/lab4.cpp -o CMakeFiles/lab4.dir/src/lab4.cpp.s

# Object files for target lab4
lab4_OBJECTS = \
"CMakeFiles/lab4.dir/src/lab4.cpp.o"

# External object files for target lab4
lab4_EXTERNAL_OBJECTS =

lab4: CMakeFiles/lab4.dir/src/lab4.cpp.o
lab4: CMakeFiles/lab4.dir/build.make
lab4: /opt/ros/humble/lib/libnav_msgs__rosidl_typesupport_fastrtps_c.so
lab4: /opt/ros/humble/lib/libnav_msgs__rosidl_typesupport_fastrtps_cpp.so
lab4: /opt/ros/humble/lib/libnav_msgs__rosidl_typesupport_introspection_c.so
lab4: /opt/ros/humble/lib/libnav_msgs__rosidl_typesupport_introspection_cpp.so
lab4: /opt/ros/humble/lib/libnav_msgs__rosidl_typesupport_cpp.so
lab4: /opt/ros/humble/lib/libnav_msgs__rosidl_generator_py.so
lab4: /opt/ros/humble/lib/libnav_msgs__rosidl_typesupport_c.so
lab4: /opt/ros/humble/lib/libnav_msgs__rosidl_generator_c.so
lab4: /usr/lib/x86_64-linux-gnu/liborocos-kdl.so
lab4: /opt/ros/humble/lib/libtf2_ros.so
lab4: /opt/ros/humble/lib/libtf2.so
lab4: /opt/ros/humble/lib/libmessage_filters.so
lab4: /opt/ros/humble/lib/librclcpp_action.so
lab4: /opt/ros/humble/lib/librclcpp.so
lab4: /opt/ros/humble/lib/liblibstatistics_collector.so
lab4: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.so
lab4: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.so
lab4: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
lab4: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
lab4: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
lab4: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_py.so
lab4: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_c.so
lab4: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_c.so
lab4: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_c.so
lab4: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_cpp.so
lab4: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
lab4: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
lab4: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
lab4: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_py.so
lab4: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_c.so
lab4: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_c.so
lab4: /opt/ros/humble/lib/librcl_action.so
lab4: /opt/ros/humble/lib/librcl.so
lab4: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so
lab4: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
lab4: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so
lab4: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
lab4: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_cpp.so
lab4: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_py.so
lab4: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_c.so
lab4: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_c.so
lab4: /opt/ros/humble/lib/librcl_yaml_param_parser.so
lab4: /opt/ros/humble/lib/libyaml.so
lab4: /opt/ros/humble/lib/libtracetools.so
lab4: /opt/ros/humble/lib/librmw_implementation.so
lab4: /opt/ros/humble/lib/libament_index_cpp.so
lab4: /opt/ros/humble/lib/librcl_logging_spdlog.so
lab4: /opt/ros/humble/lib/librcl_logging_interface.so
lab4: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_fastrtps_c.so
lab4: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_c.so
lab4: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so
lab4: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_fastrtps_c.so
lab4: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.so
lab4: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_fastrtps_c.so
lab4: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_c.so
lab4: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_introspection_c.so
lab4: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
lab4: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
lab4: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_introspection_c.so
lab4: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
lab4: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_c.so
lab4: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_fastrtps_cpp.so
lab4: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_cpp.so
lab4: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_cpp.so
lab4: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_fastrtps_cpp.so
lab4: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.so
lab4: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_fastrtps_cpp.so
lab4: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_cpp.so
lab4: /opt/ros/humble/lib/libfastcdr.so.1.0.24
lab4: /opt/ros/humble/lib/librmw.so
lab4: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_introspection_cpp.so
lab4: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
lab4: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
lab4: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_introspection_cpp.so
lab4: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
lab4: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_cpp.so
lab4: /opt/ros/humble/lib/librosidl_typesupport_introspection_cpp.so
lab4: /opt/ros/humble/lib/librosidl_typesupport_introspection_c.so
lab4: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_cpp.so
lab4: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
lab4: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_cpp.so
lab4: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_cpp.so
lab4: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
lab4: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_cpp.so
lab4: /opt/ros/humble/lib/librosidl_typesupport_cpp.so
lab4: /opt/ros/humble/lib/libtf2_msgs__rosidl_generator_py.so
lab4: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_py.so
lab4: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_py.so
lab4: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_c.so
lab4: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_c.so
lab4: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_c.so
lab4: /opt/ros/humble/lib/libtf2_msgs__rosidl_generator_c.so
lab4: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_c.so
lab4: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_c.so
lab4: /opt/ros/humble/lib/libaction_msgs__rosidl_generator_py.so
lab4: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_py.so
lab4: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_c.so
lab4: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
lab4: /opt/ros/humble/lib/libaction_msgs__rosidl_generator_c.so
lab4: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_c.so
lab4: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_generator_py.so
lab4: /usr/lib/x86_64-linux-gnu/libpython3.10.so
lab4: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_c.so
lab4: /opt/ros/humble/lib/librosidl_typesupport_c.so
lab4: /opt/ros/humble/lib/librcpputils.so
lab4: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_generator_c.so
lab4: /opt/ros/humble/lib/librosidl_runtime_c.so
lab4: /opt/ros/humble/lib/librcutils.so
lab4: CMakeFiles/lab4.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/student/ros2_ws/src/lab/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable lab4"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lab4.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lab4.dir/build: lab4
.PHONY : CMakeFiles/lab4.dir/build

CMakeFiles/lab4.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lab4.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lab4.dir/clean

CMakeFiles/lab4.dir/depend:
	cd /home/student/ros2_ws/src/lab/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/ros2_ws/src/lab /home/student/ros2_ws/src/lab /home/student/ros2_ws/src/lab/build /home/student/ros2_ws/src/lab/build /home/student/ros2_ws/src/lab/build/CMakeFiles/lab4.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lab4.dir/depend

