# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kx/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kx/catkin_ws/build

# Include any dependencies generated for this target.
include device_tutorials/CMakeFiles/example1.dir/depend.make

# Include the progress variables for this target.
include device_tutorials/CMakeFiles/example1.dir/progress.make

# Include the compile flags for this target's objects.
include device_tutorials/CMakeFiles/example1.dir/flags.make

device_tutorials/CMakeFiles/example1.dir/src/example1.cpp.o: device_tutorials/CMakeFiles/example1.dir/flags.make
device_tutorials/CMakeFiles/example1.dir/src/example1.cpp.o: /home/kx/catkin_ws/src/device_tutorials/src/example1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kx/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object device_tutorials/CMakeFiles/example1.dir/src/example1.cpp.o"
	cd /home/kx/catkin_ws/build/device_tutorials && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example1.dir/src/example1.cpp.o -c /home/kx/catkin_ws/src/device_tutorials/src/example1.cpp

device_tutorials/CMakeFiles/example1.dir/src/example1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example1.dir/src/example1.cpp.i"
	cd /home/kx/catkin_ws/build/device_tutorials && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kx/catkin_ws/src/device_tutorials/src/example1.cpp > CMakeFiles/example1.dir/src/example1.cpp.i

device_tutorials/CMakeFiles/example1.dir/src/example1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example1.dir/src/example1.cpp.s"
	cd /home/kx/catkin_ws/build/device_tutorials && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kx/catkin_ws/src/device_tutorials/src/example1.cpp -o CMakeFiles/example1.dir/src/example1.cpp.s

device_tutorials/CMakeFiles/example1.dir/src/example1.cpp.o.requires:

.PHONY : device_tutorials/CMakeFiles/example1.dir/src/example1.cpp.o.requires

device_tutorials/CMakeFiles/example1.dir/src/example1.cpp.o.provides: device_tutorials/CMakeFiles/example1.dir/src/example1.cpp.o.requires
	$(MAKE) -f device_tutorials/CMakeFiles/example1.dir/build.make device_tutorials/CMakeFiles/example1.dir/src/example1.cpp.o.provides.build
.PHONY : device_tutorials/CMakeFiles/example1.dir/src/example1.cpp.o.provides

device_tutorials/CMakeFiles/example1.dir/src/example1.cpp.o.provides.build: device_tutorials/CMakeFiles/example1.dir/src/example1.cpp.o


# Object files for target example1
example1_OBJECTS = \
"CMakeFiles/example1.dir/src/example1.cpp.o"

# External object files for target example1
example1_EXTERNAL_OBJECTS =

/home/kx/catkin_ws/devel/lib/device_tutorials/example1: device_tutorials/CMakeFiles/example1.dir/src/example1.cpp.o
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: device_tutorials/CMakeFiles/example1.dir/build.make
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /opt/ros/kinetic/lib/libroscpp.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /opt/ros/kinetic/lib/librosconsole.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /opt/ros/kinetic/lib/librostime.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /opt/ros/kinetic/lib/libcpp_common.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/kx/catkin_ws/devel/lib/device_tutorials/example1: device_tutorials/CMakeFiles/example1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kx/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/kx/catkin_ws/devel/lib/device_tutorials/example1"
	cd /home/kx/catkin_ws/build/device_tutorials && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
device_tutorials/CMakeFiles/example1.dir/build: /home/kx/catkin_ws/devel/lib/device_tutorials/example1

.PHONY : device_tutorials/CMakeFiles/example1.dir/build

device_tutorials/CMakeFiles/example1.dir/requires: device_tutorials/CMakeFiles/example1.dir/src/example1.cpp.o.requires

.PHONY : device_tutorials/CMakeFiles/example1.dir/requires

device_tutorials/CMakeFiles/example1.dir/clean:
	cd /home/kx/catkin_ws/build/device_tutorials && $(CMAKE_COMMAND) -P CMakeFiles/example1.dir/cmake_clean.cmake
.PHONY : device_tutorials/CMakeFiles/example1.dir/clean

device_tutorials/CMakeFiles/example1.dir/depend:
	cd /home/kx/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kx/catkin_ws/src /home/kx/catkin_ws/src/device_tutorials /home/kx/catkin_ws/build /home/kx/catkin_ws/build/device_tutorials /home/kx/catkin_ws/build/device_tutorials/CMakeFiles/example1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : device_tutorials/CMakeFiles/example1.dir/depend

