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
include beginner_tutorials/CMakeFiles/add_two_ints_client.dir/depend.make

# Include the progress variables for this target.
include beginner_tutorials/CMakeFiles/add_two_ints_client.dir/progress.make

# Include the compile flags for this target's objects.
include beginner_tutorials/CMakeFiles/add_two_ints_client.dir/flags.make

beginner_tutorials/CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.o: beginner_tutorials/CMakeFiles/add_two_ints_client.dir/flags.make
beginner_tutorials/CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.o: /home/kx/catkin_ws/src/beginner_tutorials/src/add_two_ints_client.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kx/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object beginner_tutorials/CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.o"
	cd /home/kx/catkin_ws/build/beginner_tutorials && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.o -c /home/kx/catkin_ws/src/beginner_tutorials/src/add_two_ints_client.cpp

beginner_tutorials/CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.i"
	cd /home/kx/catkin_ws/build/beginner_tutorials && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kx/catkin_ws/src/beginner_tutorials/src/add_two_ints_client.cpp > CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.i

beginner_tutorials/CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.s"
	cd /home/kx/catkin_ws/build/beginner_tutorials && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kx/catkin_ws/src/beginner_tutorials/src/add_two_ints_client.cpp -o CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.s

beginner_tutorials/CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.o.requires:

.PHONY : beginner_tutorials/CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.o.requires

beginner_tutorials/CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.o.provides: beginner_tutorials/CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.o.requires
	$(MAKE) -f beginner_tutorials/CMakeFiles/add_two_ints_client.dir/build.make beginner_tutorials/CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.o.provides.build
.PHONY : beginner_tutorials/CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.o.provides

beginner_tutorials/CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.o.provides.build: beginner_tutorials/CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.o


# Object files for target add_two_ints_client
add_two_ints_client_OBJECTS = \
"CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.o"

# External object files for target add_two_ints_client
add_two_ints_client_EXTERNAL_OBJECTS =

/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: beginner_tutorials/CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.o
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: beginner_tutorials/CMakeFiles/add_two_ints_client.dir/build.make
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /opt/ros/kinetic/lib/libroscpp.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /opt/ros/kinetic/lib/librosconsole.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /opt/ros/kinetic/lib/librostime.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /opt/ros/kinetic/lib/libcpp_common.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client: beginner_tutorials/CMakeFiles/add_two_ints_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kx/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client"
	cd /home/kx/catkin_ws/build/beginner_tutorials && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/add_two_ints_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
beginner_tutorials/CMakeFiles/add_two_ints_client.dir/build: /home/kx/catkin_ws/devel/lib/beginner_tutorials/add_two_ints_client

.PHONY : beginner_tutorials/CMakeFiles/add_two_ints_client.dir/build

beginner_tutorials/CMakeFiles/add_two_ints_client.dir/requires: beginner_tutorials/CMakeFiles/add_two_ints_client.dir/src/add_two_ints_client.cpp.o.requires

.PHONY : beginner_tutorials/CMakeFiles/add_two_ints_client.dir/requires

beginner_tutorials/CMakeFiles/add_two_ints_client.dir/clean:
	cd /home/kx/catkin_ws/build/beginner_tutorials && $(CMAKE_COMMAND) -P CMakeFiles/add_two_ints_client.dir/cmake_clean.cmake
.PHONY : beginner_tutorials/CMakeFiles/add_two_ints_client.dir/clean

beginner_tutorials/CMakeFiles/add_two_ints_client.dir/depend:
	cd /home/kx/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kx/catkin_ws/src /home/kx/catkin_ws/src/beginner_tutorials /home/kx/catkin_ws/build /home/kx/catkin_ws/build/beginner_tutorials /home/kx/catkin_ws/build/beginner_tutorials/CMakeFiles/add_two_ints_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : beginner_tutorials/CMakeFiles/add_two_ints_client.dir/depend

