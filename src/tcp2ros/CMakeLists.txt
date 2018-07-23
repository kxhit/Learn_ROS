cmake_minimum_required(VERSION 2.8.3)
project(tcp2ros)

set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  nav2d_operator
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

#find_package(PkgConfig)
#pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
#if(NEW_YAMLCPP_FOUND)
#add_definitions(-DHAVE_NEW_YAMLCPP)
#endif(NEW_YAMLCPP_FOUND)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
add_message_files(
   FILES
   readDataAll.msg
   rtkGPSmessage.msg
   reach.msg
   cmd.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs#   std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tcp2ros
#  CATKIN_DEPENDS message_generation roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


add_executable(carDriver src/main.cpp src/carDriver.cpp)
target_link_libraries(carDriver
   ${catkin_LIBRARIES}
)
# 
add_dependencies(carDriver ${${PROJECT_NAME}_generate_messages_cpp} ${catkin_EXPORTED_TARGETS})

add_executable(joystick_driver src/joystick.cpp)
target_link_libraries(joystick_driver
   ${catkin_LIBRARIES}
)
# 
add_dependencies(joystick_driver ${${PROJECT_NAME}_generate_messages_cpp} ${catkin_EXPORTED_TARGETS})

add_executable(joystick_driver2 src/joystick_pre.cpp)
target_link_libraries(joystick_driver2
   ${catkin_LIBRARIES}
)
# 
add_dependencies(joystick_driver2 ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(LaserProcess src/LaserProcess.cpp)
target_link_libraries(LaserProcess
   ${catkin_LIBRARIES}
)

add_executable(path_publish src/pathpublish.cpp src/dijkstra.cpp)
target_link_libraries(path_publish
   ${catkin_LIBRARIES}
)

#add_executable(path_publish_new src/pathpublish_new.cpp src/dijkstra.cpp)
#target_link_libraries(path_publish_new
#   ${catkin_LIBRARIES}
#)

add_executable(car_record src/car_record.cpp)
target_link_libraries(car_record
   ${catkin_LIBRARIES}
)
# 
add_dependencies(car_record ${${PROJECT_NAME}_generate_messages_cpp} ${catkin_EXPORTED_TARGETS})


add_executable(car_record_control src/car_record_control.cpp)
target_link_libraries(car_record_control
   ${catkin_LIBRARIES}
)
# 
add_dependencies(car_record_control ${${PROJECT_NAME}_generate_messages_cpp} ${catkin_EXPORTED_TARGETS})

add_executable(car_play src/car_play.cpp)
target_link_libraries(car_play
   ${catkin_LIBRARIES}
)
# 
add_dependencies(car_play ${${PROJECT_NAME}_generate_messages_cpp} ${catkin_EXPORTED_TARGETS})

add_executable(gmapping_init_record src/gmappinginitrecord.cpp)
target_link_libraries(gmapping_init_record
   ${catkin_LIBRARIES}
)
# 
add_dependencies(gmapping_init_record ${${PROJECT_NAME}_generate_messages_cpp} ${catkin_EXPORTED_TARGETS})

add_executable(avoider_cmd src/avoider_cmd.cpp src/obs_avoider.cc)
target_link_libraries(avoider_cmd
   ${catkin_LIBRARIES}
)
# 
add_dependencies(avoider_cmd ${${PROJECT_NAME}_generate_messages_cpp} ${catkin_EXPORTED_TARGETS})

add_executable(pub_map_pose src/pubmappose.cpp src/loadMap.cpp)
target_link_libraries(pub_map_pose
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
# 
add_dependencies(pub_map_pose ${${PROJECT_NAME}_generate_messages_cpp} ${catkin_EXPORTED_TARGETS})

## Declare a C++ library
# add_library(tcp2ros
#   src/${PROJECT_NAME}/tcp2ros.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(tcp2ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(tcp2ros_node src/tcp2ros_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(tcp2ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(tcp2ros_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS tcp2ros tcp2ros_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tcp2ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
