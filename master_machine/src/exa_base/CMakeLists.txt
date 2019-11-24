cmake_minimum_required(VERSION 2.8.3)
project(exa_base)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  tf
  tf2
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES exa_base
#  CATKIN_DEPENDS geometry_msgs roscpp rospy tf tf2
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set(CAMERA_BASE_CONFIG
#  src/exa_geriger_data_subscriber.cpp
#  src/geiger_data_receiver_standalone.cpp
  # src/geiger_data_map.cpp
  src/camera_base_broadcaster.cpp
)

add_executable(base_base_config src/base_broadcaste.cpp)
#add_dependencies(exa_data_saver nav_msgs_generate_messages_cpp)
target_link_libraries(base_base_config ${catkin_LIBRARIES})

add_executable(depth_base_config src/depth_base_broadcaster.cpp)
#add_dependencies(exa_data_saver nav_msgs_generate_messages_cpp)
target_link_libraries(depth_base_config ${catkin_LIBRARIES})

add_executable(camera_base_config src/camera_base_broadcaster.cpp)
target_link_libraries(camera_base_config ${catkin_LIBRARIES})

add_executable(geiger_base_config src/geiger_base_broadcaster.cpp)
#add_dependencies(exa_data_saver nav_msgs_generate_messages_cpp)
target_link_libraries(geiger_base_config ${catkin_LIBRARIES})

add_executable(odom_base_config src/odom_base_broadcaster.cpp)
#add_dependencies(exa_data_saver nav_msgs_generate_messages_cpp)
target_link_libraries(odom_base_config ${catkin_LIBRARIES})
