cmake_minimum_required(VERSION 2.8.3)
project(exa_vision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  pcl_conversions 
  pcl_ros
  rtabmap_ros
)

find_package(RTABMap REQUIRED)

find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)

# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# add_compile_definitions(USE_CPS_ONLY)

add_definitions(-DUSE_CPS_ONLY)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES exa_vision
  # CATKIN_DEPENDS roscpp std_msgs geometry_msgs pcl_conversions pcl_ros rtabmap_ros
  # DEPENDS system_lib
)

set(SOURCES_GEIGER_SUBSCIBER
#  src/exa_geriger_data_subscriber.cpp
  src/geiger_data_receiver_standalone.cpp
#  src/geiger_data_map.cpp
)

set(SOURCES_GEIGER_DATA_MAP
#  src/exa_geriger_data_subscriber.cpp
#  src/geiger_data_receiver_standalone.cpp
  # src/geiger_data_map.cpp
  src/exa_geiger_data_map.cpp
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(exa_geiger_subscriber ${SOURCES_GEIGER_SUBSCIBER})
target_link_libraries(exa_geiger_subscriber ${catkin_LIBRARIES})


# add_executable(exa_geiger_map ${SOURCES_GEIGER_DATA_MAP})
# target_link_libraries(exa_geiger_map ${catkin_LIBRARIES})

add_executable(exa_geiger_map ${SOURCES_GEIGER_DATA_MAP})
target_link_libraries(exa_geiger_map ${catkin_LIBRARIES})
