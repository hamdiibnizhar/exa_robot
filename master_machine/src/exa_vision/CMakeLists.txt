cmake_minimum_required(VERSION 2.8.3)
project(exa_vision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
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

add_definitions(
  -DDEBUG
  -DUSE_CPM 
  -DUSE_5_SECOND_TIMER_SAMPLE
  # -DUSE_DOSE_EQUIVALENT
  -DUSE_COLOR_MAP
  # -DUSE_DATA_RECORD_TO_FILE
  # -DUSE_CHRONO
  -DUSE_ROS_TIME
  -DUSE_PREDICTION_CIRCLE_RANGE
  -DUSE_CSV_FORMAT
)

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
# set_target_properties(exa_geiger_data_subscriber PROPERTIES OUTPUT_NAME "exa_geiger_data_sub")

# add_executable(exa_geiger_map ${SOURCES_GEIGER_DATA_MAP})
# target_link_libraries(exa_geiger_map ${catkin_LIBRARIES})

add_executable(exa_geiger_map ${SOURCES_GEIGER_DATA_MAP})
target_link_libraries(exa_geiger_map ${catkin_LIBRARIES})
# set_target_properties(exa_geiger_data_map PROPERTIES OUTPUT_NAME "exa_radiation_map")

add_executable(exa_geiger_map_new src/exa_geiger_data_map_new.cpp)
#add_dependencies(exa_data_saver nav_msgs_generate_messages_cpp)
target_link_libraries(exa_geiger_map_new ${catkin_LIBRARIES})

add_executable(exa_data_saver src/data_getter.cpp)
add_dependencies(exa_data_saver nav_msgs_generate_messages_cpp)
target_link_libraries(exa_data_saver ${catkin_LIBRARIES})

add_executable(exa_data_saver_loop1 src/data_getter_loop1.cpp)
add_dependencies(exa_data_saver_loop1 nav_msgs_generate_messages_cpp)
target_link_libraries(exa_data_saver_loop1 ${catkin_LIBRARIES})

add_executable(exa_data_saver_loop2 src/data_getter_loop2.cpp)
add_dependencies(exa_data_saver_loop2 nav_msgs_generate_messages_cpp)
target_link_libraries(exa_data_saver_loop2 ${catkin_LIBRARIES})

add_executable(exa_data_saver_loop3 src/data_getter_loop3.cpp)
add_dependencies(exa_data_saver_loop3 nav_msgs_generate_messages_cpp)
target_link_libraries(exa_data_saver_loop3 ${catkin_LIBRARIES})