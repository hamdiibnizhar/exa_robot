cmake_minimum_required(VERSION 2.8.3)
project(exa_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#find package
 
find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  turtlesim
)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES learning_joy
#  CATKIN_DEPENDS joy roscpp turtlesim
#  DEPENDS system_lib
)

include_directories(
# include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(exa_teleop src/exa_teleop.cpp)
target_link_libraries(exa_teleop ${catkin_LIBRARIES})
