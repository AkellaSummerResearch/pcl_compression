cmake_minimum_required(VERSION 2.8.3)
project(pcl_compression)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    cmake_modules
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcl_compression src/pcl_compression.cpp)
target_link_libraries(pcl_compression ${catkin_LIBRARIES})

link_directories(
    ${PCL_LIBRARY_DIRS})