cmake_minimum_required(VERSION 3.0.2)
project(line_following)

## Find Catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  ackermann_msgs
  cv_bridge
  image_transport
  ar_track_alvar_msgs
)

## System dependencies
find_package(OpenCV REQUIRED)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs ackermann_msgs cv_bridge image_transport ar_track_alvar_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Add executable for the main node
add_executable(line_following_node src/line_following.cpp)

## Use C++17
target_compile_features(line_following_node PUBLIC cxx_std_17)

## Specify libraries to link a library or executable target against
target_link_libraries(line_following_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

