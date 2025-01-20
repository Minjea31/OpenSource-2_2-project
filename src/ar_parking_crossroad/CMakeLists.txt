cmake_minimum_required(VERSION 3.0.2)
project(ar_parking_crossroad)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ar_track_alvar_msgs
  xycar_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(ar_parking_crossroad_node src/ar_parking_crossroad.cpp)

target_link_libraries(ar_parking_crossroad_node
  ${catkin_LIBRARIES}
)

