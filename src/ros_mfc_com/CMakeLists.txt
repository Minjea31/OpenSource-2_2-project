cmake_minimum_required(VERSION 3.0.2)
project(ros_mfc_com)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)



## OpenCV 관련 설정
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")  # OpenCV 경로 설정
find_package(OpenCV REQUIRED)


catkin_package()



include_directories(
  ${catkin_INCLUDE_DIRS}
  
  
  ${OpenCV_INCLUDE_DIRS}  # OpenCV 헤더 포함
  
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(ros_mfc_com_node src/ros_mfc_com.cpp)

target_link_libraries(ros_mfc_com_node
  ${catkin_LIBRARIES}
  
  
  ${OpenCV_LIBS}  # OpenCV 라이브러리 연결
  
  
)


