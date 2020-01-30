cmake_minimum_required(VERSION 2.8.3)
project(manipulator_study)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  actionlib_msgs
  actionlib
  control_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES manipulator_study
   CATKIN_DEPENDS
#  DEPENDS system_lib
)
set(SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/my_posController.cpp
)
set(HEADS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/manipulator_study/my_posController.hpp
)
add_library(manipulator_study ${SRCS} ${HEADS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(set_target_once src/setOnce.cpp)

target_link_libraries(set_target_once
  manipulator_study
  ${catkin_LIBRARIES}
)
add_executable(set_target_multiple src/setMultiple.cpp)

target_link_libraries(set_target_multiple
  manipulator_study
  ${catkin_LIBRARIES}
)
