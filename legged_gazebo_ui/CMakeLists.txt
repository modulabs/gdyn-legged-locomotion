cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_AUTOMOC ON)
project(legged_gazebo_ui)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  legged_controllers
)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  include
  SYSTEM 
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS} 
  ${Qt5Core_INCLUDE_DIRS})

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(
  legged_gazebo_ui SHARED ${headers_MOC}
  src/LeggedGazeboUI.cpp
)

target_link_libraries(
  legged_gazebo_ui
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${OGRE_LIBRARIES}
  ${OGRE-Terrain_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

add_dependencies(
  legged_gazebo_ui
  ${catkin_EXPORTED_TARGETS}
)