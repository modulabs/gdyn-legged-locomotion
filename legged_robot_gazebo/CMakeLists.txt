cmake_minimum_required(VERSION 2.8.3)
project(legged_robot_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
  COMPONENTS
    gazebo_ros
)
find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS gazebo_ros
  DEPENDS gazebo
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
