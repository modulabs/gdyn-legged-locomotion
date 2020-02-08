cmake_minimum_required(VERSION 2.8.3)
project(legged_robot_teleop)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    legged_robot_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp
    legged_robot_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(legged_robot_teleop_joystick src/legged_robot_teleop_joystick.cpp)
add_dependencies(legged_robot_teleop_joystick ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(legged_robot_teleop_joystick ${catkin_LIBRARIES} )

add_executable(legged_robot_teleop_keyboard src/legged_robot_teleop_keyboard.cpp)
add_dependencies(legged_robot_teleop_keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(legged_robot_teleop_keyboard ${catkin_LIBRARIES} )

install(TARGETS legged_robot_teleop_joystick legged_robot_teleop_keyboard 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
