cmake_minimum_required(VERSION 2.8.0)
project(legged_robot_gui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    legged_robot_msgs
)
find_package(Qt5 REQUIRED
  COMPONENTS
    Widgets
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    legged_robot_msgs
)

set(legged_robot_gui_INCLUDE_DIRS
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)
if(NOT EXISTS ${legged_robot_gui_INCLUDE_DIRS})
  file(MAKE_DIRECTORY ${legged_robot_gui_INCLUDE_DIRS})
endif()
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${legged_robot_gui_INCLUDE_DIRS})
set(QT_FORMS
  ui/main_window.ui
)
qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(
  include
  ${legged_robot_gui_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(QT_MOC
  include/${PROJECT_NAME}/main_window.hpp
  include/${PROJECT_NAME}/qnode.hpp
)
qt5_wrap_cpp(QT_MOC_HPP ${QT_MOC})

set(QT_SRCS
  src/main_window.cpp
  src/main.cpp
  src/qnode.cpp
)

add_executable(legged_robot_gui ${QT_SRCS} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(legged_robot_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(legged_robot_gui Qt5::Widgets ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch ui
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
