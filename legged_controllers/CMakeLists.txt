cmake_minimum_required(VERSION 2.8.3)
project(legged_controllers)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  angles
  controller_interface
  control_msgs
  urdf
  kdl_parser
  realtime_tools
  qpoases
  # dynamic_reconfigure
)

# Dynamics reconfigure
# generate_dynamic_reconfigure_options(
#   cfg/LegControllerParams.cfg
#   )

# message
add_message_files(FILES ControllerJointState.msg)

add_service_files(
  FILES
  UpdateGain.srv
)
generate_messages(DEPENDENCIES std_msgs)

# include
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    angles
    controller_interface
    control_msgs
    urdf
    kdl_parser
    realtime_tools
    message_generation
    qpoases
    # dynamic_reconfigure
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/main_controller.cpp
  src/balance_controller.cpp
  src/virtual_spring_damper_controller.cpp
  src/utility/math_func.cpp
  src/mpc_controller.cpp
  src/legged_robot/leg.cpp
  src/legged_robot/body.cpp
  src/legged_robot/quadruped_robot.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

