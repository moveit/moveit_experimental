cmake_minimum_required(VERSION 2.8.3)
project(moveit_teleop)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  moveit_boilerplate
  dashboard_msgs
  roslint
)

find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    moveit_boilerplate
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_remote_control
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Encapsulate all external control inputs from user
add_library(${PROJECT_NAME}_remote_control
  src/remote_control.cpp
)
target_link_libraries(${PROJECT_NAME}_remote_control
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Teleop-specific functionality
add_library(${PROJECT_NAME}
  src/teleoperation.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_remote_control
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Main executable
add_executable(${PROJECT_NAME}_main
  src/teleoperation_main.cpp
)
target_link_libraries(${PROJECT_NAME}_main
  ${PROJECT_NAME}
  gflags
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Test for correct C++ source code
roslint_cpp()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_remote_control ${PROJECT_NAME}_main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark roslaunch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE
)

# Mark config files for installation
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml"
  PATTERN ".svn" EXCLUDE
)
