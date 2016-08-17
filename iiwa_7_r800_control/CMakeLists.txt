cmake_minimum_required(VERSION 2.8.3)
project(iiwa_7_r800_control)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  ros_control_boilerplate
  roscpp
  rosparam_shortcuts
  roslint
  moveit_dashboard
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ros_control_boilerplate
    roscpp
    rosparam_shortcuts
    moveit_dashboard
)

###########
## Build ##
###########

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/fri_client_sdk/fri_client_sdk/include
  ${catkin_INCLUDE_DIRS}
)

# https://cmake.org/Wiki/CMake/Tutorials/Exporting_and_Importing_Targets
add_library(fri_client_sdk STATIC IMPORTED)
set_property(TARGET fri_client_sdk PROPERTY IMPORTED_LOCATION
  ${CMAKE_CURRENT_SOURCE_DIR}/fri_client_sdk/fri_client_sdk/lib/libFRIClient.a)
add_dependencies(fri_client_sdk kuka_fri_client_sdk)

# IIWA Hardware Interface
add_library(${PROJECT_NAME}
  src/iiwa_hw_interface.cpp
)
target_link_libraries(${PROJECT_NAME}
  fri_client_sdk
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(${PROJECT_NAME}_main
  src/iiwa_hw_main.cpp
  src/fri_client.cpp
)
target_link_libraries(${PROJECT_NAME}_main
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Trajectory testing executable
add_executable(${PROJECT_NAME}_test_trajectory
  src/test_trajectory.cpp
)
target_link_libraries(${PROJECT_NAME}_test_trajectory
  ${catkin_LIBRARIES}
)

# Call make to build the FRI SDK
add_subdirectory(fri_client_sdk)

#############
## Testing ##
#############

## Test for correct C++ source code
roslint_cpp()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_main ${PROJECT_NAME}_test_trajectory
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
