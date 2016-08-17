cmake_minimum_required(VERSION 2.8.3)

# Documentation: http://www.kitware.com/media/html/BuildingExternalProjectsWithCMake2.8.html
#                https://cmake.org/cmake/help/v3.0/module/ExternalProject.html
include(ExternalProject)
externalproject_add(
  kuka_fri_client_sdk
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/fri_client_sdk/
  CONFIGURE_COMMAND echo 'Not configuring'
  BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/fri_client_sdk/build/GNUMake
  BUILD_COMMAND make
  BUILD_IN_SOURCE 0
  INSTALL_COMMAND echo 'Not installing'
)