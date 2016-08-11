cmake_minimum_required(VERSION 2.8.3)
project(iiwa_7_r800_description)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir launch meshes/iiwa_7_r800/collision meshes/iiwa_7_r800/visual urdf)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
