cmake_minimum_required(VERSION 2.8.3)
project(ur5e_description)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch REQUIRED)

install(DIRECTORY cfg config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
