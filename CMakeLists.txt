cmake_minimum_required(VERSION 2.8.3)
project(twist_teleop)

find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS scripts/*
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
