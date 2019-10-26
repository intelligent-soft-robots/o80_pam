cmake_minimum_required(VERSION 2.8.7)

project(O8O_pam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  pam_interface
  O8O )

DEFINE_OS()

catkin_package(
 LIBRARIES O8O_pam
 INCLUDE_DIRS include
 CATKIN_DEPENDS pam_interface O8O )

include_directories(include ${catkin_INCLUDE_DIRS} )

add_library(O8O_pam
  src/o8o_pam_actuator_state.cpp)
target_link_libraries(O8O_pam ${catkin_LIBRARIES})

add_executable(dummy_standalone
  src/dummy_standalone.cpp)
target_link_libraries(dummy_standalone O8O_pam ${catkin_LIBRARIES})

