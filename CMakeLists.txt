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

#add_executable(dummy_leader
#  src/dummy_leader.cpp)
#target_link_libraries(dummy_leader O8O_pam ${catkin_LIBRARIES})

#add_executable(dummy_follower
#  src/dummy_follower.cpp)
#target_link_libraries(dummy_follower O8O_pam ${catkin_LIBRARIES})


add_executable(dummy_standalone
  src/dummy_standalone.cpp)
target_link_libraries(dummy_standalone O8O_pam ${catkin_LIBRARIES})

add_executable(dummy_frontend
  src/dummy_frontend.cpp)
target_link_libraries(dummy_frontend O8O_pam ${catkin_LIBRARIES})

pybind11_add_module(pam_O8O srcpy/wrappers.cpp)
target_link_libraries(pam_O8O PRIVATE O8O_pam ${catkin_LIBRARIES})
install(TARGETS pam_O8O DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
