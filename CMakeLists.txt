cmake_minimum_required(VERSION 2.8.7)

project(O8O_pam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  pam_interface
  o80 )

DEFINE_OS()

catkin_package(
 LIBRARIES o80_pam
 INCLUDE_DIRS include
 CATKIN_DEPENDS pam_interface o80 )

include_directories(include ${catkin_INCLUDE_DIRS} )

add_library(o80_pam
  src/actuator_state.cpp)
target_link_libraries(o80_pam ${catkin_LIBRARIES})

pybind11_add_module(pam_o80 srcpy/wrappers.cpp)
target_link_libraries(pam_o80 PRIVATE o80_pam ${catkin_LIBRARIES})
target_compile_definitions(pam_o80 PRIVATE
  PAM_DEFAULT_CONFIG_FILE_PATH="${JSON_DEFAULT_CONFIG_FILE}"
  BITFILE_4DOFS_FILE_PATH="${PAM_BITFILE}")
install(TARGETS pam_o80 DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
