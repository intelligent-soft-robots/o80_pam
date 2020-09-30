cmake_minimum_required(VERSION 2.8.7)

project(o80_pam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)

find_package(catkin REQUIRED COMPONENTS
  pam_interface
  context
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

# pybind11 package o80_pam_wrp
pybind11_add_module(o80_pam_py srcpy/wrappers.cpp)
target_link_libraries(o80_pam_py PRIVATE o80_pam ${catkin_LIBRARIES})
target_compile_definitions(o80_pam_py PRIVATE
  PAM_DEFAULT_CONFIG_FILE_PATH="${JSON_DEFAULT_CONFIG_FILE}"
  BITFILE_4DOFS_FILE_PATH="${PAM_BITFILE}")
set_target_properties(o80_pam_py PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}
  OUTPUT_NAME o80_pam_wrp
)
install(TARGETS o80_pam_py DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})

# final python package o80_pam, consisting of the python sources
# (in python folder) + o80_pam_wrp (see __init__.py file in python sources)
catkin_python_setup()


configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/o80_pam
               ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/o80_pam
               COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/o80_pam_check
               ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/o80_pam_check
               COPYONLY)

