cmake_minimum_required(VERSION 3.10.2)
project(o80_pam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)


################
# Dependencies #
################

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_nose REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(pam_interface REQUIRED)
find_package(pybind11 REQUIRED)
find_package(context REQUIRED)
find_package(o80 REQUIRED)
ament_export_dependencies(
  mpi_cmake_modules
  pybind11
  context
  pam_interface
  o80)

# prepare to export all needed targets
set(all_targets)
set(all_target_exports)


###########
# Library #
###########

add_library( ${PROJECT_NAME} SHARED
  src/actuator_state.cpp
  src/robot_fk_extended_state.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  o80 context)
target_link_libraries(${PROJECT_NAME} o80::o80)
target_link_libraries(${PROJECT_NAME} pam_interface::pam_interface)
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

list(APPEND all_targets ${PROJECT_NAME})
list(APPEND all_target_exports export_${PROJECT_NAME})


###################
# Python wrappers #
###################

add_library(${PROJECT_NAME}_py MODULE srcpy/wrappers.cpp)
target_link_libraries(${PROJECT_NAME}_py PRIVATE pybind11::module)
target_link_libraries(${PROJECT_NAME}_py PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_py PRIVATE ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_py
  PROPERTIES PREFIX "" SUFFIX "${PYTHON_MODULE_EXTENSION}")
set_target_properties(${PROJECT_NAME}_py PROPERTIES
  OUTPUT_NAME ${PROJECT_NAME}_wrp)
target_include_directories(
  ${PROJECT_NAME}_py
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> SYSTEM
	 PUBLIC ${PYTHON_INCLUDE_DIRS})
_ament_cmake_python_get_python_install_dir()
install(TARGETS ${PROJECT_NAME}_py DESTINATION ${PYTHON_INSTALL_DIR})


######################
# Python Native code #
######################

# final python package o80_pam, consisting of the python sources
# (in python folder) + o80_pam_wrp (see __init__.py file in python sources)
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})


###############
# Executables #
###############

install_scripts(
  ${CMAKE_CURRENT_LIST_DIR}/bin/o80_dummy.py
  ${CMAKE_CURRENT_LIST_DIR}/bin/o80_pamy1.py
  ${CMAKE_CURRENT_LIST_DIR}/bin/o80_pamy2.py
  ${CMAKE_CURRENT_LIST_DIR}/bin/o80_plotting.py
  ${CMAKE_CURRENT_LIST_DIR}/bin/o80_console.py
  ${CMAKE_CURRENT_LIST_DIR}/bin/o80_check.py
  ${CMAKE_CURRENT_LIST_DIR}/bin/o80_swing_demo.py
  ${CMAKE_CURRENT_LIST_DIR}/bin/o80_logger.py
  ${CMAKE_CURRENT_LIST_DIR}/bin/o80_log_file_stats.py
  ${CMAKE_CURRENT_LIST_DIR}/bin/o80_introspector.py 
  ${CMAKE_CURRENT_LIST_DIR}/bin/o80_robot_ball_logger.py
  ${CMAKE_CURRENT_LIST_DIR}/bin/o80_robot_ball_replay.py 
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
  )

######################
# Install and export #
######################

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${all_targets}
  EXPORT ${all_target_exports}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)
ament_package()


##############
# Unit Tests #
##############


if(BUILD_TESTING)
  # python unit tests
  find_package(ament_cmake_nose REQUIRED)
  ament_add_nose_test(test_${PROJECT_NAME}_python tests/test_logger.py)
endif()
