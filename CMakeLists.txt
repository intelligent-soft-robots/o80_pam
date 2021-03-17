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
find_package(pybind11_vendor REQUIRED)
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
  src/actuator_state.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  o80 context)
target_link_libraries(${PROJECT_NAME} o80::o80)
target_link_libraries(${PROJECT_NAME} pam_interface::pam_interface)
#target_link_libraries(${PROJECT_NAME} context::context)
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
target_compile_definitions(${PROJECT_NAME}_py PRIVATE
  PAM_DEFAULT_CONFIG_FILE_PATH="${JSON_DEFAULT_CONFIG_FILE}"
  BITFILE_4DOFS_FILE_PATH="${PAM_BITFILE}")
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

configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/o80_dummy
  ${CMAKE_INSTALL_PREFIX}/bin/o80_dummy COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/o80_real
  ${CMAKE_INSTALL_PREFIX}/bin/o80_real COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/o80_plotting
  ${CMAKE_INSTALL_PREFIX}/bin/o80_plotting COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/o80_console
  ${CMAKE_INSTALL_PREFIX}/bin/o80_console COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/o80_check
  ${CMAKE_INSTALL_PREFIX}/bin/o80_check COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/o80_swing_demo
  ${CMAKE_INSTALL_PREFIX}/bin/o80_swing_demo COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/o80_joint_control_demo
  ${CMAKE_INSTALL_PREFIX}/bin/o80_joint_control_demo COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/o80_logger
  ${CMAKE_INSTALL_PREFIX}/bin/o80_logger COPYONLY)
configure_file(${CMAKE_CURRENT_LIST_DIR}/bin/o80_log_file_stats
  ${CMAKE_INSTALL_PREFIX}/bin/o80_log_file_stats COPYONLY)


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
