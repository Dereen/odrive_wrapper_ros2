# Copyright (c) JettyVision s.r.o in Prague  - All Rights Reserved
# Created on: 21.5.2021
#     Author: Valentyn Cihala <cihala@jettyvision.cz>

#######################
# include CMake helpers
#######################

# provides default CMAKE_INSTALL_<dir> variables
include(GNUInstallDirs)
# helpers to generate <package>Config.cmake
include(CMakePackageConfigHelpers)

cmake_minimum_required(VERSION 2.8.12)
project(odrive_wrapper)
set(ODRIVE_WRAPPER_VERSION 1.0)
set(CMAKE_CXX_STANDARD 11)

# install location of generated cmake config files
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake)

# Output more warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++11 -Wall -pthread -Wextra -Wpedantic -fPIC)
endif()

###################
# target defintions
###################

#include_directories(include)

add_library(${PROJECT_NAME}_lib src/candevice.cpp src/odrivecan.cpp src/odriveaxis.cpp)
# target sources is IDE friendly (especially for INTERFACE libs)
target_sources(${CMAKE_PROJECT_NAME}_lib PRIVATE
  src/candevice.cpp src/odriveaxis.cpp src/odrivecan.cpp)
# different locations on build and after install
target_include_directories(${CMAKE_PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_lib -pthread)

add_executable(odrivecan_test src/main.cpp)
target_link_libraries(odrivecan_test ${PROJECT_NAME}_lib)

#################
# install targets
#################

# libs and binaries, generate export targets
install(TARGETS ${CMAKE_PROJECT_NAME}_lib
  EXPORT ${CMAKE_PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
# header files, note the "/" after include
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

################
# export targets
################

# for build tree / in workspace
export(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
  NAMESPACE ${CMAKE_PROJECT_NAME}::)
# exported targets
install(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_PROJECT_NAME}Targets.cmake
  NAMESPACE ${CMAKE_PROJECT_NAME}::
  DESTINATION ${ConfigPackageLocation})

###########################
# CMake package generation
###########################

# generate package version file
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  VERSION ${ODRIVE_WRAPPER_VERSION}
  COMPATIBILITY SameMajorVersion)

# generate package configuration file
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
configure_package_config_file(
  ${CMAKE_PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
  PATH_VARS INCLUDE_INSTALL_DIR)

# install the package configuration files
install(FILES 
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation})
