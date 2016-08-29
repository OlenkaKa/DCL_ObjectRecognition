# Project name
project(ObjectRecognition)

# CMake required version must be >= 2.8
cmake_minimum_required(VERSION 2.8)

# Find DisCODe installation
find_package(DisCODe REQUIRED)

# Use CMake macros provided with DisCODe
set(CMAKE_MODULE_PATH ${DisCODe_DIR} ${CMAKE_MODULE_PATH})
include(DisCODeMacros)

# Set default install prefix to dist folder
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX
      ${CMAKE_SOURCE_DIR}/dist CACHE PATH "${CMAKE_PROJECT_NAME} install prefix" FORCE
  )
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

add_subdirectory(src)

rebuild_dcl_cache()

