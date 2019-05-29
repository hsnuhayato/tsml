cmake_minimum_required(VERSION 2.6)

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
if (NOT CMAKE_MODULE_PATH)
  set(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/share/hrpsys/cmake_modules/ CACHE STRING "set path to cmake_modules")
endif (NOT CMAKE_MODULE_PATH)

add_definitions(-std=c++11)
####
## settings for OpenRTM and Choreonoid
##
include(FindPkgConfig)
include(CheckIncludeFiles)
set(ENV{PKG_CONFIG_PATH} "/usr/lib/x86_64-linux-gnu/pkgconfig/:$ENV{PKG_CONFIG_PATH}")

pkg_check_modules(OPENRTM REQUIRED openrtm-aist)
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})

execute_process(
  COMMAND $ENV{RTM_ROOT}/bin/rtm-config --rtm-idldir
  OUTPUT_VARIABLE OPENRTM_IDL_DIR
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
include(GenerateStubSkelPy)

pkg_check_modules(CHOREONOID REQUIRED choreonoid choreonoid-body-plugin)
include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})

include_directories(${CMAKE_MODULE_PATH})

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})
set(NO_COMP TRUE)

add_subdirectory(rtc)
add_subdirectory(customBush)
