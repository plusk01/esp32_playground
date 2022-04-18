cmake_minimum_required(VERSION 3.10)
project(esp32imu VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

###############################################################################
# async_comm
###############################################################################

include(FetchContent)
FetchContent_Declare(async_comm
    GIT_REPOSITORY https://github.com/plusk01/async_comm
    GIT_TAG msvc-fixes)
FetchContent_MakeAvailable(async_comm)

###############################################################################
# Targets
###############################################################################

add_library(esp32imu src/serial_driver.cpp)
set_target_properties(esp32imu PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  VERSION ${PROJECT_VERSION})
target_link_libraries(esp32imu PRIVATE async_comm)
target_include_directories(esp32imu PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_compile_definitions(esp32imu PUBLIC PROJECT_VERSION="${PROJECT_VERSION}")


add_executable(screenprint src/screenprint.cpp)
target_link_libraries(screenprint esp32imu)