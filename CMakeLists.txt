project(tello)

cmake_minimum_required(VERSION 3.5)

add_definitions(-std=c++17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -DDEBUG")

# These should be the only options the user requires

option(SIMPLE "Use simple formatting for all output to terminal" OFF)
option(USE_JOYSTICK "Use a joystick/controller to control the drone manually" ON)
option(RECORD "Record tello video feed" OFF)
option(RUN_SLAM "Run SLAM in real time" OFF)
option(USE_TERMINAL "Run with terminal for CLI" OFF)
option(USE_CONFIG "Use configuration file to set up tello(s)" OFF)

message(STATUS "CMake Option `SIMPLE` - Use simple formatting for all output to terminal ${SIMPLE}")
message(STATUS "CMake Option `USE_JOYSTICK` - Use a joystick/controller to control the drone manually ${USE_JOYSTICK}")
message(STATUS "CMake Option `RECORD` - Record tello video feed ${RECORD}")
message(STATUS "CMake Option `RUN_SLAM` - Run SLAM in real time ${RUN_SLAM}")
message(STATUS "CMake Option `USE_TERMINAL` - `Run with terminal for CLI` ${USE_TERMINAL}")
message(STATUS "CMake_Option `USE_CONFIG` - Use configuration file to set up tello(s) ${USE_CONFIG}")

# End These should be the only options the user requires

option(USE_CMAKE_NOT_SCRIPT "Use Cmake ExternalProject to get and build OpenVSLAM and its dependencies" OFF)
option(REBUILD_OPENVSLAM "Rebuild OpenVSLAM" OFF)

# Find Eigen3 first (before setting include directories)
find_path(EIGEN3_INCLUDE_DIR 
  NAMES Eigen/Dense
  PATHS
    /usr/include/eigen3
    /usr/local/include/eigen3
    /opt/homebrew/include/eigen3
    /usr/include
    /usr/local/include
  NO_DEFAULT_PATH
)

if(EIGEN3_INCLUDE_DIR)
  set(EIGEN3_FOUND TRUE)
  message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")
else()
  # Try find_package as fallback
  find_package(Eigen3 QUIET)
  if(Eigen3_FOUND)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
    message(STATUS "Found Eigen3 via find_package: ${EIGEN3_INCLUDE_DIR}")
  else()
    message(FATAL_ERROR "Eigen3 not found. VIO functionality requires Eigen3.")
    message(FATAL_ERROR "Install with: sudo apt install libeigen3-dev")
    message(FATAL_ERROR "Or download from: https://eigen.tuxfamily.org/")
  endif()
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc
                    ${CMAKE_CURRENT_SOURCE_DIR}/lib_h264decoder
                    ${CMAKE_CURRENT_SOURCE_DIR}/lib_utils
                    ${CMAKE_CURRENT_SOURCE_DIR}/lib_openvslam/openvslam/src
                    ${CMAKE_CURRENT_SOURCE_DIR}/monocular_vo/include
                    ${EIGEN3_INCLUDE_DIR}
                   )

# Add joystick include directory only if it exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib_joystick)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib_joystick)
endif()

file( GLOB LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file( GLOB LIB_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp)

# Check if joystick directory exists and USE_JOYSTICK is enabled
set(JOYSTICK_ENABLED FALSE)
if(USE_JOYSTICK)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib_joystick)
    file( GLOB LIB_SOURCES_JOYSTICK ${CMAKE_CURRENT_SOURCE_DIR}/lib_joystick/*.cpp)
    file( GLOB LIB_HEADERS_JOYSTICK ${CMAKE_CURRENT_SOURCE_DIR}/lib_joystick/*.hpp)
    file( GLOB LIB_HEADERS_JOYSTICK_H ${CMAKE_CURRENT_SOURCE_DIR}/lib_joystick/*.h)
    list(APPEND LIB_HEADERS_JOYSTICK ${LIB_HEADERS_JOYSTICK_H})
    if(LIB_SOURCES_JOYSTICK)
      set(JOYSTICK_ENABLED TRUE)
      message(STATUS "Joystick support enabled: Found lib_joystick directory")
    else()
      message(WARNING "USE_JOYSTICK is ON but no joystick source files found in lib_joystick. Disabling joystick support.")
      set(USE_JOYSTICK OFF)
    endif()
  else()
    message(WARNING "USE_JOYSTICK is ON but lib_joystick directory not found. Disabling joystick support.")
    set(USE_JOYSTICK OFF)
  endif()
endif()

if(SIMPLE)
  add_definitions(-DSIMPLE)
endif(SIMPLE)

add_definitions(-DASIO_STANDALONE)
# __cplusplus macro not being set corretly in Travis CI,
# If __cplusplus >  2011... asio shoould be able to compile without boost.
# Succeeds locally but fails in travis CI.
# TODO: Debug this.

# Only define USE_JOYSTICK if joystick support is actually enabled
if(JOYSTICK_ENABLED)
  add_definitions(-DUSE_JOYSTICK)
endif()

if(RECORD)
  add_definitions(-DRECORD)
endif(RECORD)

if(USE_TERMINAL)
  add_definitions(-DUSE_TERMINAL)
endif(USE_TERMINAL)

if(USE_CONFIG)
  find_package(yaml-cpp REQUIRED)
  add_definitions(-DUSE_CONFIG)
endif(USE_CONFIG)

find_package(Threads REQUIRED)

add_library( h264decoder SHARED
            ${CMAKE_CURRENT_SOURCE_DIR}/lib_h264decoder/h264decoder.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/lib_h264decoder/h264decoder.hpp
            )

target_link_libraries(h264decoder
                      avcodec
                      avutil
                      swscale)

add_library( utils SHARED
             ${CMAKE_CURRENT_SOURCE_DIR}/lib_utils/utils.cpp
             ${CMAKE_CURRENT_SOURCE_DIR}/lib_utils/utils.hpp
           )

# Build joystick library only if enabled and files exist
if(JOYSTICK_ENABLED)
  add_library( joystick SHARED
               ${LIB_SOURCES_JOYSTICK}
               ${LIB_HEADERS_JOYSTICK}
             )
endif()

add_executable( ${PROJECT_NAME}
                ${CMAKE_CURRENT_SOURCE_DIR}/main/main.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/monocular_vo/src/vo.cpp
                ${LIB_SOURCES}
                ${LIB_HEADERS}
              )

# Simple tool to check that the Tello is sending IMU/state data (separate from main app)
add_executable( tello_imu_check
                ${CMAKE_CURRENT_SOURCE_DIR}/main/check_imu.cpp
              )

if(RUN_SLAM)
  add_definitions(-DRUN_SLAM)
  add_subdirectory(lib_openvslam)
  add_dependencies( ${PROJECT_NAME} openvslam_with_api )
  include_directories(${LIB_OPENVSLAM_WITH_API_HEADERS})
  target_link_libraries( ${PROJECT_NAME} openvslam_with_api )
else(RUN_SLAM)
  find_package(OpenCV REQUIRED COMPONENTS core imgcodecs videoio features2d calib3d highgui imgproc video)
  target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
endif(RUN_SLAM)

if(USE_CONFIG)
  target_link_libraries( ${PROJECT_NAME} yaml-cpp)
endif(USE_CONFIG)

target_link_libraries( ${PROJECT_NAME}
                       Threads::Threads
                       h264decoder
                       utils
                     )

target_link_libraries( tello_imu_check
                       Threads::Threads
                     )

# Link joystick library only if enabled
if(JOYSTICK_ENABLED)
  target_link_libraries( ${PROJECT_NAME} joystick)
endif()
