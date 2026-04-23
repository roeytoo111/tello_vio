# Standalone mono-vo entry point.
# This is now tightly coupled with the Tello infrastructure so it should
# normally be built from the root CMakeLists.txt instead. Kept here for
# reference and quick local iteration.

cmake_minimum_required(VERSION 3.5)
project(mono-vo)

add_definitions(-std=c++17 -DASIO_STANDALONE)

find_package(OpenCV 4.0 REQUIRED)
find_package(Threads REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../inc
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib_h264decoder
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib_utils
  ${OpenCV_INCLUDE_DIRS}
)

# Gather tello sources needed by this target
file(GLOB TELLO_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../src/*.cpp)

add_executable(vo
  main.cpp
  src/vo.cpp
  ${TELLO_SOURCES}
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib_utils/utils.cpp
)

target_link_libraries(vo
  ${OpenCV_LIBRARIES}
  Threads::Threads
  h264decoder
  avcodec avutil swscale
)
