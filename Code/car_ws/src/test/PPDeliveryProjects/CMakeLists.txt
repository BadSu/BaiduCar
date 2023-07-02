cmake_minimum_required(VERSION 3.24)

set (TARGET_NAME "ppnc_smart_detection")
project(ppnc_apps)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -Wall")

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories("/usr/local/include/ppnc")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


add_executable(${TARGET_NAME} cpp_demo_detection.cc)

target_link_libraries(${TARGET_NAME} PRIVATE "/usr/local/lib/ppnc/libppnc_runtime.so")
target_link_libraries( ${TARGET_NAME} PRIVATE ${OpenCV_LIBS} )