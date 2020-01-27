cmake_minimum_required(VERSION 3.5.1)
project (vehicle_detection)

find_package(OpenCV 4 REQUIRED)
find_package(PythonLibs REQUIRED)

include_directories("${PYTHON_INCLUDE_DIRS}")
include_directories("${CMAKE_SOURCE_DIR}")


add_executable(detect yoloGPUtest.cc)

target_sources(detect PRIVATE yoloDetectorGPU.cc)
target_include_directories(detect PRIVATE ${CMAKE_SOURCE_DIR})	

target_link_libraries( detect ${OpenCV_LIBS} ${PYTHON_LIBRARIES})
