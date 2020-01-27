cmake_minimum_required(VERSION 3.5.1)
project (coordinate_transformation)

find_package(PythonLibs REQUIRED)

include_directories("${PYTHON_INCLUDE_DIRS}")
include_directories("${CMAKE_SOURCE_DIR}")


add_executable(estimate_transformation estimateTransformation_test.cc)

target_sources(estimate_transformation PRIVATE estimateTransformation.cc)
target_include_directories(estimate_transformation PRIVATE ${CMAKE_SOURCE_DIR})

target_link_libraries( estimate_transformation ${PYTHON_LIBRARIES})
