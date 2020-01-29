cmake_minimum_required(VERSION 3.5.1)
project (load_assets)

find_package(PythonLibs REQUIRED)

include_directories("${PYTHON_INCLUDE_DIRS}")
include_directories("${CMAKE_SOURCE_DIR}")


add_executable(load_assets assetLoad_test.cc)

target_sources(load_assets PRIVATE loadAssets.cc)
target_include_directories(load_assets PRIVATE ${CMAKE_SOURCE_DIR})

target_link_libraries( load_assets ${PYTHON_LIBRARIES})
