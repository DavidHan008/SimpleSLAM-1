cmake_minimum_required(VERSION 2.8)

project(SIMPLESLAM)

add_compile_options(-std=c++11)

add_subdirectory(Common)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${PROJECT_SOURCE_DIR}/Common/include)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(simpleslam SHARED src/SimpleSLAM.cc)
target_link_libraries(simpleslam common ${PCL_LIBRARIES})

add_executable(2dslam src/main.cc)
target_link_libraries(2dslam simpleslam)
