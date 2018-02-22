cmake_minimum_required(VERSION 2.8.0)

project(PCLHelper CXX)

find_package(PCL 1.8.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# PCLHelper Library
file(GLOB LIBFiles "include/*.h" "src/*.cpp")  
ADD_LIBRARY(PCLHelperLib  ${LIBFiles})  
target_link_libraries(PCLHelperLib ${PCL_LIBRARIES})

# Test Basic
include_directories("include/")
add_executable(TestBasic test/basic.cpp)
add_dependencies(TestBasic PCLHelperLib)
target_link_libraries(TestBasic PCLHelperLib ${PCL_LIBRARIES})