cmake_minimum_required(VERSION 3.8)
project(BezierCurve)

add_compile_options(-std=c++11 -O3)

set(CMAKE_CXX_STANDARD 11)
set(EXECUTABLE_OUTPUT_PATH          ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${PROJECT_SOURCE_DIR}/lib)

set(INC_DIR ${PROJECT_SOURCE_DIR}/include)

find_package( Eigen3  REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR} ${INC_DIR} )

add_executable(BezierCurve_node src/main.cc src/NumericalMaths.cpp)
target_link_libraries(BezierCurve_node ${Eigen3_LIBRARIES} boost_system boost_filesystem boost_iostreams)

