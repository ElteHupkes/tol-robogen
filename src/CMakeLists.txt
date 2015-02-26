# CMake Required Version
cmake_minimum_required (VERSION 2.8)

# Project Name
project (ToLRobogen)

# Add customized CMAKE modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../cmake/")
include(CustomPath)

# Pass source dir to preprocessor
add_definitions(-DSOURCE_DIR=${CMAKE_SOURCE_DIR})

# Find Eigen3, added FindEigen3 in cmake folder
find_package(Eigen3 REQUIRED)

# Find Boost
find_package(Boost REQUIRED)

# Include SdfBuilder and Eigen3
include_directories(${CMAKE_SOURCE_DIR} ${SDFBUILDER_INCLUDE_PATH} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

# Set dependency list
set(TOL_DEPS ${SDFBUILDER_LIBRARIES} ${Boost_LIBRARIES})

# All files for the SDF builder
file(GLOB_RECURSE
	 TOL_SRC
	 *.cpp
)

# Compiler options
# TODO This currently assumes GCC, add Windows support in due time
add_definitions(-pedantic -Wno-long-long -Wall -Wextra -Wformat=2
                -Wredundant-decls -Wwrite-strings -Wmissing-include-dirs 
                -Wswitch-enum -Wuninitialized 
                -Wswitch-default -Winit-self -Wfloat-equal -fPIC )

# Use C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Debug Flags 
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ggdb3 -DDEBUG")

# Release flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -funroll-loops -finline-functions -fomit-frame-pointer -DNDEBUG")

# Compile sdfbuilder library
add_library(tolrobogen ${TOL_SRC} )
target_link_libraries(tolrobogen ${TOL_DEPS})

# Test executable
add_executable(test tol_robogen/test/test.cpp)
target_link_libraries(test tolrobogen ${TOL_DEPS})
