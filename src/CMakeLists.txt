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
find_package(Boost COMPONENTS regex system)

# Include SdfBuilder, Eigen3 and Boost
include_directories(${CMAKE_SOURCE_DIR} ${SDFBUILDER_INCLUDE_PATH} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

# Set dependency list (Eigen3 is header only)
set(TOL_DEPS ${SDFBUILDER_LIBRARIES} ${Boost_LIBRARIES})

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
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -funroll-loops -finline-functions -fomit-frame-pointer -DNDEBUG")


#### TOL Robogen
# All files for TOL Robogen
file(GLOB_RECURSE
	 TOL_SRC
	 tol_robogen/configuration/*.cpp
	 tol_robogen/evolution/*.cpp
	 tol_robogen/model/*.cpp
)

# Compile tolrobogen library
add_library(tolrobogen ${TOL_SRC} )
target_link_libraries(tolrobogen ${TOL_DEPS})


#### TOL Robogen Gazebo
# Gazebo control plugins
# Find and include Gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})

# Plugin CPP files
file(GLOB_RECURSE
	 TOL_GZ_SRC
	 tol_robogen/gazebo/*.cpp
	 tol_robogen/gazebo/brain/*.c
)

add_library(tolmodelcontrol SHARED ${TOL_GZ_SRC})
target_link_libraries(tolmodelcontrol ${GAZEBO_LIBRARIES} ${TOL_DEPS})

#### Test
# Test executable
add_executable(test tol_robogen/test/test.cpp)
target_link_libraries(test tolrobogen ${TOL_DEPS})