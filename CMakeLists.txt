# Copyright Matt Overby 2021.
# Distributed under the MIT License.
cmake_minimum_required(VERSION 3.5)

project(cloth_sim_tutorial LANGUAGES CXX)

option(CLOTH_DEBUG "run with debug options" OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
add_definitions(-DCLOTH_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# Change compile flags for debug
if (CLOTH_DEBUG)
	add_definitions(-DCLOTH_DEBUG)
	set(CMAKE_BUILD_TYPE Debug)
	if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wundef -Wextra -Wno-long-long -Wlogical-op -Wshadow -Wmaybe-uninitialized")
	endif()
else()
	set(CMAKE_BUILD_TYPE Release)
endif()

# Libigl also includes Eigen, etc
include(libigl)
igl_include(glfw)

# Gather src
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
file(GLOB SRC
	${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# Create binary
add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} PUBLIC igl::glfw)

# Unit testing (work in progress)
enable_testing()
add_executable(cloth_test ${CMAKE_CURRENT_SOURCE_DIR}/test/test.cpp)
target_link_libraries(cloth_test PUBLIC igl::glfw)
add_test(NAME TestClothSolver COMMAND cloth_test)
