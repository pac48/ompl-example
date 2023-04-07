cmake_minimum_required(VERSION 3.24)
project(OMPL)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INSTALL_PREFIX ../install CACHE PATH "Install prefix" FORCE)
set(CMAKE_BUILD_TYPE Debug)
add_subdirectory(ompl)


#find_package(ompl REQUIRED)
#find_library(omplLib libompl2.so install/lib)
find_package(Eigen3 REQUIRED)


add_executable(MyOMPL main.cpp)
target_link_libraries(MyOMPL ompl2)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(install/include/ompl-1.6)


# Print all CMake variables
get_cmake_property(_variableNames VARIABLES)
list(SORT _variableNames)
foreach(_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()
