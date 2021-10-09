# The minimum version of CMake Required 
cmake_minimum_required(VERSION 3.0)

# Change project name
project(a1_skeleton) 

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# CMake will create your library, add the cpp files needed
#add_library(ranger )

# CMake will create the cell library that you will need 
add_library(cell cell.cpp)

# CMake will create executable add the cpp files needed
#add_executable(assignment2 )


# Cmake will link against the libraries in final step
#target_link_libraries(assignment2 ranger cell)
# Add project name
# CMake will now create the library, add the files needed
add_library(ranger_lib ranger.cpp rangerinterface.cpp laser.cpp sonar.cpp rangerfusion.cpp rangerfusioninterface.cpp)
# Cmake files will create executable add the files needed

add_executable(assignment2 main.cpp)
target_link_libraries(assignment2 ranger_lib cell)