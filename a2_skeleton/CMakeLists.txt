# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Add project name
project (Assignment2) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# CMake will now create the library, add the files needed
add_library(ranger_lib ranger.cpp rangerinterface.cpp laser.cpp sonar.cpp rangerfusion.cpp rangerfusioninterface.cpp)
add_library(cell cell.cpp)
# Cmake files will create executable add the files needed
add_executable(assignment2 main.cpp)

target_link_libraries(assignment2 ranger_lib cell)


