#######################################################################
# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})


include_directories(
  ${OpenCV_INCLUDE_DIRS}
)
find_package (OpenCV 4.0.0 REQUIRED)
include_directories ("/usr/include/opencv4/")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

set(CAR_LIB_HOME ${CMAKE_SOURCE_DIR}/dep)
include_directories(${CAR_LIB_HOME})
message ( STATUS "Looking for libray in ${CAR_LIB_HOME}" )

if(NOT GTEST_LIBRARY)
   message("GTest library not found")
endif()

enable_testing()
###################################

# Add the unit test executable against the source code of the test
add_executable(utest utest.cpp)
# Linking to gtest and to your libraries
target_link_libraries(utest GTest::GTest GTest::Main analysis car ${OpenCV_LIBS} )
# We can finally add the test to a list of tests to be run with make test
add_test(Q0A2 utest)
