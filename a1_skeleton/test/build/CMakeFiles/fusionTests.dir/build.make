# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/build

# Include any dependencies generated for this target.
include CMakeFiles/fusionTests.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/fusionTests.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/fusionTests.dir/flags.make

CMakeFiles/fusionTests.dir/test_fusion.cpp.o: CMakeFiles/fusionTests.dir/flags.make
CMakeFiles/fusionTests.dir/test_fusion.cpp.o: ../test_fusion.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/fusionTests.dir/test_fusion.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fusionTests.dir/test_fusion.cpp.o -c /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/test_fusion.cpp

CMakeFiles/fusionTests.dir/test_fusion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fusionTests.dir/test_fusion.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/test_fusion.cpp > CMakeFiles/fusionTests.dir/test_fusion.cpp.i

CMakeFiles/fusionTests.dir/test_fusion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fusionTests.dir/test_fusion.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/test_fusion.cpp -o CMakeFiles/fusionTests.dir/test_fusion.cpp.s

# Object files for target fusionTests
fusionTests_OBJECTS = \
"CMakeFiles/fusionTests.dir/test_fusion.cpp.o"

# External object files for target fusionTests
fusionTests_EXTERNAL_OBJECTS =

fusionTests: CMakeFiles/fusionTests.dir/test_fusion.cpp.o
fusionTests: CMakeFiles/fusionTests.dir/build.make
fusionTests: /usr/src/gtest/lib/libgtest.a
fusionTests: libmock_lib.a
fusionTests: CMakeFiles/fusionTests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable fusionTests"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fusionTests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/fusionTests.dir/build: fusionTests

.PHONY : CMakeFiles/fusionTests.dir/build

CMakeFiles/fusionTests.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/fusionTests.dir/cmake_clean.cmake
.PHONY : CMakeFiles/fusionTests.dir/clean

CMakeFiles/fusionTests.dir/depend:
	cd /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/build /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/build /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/build/CMakeFiles/fusionTests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/fusionTests.dir/depend

