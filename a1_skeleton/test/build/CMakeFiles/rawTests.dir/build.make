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
include CMakeFiles/rawTests.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rawTests.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rawTests.dir/flags.make

CMakeFiles/rawTests.dir/test_rawdata.cpp.o: CMakeFiles/rawTests.dir/flags.make
CMakeFiles/rawTests.dir/test_rawdata.cpp.o: ../test_rawdata.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rawTests.dir/test_rawdata.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rawTests.dir/test_rawdata.cpp.o -c /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/test_rawdata.cpp

CMakeFiles/rawTests.dir/test_rawdata.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rawTests.dir/test_rawdata.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/test_rawdata.cpp > CMakeFiles/rawTests.dir/test_rawdata.cpp.i

CMakeFiles/rawTests.dir/test_rawdata.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rawTests.dir/test_rawdata.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/test_rawdata.cpp -o CMakeFiles/rawTests.dir/test_rawdata.cpp.s

# Object files for target rawTests
rawTests_OBJECTS = \
"CMakeFiles/rawTests.dir/test_rawdata.cpp.o"

# External object files for target rawTests
rawTests_EXTERNAL_OBJECTS =

rawTests: CMakeFiles/rawTests.dir/test_rawdata.cpp.o
rawTests: CMakeFiles/rawTests.dir/build.make
rawTests: /usr/src/gtest/lib/libgtest.a
rawTests: libstudent_lib.a
rawTests: CMakeFiles/rawTests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable rawTests"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rawTests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rawTests.dir/build: rawTests

.PHONY : CMakeFiles/rawTests.dir/build

CMakeFiles/rawTests.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rawTests.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rawTests.dir/clean

CMakeFiles/rawTests.dir/depend:
	cd /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/build /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/build /mnt/e/Projects/CPlusPlusProject/Job1/a1_skeleton/test/build/CMakeFiles/rawTests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rawTests.dir/depend

