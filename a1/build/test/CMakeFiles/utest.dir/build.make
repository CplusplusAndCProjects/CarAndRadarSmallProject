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
CMAKE_SOURCE_DIR = /home/glacio/Desktop/quiz3/a1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/glacio/Desktop/quiz3/a1/build

# Include any dependencies generated for this target.
include test/CMakeFiles/utest.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/utest.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/utest.dir/flags.make

test/CMakeFiles/utest.dir/utest.cpp.o: test/CMakeFiles/utest.dir/flags.make
test/CMakeFiles/utest.dir/utest.cpp.o: ../test/utest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/glacio/Desktop/quiz3/a1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/utest.dir/utest.cpp.o"
	cd /home/glacio/Desktop/quiz3/a1/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/utest.dir/utest.cpp.o -c /home/glacio/Desktop/quiz3/a1/test/utest.cpp

test/CMakeFiles/utest.dir/utest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utest.dir/utest.cpp.i"
	cd /home/glacio/Desktop/quiz3/a1/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/glacio/Desktop/quiz3/a1/test/utest.cpp > CMakeFiles/utest.dir/utest.cpp.i

test/CMakeFiles/utest.dir/utest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utest.dir/utest.cpp.s"
	cd /home/glacio/Desktop/quiz3/a1/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/glacio/Desktop/quiz3/a1/test/utest.cpp -o CMakeFiles/utest.dir/utest.cpp.s

# Object files for target utest
utest_OBJECTS = \
"CMakeFiles/utest.dir/utest.cpp.o"

# External object files for target utest
utest_EXTERNAL_OBJECTS =

test/utest: test/CMakeFiles/utest.dir/utest.cpp.o
test/utest: test/CMakeFiles/utest.dir/build.make
test/utest: /usr/src/gtest/lib/libgtest.a
test/utest: /usr/src/gtest/lib/libgtest_main.a
test/utest: libanalysis.a
test/utest: /usr/src/gtest/lib/libgtest.a
test/utest: libdisplayrace.a
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
test/utest: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
test/utest: test/CMakeFiles/utest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/glacio/Desktop/quiz3/a1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable utest"
	cd /home/glacio/Desktop/quiz3/a1/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/utest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/utest.dir/build: test/utest

.PHONY : test/CMakeFiles/utest.dir/build

test/CMakeFiles/utest.dir/clean:
	cd /home/glacio/Desktop/quiz3/a1/build/test && $(CMAKE_COMMAND) -P CMakeFiles/utest.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/utest.dir/clean

test/CMakeFiles/utest.dir/depend:
	cd /home/glacio/Desktop/quiz3/a1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/glacio/Desktop/quiz3/a1 /home/glacio/Desktop/quiz3/a1/test /home/glacio/Desktop/quiz3/a1/build /home/glacio/Desktop/quiz3/a1/build/test /home/glacio/Desktop/quiz3/a1/build/test/CMakeFiles/utest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/utest.dir/depend

