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
CMAKE_SOURCE_DIR = /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/build

# Include any dependencies generated for this target.
include test/CMakeFiles/posetest.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/posetest.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/posetest.dir/flags.make

test/CMakeFiles/posetest.dir/posetest.cpp.o: test/CMakeFiles/posetest.dir/flags.make
test/CMakeFiles/posetest.dir/posetest.cpp.o: ../test/posetest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/posetest.dir/posetest.cpp.o"
	cd /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/posetest.dir/posetest.cpp.o -c /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/test/posetest.cpp

test/CMakeFiles/posetest.dir/posetest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/posetest.dir/posetest.cpp.i"
	cd /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/test/posetest.cpp > CMakeFiles/posetest.dir/posetest.cpp.i

test/CMakeFiles/posetest.dir/posetest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/posetest.dir/posetest.cpp.s"
	cd /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/test/posetest.cpp -o CMakeFiles/posetest.dir/posetest.cpp.s

# Object files for target posetest
posetest_OBJECTS = \
"CMakeFiles/posetest.dir/posetest.cpp.o"

# External object files for target posetest
posetest_EXTERNAL_OBJECTS =

test/posetest: test/CMakeFiles/posetest.dir/posetest.cpp.o
test/posetest: test/CMakeFiles/posetest.dir/build.make
test/posetest: /usr/src/gtest/lib/libgtest.a
test/posetest: /usr/src/gtest/lib/libgtest_main.a
test/posetest: libtf2.a
test/posetest: libanalysis.a
test/posetest: libcontrol_aircraft.a
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
test/posetest: /usr/src/gtest/lib/libgtest.a
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
test/posetest: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
test/posetest: test/CMakeFiles/posetest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable posetest"
	cd /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/posetest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/posetest.dir/build: test/posetest

.PHONY : test/CMakeFiles/posetest.dir/build

test/CMakeFiles/posetest.dir/clean:
	cd /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/build/test && $(CMAKE_COMMAND) -P CMakeFiles/posetest.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/posetest.dir/clean

test/CMakeFiles/posetest.dir/depend:
	cd /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/test /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/build /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/build/test /home/glacio/Desktop/CarAndRadarSmallProject/airCraft/a2_aircraft/a2_skeleton/build/test/CMakeFiles/posetest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/posetest.dir/depend

