# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_SOURCE_DIR = /home/pi/arucoTest

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/arucoTest

# Include any dependencies generated for this target.
include CMakeFiles/arucoTest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/arucoTest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/arucoTest.dir/flags.make

CMakeFiles/arucoTest.dir/arucoDetect.cpp.o: CMakeFiles/arucoTest.dir/flags.make
CMakeFiles/arucoTest.dir/arucoDetect.cpp.o: arucoDetect.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/arucoTest/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/arucoTest.dir/arucoDetect.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/arucoTest.dir/arucoDetect.cpp.o -c /home/pi/arucoTest/arucoDetect.cpp

CMakeFiles/arucoTest.dir/arucoDetect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/arucoTest.dir/arucoDetect.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/arucoTest/arucoDetect.cpp > CMakeFiles/arucoTest.dir/arucoDetect.cpp.i

CMakeFiles/arucoTest.dir/arucoDetect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/arucoTest.dir/arucoDetect.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/arucoTest/arucoDetect.cpp -o CMakeFiles/arucoTest.dir/arucoDetect.cpp.s

CMakeFiles/arucoTest.dir/arucoDetect.cpp.o.requires:

.PHONY : CMakeFiles/arucoTest.dir/arucoDetect.cpp.o.requires

CMakeFiles/arucoTest.dir/arucoDetect.cpp.o.provides: CMakeFiles/arucoTest.dir/arucoDetect.cpp.o.requires
	$(MAKE) -f CMakeFiles/arucoTest.dir/build.make CMakeFiles/arucoTest.dir/arucoDetect.cpp.o.provides.build
.PHONY : CMakeFiles/arucoTest.dir/arucoDetect.cpp.o.provides

CMakeFiles/arucoTest.dir/arucoDetect.cpp.o.provides.build: CMakeFiles/arucoTest.dir/arucoDetect.cpp.o


# Object files for target arucoTest
arucoTest_OBJECTS = \
"CMakeFiles/arucoTest.dir/arucoDetect.cpp.o"

# External object files for target arucoTest
arucoTest_EXTERNAL_OBJECTS =

arucoTest: CMakeFiles/arucoTest.dir/arucoDetect.cpp.o
arucoTest: CMakeFiles/arucoTest.dir/build.make
arucoTest: /usr/local/lib/libopencv_stitching.so.3.4.0
arucoTest: /usr/local/lib/libopencv_superres.so.3.4.0
arucoTest: /usr/local/lib/libopencv_videostab.so.3.4.0
arucoTest: /usr/local/lib/libopencv_aruco.so.3.4.0
arucoTest: /usr/local/lib/libopencv_bgsegm.so.3.4.0
arucoTest: /usr/local/lib/libopencv_bioinspired.so.3.4.0
arucoTest: /usr/local/lib/libopencv_ccalib.so.3.4.0
arucoTest: /usr/local/lib/libopencv_dpm.so.3.4.0
arucoTest: /usr/local/lib/libopencv_face.so.3.4.0
arucoTest: /usr/local/lib/libopencv_freetype.so.3.4.0
arucoTest: /usr/local/lib/libopencv_fuzzy.so.3.4.0
arucoTest: /usr/local/lib/libopencv_img_hash.so.3.4.0
arucoTest: /usr/local/lib/libopencv_line_descriptor.so.3.4.0
arucoTest: /usr/local/lib/libopencv_optflow.so.3.4.0
arucoTest: /usr/local/lib/libopencv_reg.so.3.4.0
arucoTest: /usr/local/lib/libopencv_rgbd.so.3.4.0
arucoTest: /usr/local/lib/libopencv_saliency.so.3.4.0
arucoTest: /usr/local/lib/libopencv_stereo.so.3.4.0
arucoTest: /usr/local/lib/libopencv_structured_light.so.3.4.0
arucoTest: /usr/local/lib/libopencv_surface_matching.so.3.4.0
arucoTest: /usr/local/lib/libopencv_tracking.so.3.4.0
arucoTest: /usr/local/lib/libopencv_xfeatures2d.so.3.4.0
arucoTest: /usr/local/lib/libopencv_ximgproc.so.3.4.0
arucoTest: /usr/local/lib/libopencv_xobjdetect.so.3.4.0
arucoTest: /usr/local/lib/libopencv_xphoto.so.3.4.0
arucoTest: /usr/local/lib/libopencv_shape.so.3.4.0
arucoTest: /usr/local/lib/libopencv_photo.so.3.4.0
arucoTest: /usr/local/lib/libopencv_datasets.so.3.4.0
arucoTest: /usr/local/lib/libopencv_plot.so.3.4.0
arucoTest: /usr/local/lib/libopencv_text.so.3.4.0
arucoTest: /usr/local/lib/libopencv_dnn.so.3.4.0
arucoTest: /usr/local/lib/libopencv_ml.so.3.4.0
arucoTest: /usr/local/lib/libopencv_video.so.3.4.0
arucoTest: /usr/local/lib/libopencv_calib3d.so.3.4.0
arucoTest: /usr/local/lib/libopencv_features2d.so.3.4.0
arucoTest: /usr/local/lib/libopencv_highgui.so.3.4.0
arucoTest: /usr/local/lib/libopencv_videoio.so.3.4.0
arucoTest: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.0
arucoTest: /usr/local/lib/libopencv_flann.so.3.4.0
arucoTest: /usr/local/lib/libopencv_imgcodecs.so.3.4.0
arucoTest: /usr/local/lib/libopencv_objdetect.so.3.4.0
arucoTest: /usr/local/lib/libopencv_imgproc.so.3.4.0
arucoTest: /usr/local/lib/libopencv_core.so.3.4.0
arucoTest: CMakeFiles/arucoTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/arucoTest/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable arucoTest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/arucoTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/arucoTest.dir/build: arucoTest

.PHONY : CMakeFiles/arucoTest.dir/build

CMakeFiles/arucoTest.dir/requires: CMakeFiles/arucoTest.dir/arucoDetect.cpp.o.requires

.PHONY : CMakeFiles/arucoTest.dir/requires

CMakeFiles/arucoTest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/arucoTest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/arucoTest.dir/clean

CMakeFiles/arucoTest.dir/depend:
	cd /home/pi/arucoTest && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/arucoTest /home/pi/arucoTest /home/pi/arucoTest /home/pi/arucoTest /home/pi/arucoTest/CMakeFiles/arucoTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/arucoTest.dir/depend
