# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/akshay/Documents/zed-getColorAndDepth/depthAndColor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/akshay/Documents/zed-getColorAndDepth/depthAndColor/build

# Include any dependencies generated for this target.
include CMakeFiles/depthAndColor.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/depthAndColor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/depthAndColor.dir/flags.make

CMakeFiles/depthAndColor.dir/main.o: CMakeFiles/depthAndColor.dir/flags.make
CMakeFiles/depthAndColor.dir/main.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/akshay/Documents/zed-getColorAndDepth/depthAndColor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/depthAndColor.dir/main.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/depthAndColor.dir/main.o -c /home/akshay/Documents/zed-getColorAndDepth/depthAndColor/main.cpp

CMakeFiles/depthAndColor.dir/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/depthAndColor.dir/main.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/akshay/Documents/zed-getColorAndDepth/depthAndColor/main.cpp > CMakeFiles/depthAndColor.dir/main.i

CMakeFiles/depthAndColor.dir/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/depthAndColor.dir/main.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/akshay/Documents/zed-getColorAndDepth/depthAndColor/main.cpp -o CMakeFiles/depthAndColor.dir/main.s

CMakeFiles/depthAndColor.dir/main.o.requires:

.PHONY : CMakeFiles/depthAndColor.dir/main.o.requires

CMakeFiles/depthAndColor.dir/main.o.provides: CMakeFiles/depthAndColor.dir/main.o.requires
	$(MAKE) -f CMakeFiles/depthAndColor.dir/build.make CMakeFiles/depthAndColor.dir/main.o.provides.build
.PHONY : CMakeFiles/depthAndColor.dir/main.o.provides

CMakeFiles/depthAndColor.dir/main.o.provides.build: CMakeFiles/depthAndColor.dir/main.o


# Object files for target depthAndColor
depthAndColor_OBJECTS = \
"CMakeFiles/depthAndColor.dir/main.o"

# External object files for target depthAndColor
depthAndColor_EXTERNAL_OBJECTS =

depthAndColor: CMakeFiles/depthAndColor.dir/main.o
depthAndColor: CMakeFiles/depthAndColor.dir/build.make
depthAndColor: /usr/local/zed/lib/libsl_zed.so
depthAndColor: /usr/local/zed/lib/libsl_depthcore.so
depthAndColor: /usr/local/zed/lib/libsl_calibration.so
depthAndColor: /usr/local/zed/lib/libsl_tracking.so
depthAndColor: /usr/local/zed/lib/libsl_disparityFusion.so
depthAndColor: /usr/local/zed/lib/libsl_svorw.so
depthAndColor: /usr/local/zed/lib/libsl_scanning.so
depthAndColor: /usr/local/zed/lib/libsl_core.so
depthAndColor: /usr/lib/x86_64-linux-gnu/libcuda.so
depthAndColor: /usr/local/cuda/lib64/libcudart.so
depthAndColor: /usr/local/cuda/lib64/libnppc.so
depthAndColor: /usr/local/cuda/lib64/libnppi.so
depthAndColor: /usr/local/cuda/lib64/libnpps.so
depthAndColor: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_superres3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_face3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_plot3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_reg3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_text3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_shape3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_video3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_viz3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_phase_unwrapping3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_flann3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_ml3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_photo3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.2.0
depthAndColor: /opt/ros/kinetic/lib/libopencv_core3.so.3.2.0
depthAndColor: CMakeFiles/depthAndColor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/akshay/Documents/zed-getColorAndDepth/depthAndColor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable depthAndColor"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/depthAndColor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/depthAndColor.dir/build: depthAndColor

.PHONY : CMakeFiles/depthAndColor.dir/build

CMakeFiles/depthAndColor.dir/requires: CMakeFiles/depthAndColor.dir/main.o.requires

.PHONY : CMakeFiles/depthAndColor.dir/requires

CMakeFiles/depthAndColor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/depthAndColor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/depthAndColor.dir/clean

CMakeFiles/depthAndColor.dir/depend:
	cd /home/akshay/Documents/zed-getColorAndDepth/depthAndColor/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/akshay/Documents/zed-getColorAndDepth/depthAndColor /home/akshay/Documents/zed-getColorAndDepth/depthAndColor /home/akshay/Documents/zed-getColorAndDepth/depthAndColor/build /home/akshay/Documents/zed-getColorAndDepth/depthAndColor/build /home/akshay/Documents/zed-getColorAndDepth/depthAndColor/build/CMakeFiles/depthAndColor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/depthAndColor.dir/depend
