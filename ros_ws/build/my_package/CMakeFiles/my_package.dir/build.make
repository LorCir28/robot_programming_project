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
CMAKE_SOURCE_DIR = /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build

# Include any dependencies generated for this target.
include my_package/CMakeFiles/my_package.dir/depend.make

# Include the progress variables for this target.
include my_package/CMakeFiles/my_package.dir/progress.make

# Include the compile flags for this target's objects.
include my_package/CMakeFiles/my_package.dir/flags.make

my_package/CMakeFiles/my_package.dir/src/robot.cpp.o: my_package/CMakeFiles/my_package.dir/flags.make
my_package/CMakeFiles/my_package.dir/src/robot.cpp.o: /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package/src/robot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object my_package/CMakeFiles/my_package.dir/src/robot.cpp.o"
	cd /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/my_package && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my_package.dir/src/robot.cpp.o -c /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package/src/robot.cpp

my_package/CMakeFiles/my_package.dir/src/robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_package.dir/src/robot.cpp.i"
	cd /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/my_package && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package/src/robot.cpp > CMakeFiles/my_package.dir/src/robot.cpp.i

my_package/CMakeFiles/my_package.dir/src/robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_package.dir/src/robot.cpp.s"
	cd /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/my_package && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package/src/robot.cpp -o CMakeFiles/my_package.dir/src/robot.cpp.s

my_package/CMakeFiles/my_package.dir/src/world.cpp.o: my_package/CMakeFiles/my_package.dir/flags.make
my_package/CMakeFiles/my_package.dir/src/world.cpp.o: /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package/src/world.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object my_package/CMakeFiles/my_package.dir/src/world.cpp.o"
	cd /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/my_package && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my_package.dir/src/world.cpp.o -c /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package/src/world.cpp

my_package/CMakeFiles/my_package.dir/src/world.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_package.dir/src/world.cpp.i"
	cd /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/my_package && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package/src/world.cpp > CMakeFiles/my_package.dir/src/world.cpp.i

my_package/CMakeFiles/my_package.dir/src/world.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_package.dir/src/world.cpp.s"
	cd /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/my_package && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package/src/world.cpp -o CMakeFiles/my_package.dir/src/world.cpp.s

my_package/CMakeFiles/my_package.dir/src/lidar.cpp.o: my_package/CMakeFiles/my_package.dir/flags.make
my_package/CMakeFiles/my_package.dir/src/lidar.cpp.o: /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package/src/lidar.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object my_package/CMakeFiles/my_package.dir/src/lidar.cpp.o"
	cd /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/my_package && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my_package.dir/src/lidar.cpp.o -c /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package/src/lidar.cpp

my_package/CMakeFiles/my_package.dir/src/lidar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_package.dir/src/lidar.cpp.i"
	cd /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/my_package && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package/src/lidar.cpp > CMakeFiles/my_package.dir/src/lidar.cpp.i

my_package/CMakeFiles/my_package.dir/src/lidar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_package.dir/src/lidar.cpp.s"
	cd /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/my_package && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package/src/lidar.cpp -o CMakeFiles/my_package.dir/src/lidar.cpp.s

# Object files for target my_package
my_package_OBJECTS = \
"CMakeFiles/my_package.dir/src/robot.cpp.o" \
"CMakeFiles/my_package.dir/src/world.cpp.o" \
"CMakeFiles/my_package.dir/src/lidar.cpp.o"

# External object files for target my_package
my_package_EXTERNAL_OBJECTS =

/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: my_package/CMakeFiles/my_package.dir/src/robot.cpp.o
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: my_package/CMakeFiles/my_package.dir/src/world.cpp.o
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: my_package/CMakeFiles/my_package.dir/src/lidar.cpp.o
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: my_package/CMakeFiles/my_package.dir/build.make
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so: my_package/CMakeFiles/my_package.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so"
	cd /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/my_package && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my_package.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
my_package/CMakeFiles/my_package.dir/build: /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/devel/lib/libmy_package.so

.PHONY : my_package/CMakeFiles/my_package.dir/build

my_package/CMakeFiles/my_package.dir/clean:
	cd /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/my_package && $(CMAKE_COMMAND) -P CMakeFiles/my_package.dir/cmake_clean.cmake
.PHONY : my_package/CMakeFiles/my_package.dir/clean

my_package/CMakeFiles/my_package.dir/depend:
	cd /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/src/my_package /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/my_package /home/lattinone/Desktop/Lorenzo/rp/rp_project/ros_ws/build/my_package/CMakeFiles/my_package.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : my_package/CMakeFiles/my_package.dir/depend

