# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /home/osboxes/Desktop/clion-2019.2.5/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/osboxes/Desktop/clion-2019.2.5/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/osboxes/workspace/project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/osboxes/workspace/project/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/student.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/student.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/student.dir/flags.make

CMakeFiles/student.dir/src/student_interface.cpp.o: CMakeFiles/student.dir/flags.make
CMakeFiles/student.dir/src/student_interface.cpp.o: ../src/student_interface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/osboxes/workspace/project/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/student.dir/src/student_interface.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/student.dir/src/student_interface.cpp.o -c /home/osboxes/workspace/project/src/student_interface.cpp

CMakeFiles/student.dir/src/student_interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/student.dir/src/student_interface.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/osboxes/workspace/project/src/student_interface.cpp > CMakeFiles/student.dir/src/student_interface.cpp.i

CMakeFiles/student.dir/src/student_interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/student.dir/src/student_interface.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/osboxes/workspace/project/src/student_interface.cpp -o CMakeFiles/student.dir/src/student_interface.cpp.s

CMakeFiles/student.dir/src/find_robot.cpp.o: CMakeFiles/student.dir/flags.make
CMakeFiles/student.dir/src/find_robot.cpp.o: ../src/find_robot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/osboxes/workspace/project/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/student.dir/src/find_robot.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/student.dir/src/find_robot.cpp.o -c /home/osboxes/workspace/project/src/find_robot.cpp

CMakeFiles/student.dir/src/find_robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/student.dir/src/find_robot.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/osboxes/workspace/project/src/find_robot.cpp > CMakeFiles/student.dir/src/find_robot.cpp.i

CMakeFiles/student.dir/src/find_robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/student.dir/src/find_robot.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/osboxes/workspace/project/src/find_robot.cpp -o CMakeFiles/student.dir/src/find_robot.cpp.s

CMakeFiles/student.dir/src/process_arena.cpp.o: CMakeFiles/student.dir/flags.make
CMakeFiles/student.dir/src/process_arena.cpp.o: ../src/process_arena.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/osboxes/workspace/project/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/student.dir/src/process_arena.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/student.dir/src/process_arena.cpp.o -c /home/osboxes/workspace/project/src/process_arena.cpp

CMakeFiles/student.dir/src/process_arena.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/student.dir/src/process_arena.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/osboxes/workspace/project/src/process_arena.cpp > CMakeFiles/student.dir/src/process_arena.cpp.i

CMakeFiles/student.dir/src/process_arena.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/student.dir/src/process_arena.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/osboxes/workspace/project/src/process_arena.cpp -o CMakeFiles/student.dir/src/process_arena.cpp.s

CMakeFiles/student.dir/src/find_victim.cpp.o: CMakeFiles/student.dir/flags.make
CMakeFiles/student.dir/src/find_victim.cpp.o: ../src/find_victim.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/osboxes/workspace/project/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/student.dir/src/find_victim.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/student.dir/src/find_victim.cpp.o -c /home/osboxes/workspace/project/src/find_victim.cpp

CMakeFiles/student.dir/src/find_victim.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/student.dir/src/find_victim.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/osboxes/workspace/project/src/find_victim.cpp > CMakeFiles/student.dir/src/find_victim.cpp.i

CMakeFiles/student.dir/src/find_victim.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/student.dir/src/find_victim.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/osboxes/workspace/project/src/find_victim.cpp -o CMakeFiles/student.dir/src/find_victim.cpp.s

CMakeFiles/student.dir/src/dubins.cpp.o: CMakeFiles/student.dir/flags.make
CMakeFiles/student.dir/src/dubins.cpp.o: ../src/dubins.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/osboxes/workspace/project/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/student.dir/src/dubins.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/student.dir/src/dubins.cpp.o -c /home/osboxes/workspace/project/src/dubins.cpp

CMakeFiles/student.dir/src/dubins.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/student.dir/src/dubins.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/osboxes/workspace/project/src/dubins.cpp > CMakeFiles/student.dir/src/dubins.cpp.i

CMakeFiles/student.dir/src/dubins.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/student.dir/src/dubins.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/osboxes/workspace/project/src/dubins.cpp -o CMakeFiles/student.dir/src/dubins.cpp.s

CMakeFiles/student.dir/src/loadImage.cpp.o: CMakeFiles/student.dir/flags.make
CMakeFiles/student.dir/src/loadImage.cpp.o: ../src/loadImage.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/osboxes/workspace/project/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/student.dir/src/loadImage.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/student.dir/src/loadImage.cpp.o -c /home/osboxes/workspace/project/src/loadImage.cpp

CMakeFiles/student.dir/src/loadImage.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/student.dir/src/loadImage.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/osboxes/workspace/project/src/loadImage.cpp > CMakeFiles/student.dir/src/loadImage.cpp.i

CMakeFiles/student.dir/src/loadImage.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/student.dir/src/loadImage.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/osboxes/workspace/project/src/loadImage.cpp -o CMakeFiles/student.dir/src/loadImage.cpp.s

CMakeFiles/student.dir/src/find_collision.cpp.o: CMakeFiles/student.dir/flags.make
CMakeFiles/student.dir/src/find_collision.cpp.o: ../src/find_collision.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/osboxes/workspace/project/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/student.dir/src/find_collision.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/student.dir/src/find_collision.cpp.o -c /home/osboxes/workspace/project/src/find_collision.cpp

CMakeFiles/student.dir/src/find_collision.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/student.dir/src/find_collision.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/osboxes/workspace/project/src/find_collision.cpp > CMakeFiles/student.dir/src/find_collision.cpp.i

CMakeFiles/student.dir/src/find_collision.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/student.dir/src/find_collision.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/osboxes/workspace/project/src/find_collision.cpp -o CMakeFiles/student.dir/src/find_collision.cpp.s

CMakeFiles/student.dir/src/Voronoi.cpp.o: CMakeFiles/student.dir/flags.make
CMakeFiles/student.dir/src/Voronoi.cpp.o: ../src/Voronoi.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/osboxes/workspace/project/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/student.dir/src/Voronoi.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/student.dir/src/Voronoi.cpp.o -c /home/osboxes/workspace/project/src/Voronoi.cpp

CMakeFiles/student.dir/src/Voronoi.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/student.dir/src/Voronoi.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/osboxes/workspace/project/src/Voronoi.cpp > CMakeFiles/student.dir/src/Voronoi.cpp.i

CMakeFiles/student.dir/src/Voronoi.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/student.dir/src/Voronoi.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/osboxes/workspace/project/src/Voronoi.cpp -o CMakeFiles/student.dir/src/Voronoi.cpp.s

# Object files for target student
student_OBJECTS = \
"CMakeFiles/student.dir/src/student_interface.cpp.o" \
"CMakeFiles/student.dir/src/find_robot.cpp.o" \
"CMakeFiles/student.dir/src/process_arena.cpp.o" \
"CMakeFiles/student.dir/src/find_victim.cpp.o" \
"CMakeFiles/student.dir/src/dubins.cpp.o" \
"CMakeFiles/student.dir/src/loadImage.cpp.o" \
"CMakeFiles/student.dir/src/find_collision.cpp.o" \
"CMakeFiles/student.dir/src/Voronoi.cpp.o"

# External object files for target student
student_EXTERNAL_OBJECTS =

libstudent.so: CMakeFiles/student.dir/src/student_interface.cpp.o
libstudent.so: CMakeFiles/student.dir/src/find_robot.cpp.o
libstudent.so: CMakeFiles/student.dir/src/process_arena.cpp.o
libstudent.so: CMakeFiles/student.dir/src/find_victim.cpp.o
libstudent.so: CMakeFiles/student.dir/src/dubins.cpp.o
libstudent.so: CMakeFiles/student.dir/src/loadImage.cpp.o
libstudent.so: CMakeFiles/student.dir/src/find_collision.cpp.o
libstudent.so: CMakeFiles/student.dir/src/Voronoi.cpp.o
libstudent.so: CMakeFiles/student.dir/build.make
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
libstudent.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
libstudent.so: CMakeFiles/student.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/osboxes/workspace/project/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX shared library libstudent.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/student.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/student.dir/build: libstudent.so

.PHONY : CMakeFiles/student.dir/build

CMakeFiles/student.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/student.dir/cmake_clean.cmake
.PHONY : CMakeFiles/student.dir/clean

CMakeFiles/student.dir/depend:
	cd /home/osboxes/workspace/project/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/osboxes/workspace/project /home/osboxes/workspace/project /home/osboxes/workspace/project/cmake-build-debug /home/osboxes/workspace/project/cmake-build-debug /home/osboxes/workspace/project/cmake-build-debug/CMakeFiles/student.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/student.dir/depend
