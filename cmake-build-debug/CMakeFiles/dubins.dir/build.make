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
include CMakeFiles/dubins.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/dubins.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dubins.dir/flags.make

CMakeFiles/dubins.dir/src/dubins.cpp.o: CMakeFiles/dubins.dir/flags.make
CMakeFiles/dubins.dir/src/dubins.cpp.o: ../src/dubins.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/osboxes/workspace/project/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/dubins.dir/src/dubins.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dubins.dir/src/dubins.cpp.o -c /home/osboxes/workspace/project/src/dubins.cpp

CMakeFiles/dubins.dir/src/dubins.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dubins.dir/src/dubins.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/osboxes/workspace/project/src/dubins.cpp > CMakeFiles/dubins.dir/src/dubins.cpp.i

CMakeFiles/dubins.dir/src/dubins.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dubins.dir/src/dubins.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/osboxes/workspace/project/src/dubins.cpp -o CMakeFiles/dubins.dir/src/dubins.cpp.s

# Object files for target dubins
dubins_OBJECTS = \
"CMakeFiles/dubins.dir/src/dubins.cpp.o"

# External object files for target dubins
dubins_EXTERNAL_OBJECTS =

libdubins.so: CMakeFiles/dubins.dir/src/dubins.cpp.o
libdubins.so: CMakeFiles/dubins.dir/build.make
libdubins.so: CMakeFiles/dubins.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/osboxes/workspace/project/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libdubins.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dubins.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/dubins.dir/build: libdubins.so

.PHONY : CMakeFiles/dubins.dir/build

CMakeFiles/dubins.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dubins.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dubins.dir/clean

CMakeFiles/dubins.dir/depend:
	cd /home/osboxes/workspace/project/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/osboxes/workspace/project /home/osboxes/workspace/project /home/osboxes/workspace/project/cmake-build-debug /home/osboxes/workspace/project/cmake-build-debug /home/osboxes/workspace/project/cmake-build-debug/CMakeFiles/dubins.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dubins.dir/depend

