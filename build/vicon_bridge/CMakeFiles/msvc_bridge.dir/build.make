# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/isaac/uav_avoidance/src/vicon_bridge

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/isaac/uav_avoidance/build/vicon_bridge

# Include any dependencies generated for this target.
include CMakeFiles/msvc_bridge.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/msvc_bridge.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/msvc_bridge.dir/flags.make

CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o: CMakeFiles/msvc_bridge.dir/flags.make
CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o: /home/isaac/uav_avoidance/src/vicon_bridge/src/msvc_bridge.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/isaac/uav_avoidance/build/vicon_bridge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o -c /home/isaac/uav_avoidance/src/vicon_bridge/src/msvc_bridge.cpp

CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/isaac/uav_avoidance/src/vicon_bridge/src/msvc_bridge.cpp > CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.i

CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/isaac/uav_avoidance/src/vicon_bridge/src/msvc_bridge.cpp -o CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.s

CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o.requires:

.PHONY : CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o.requires

CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o.provides: CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o.requires
	$(MAKE) -f CMakeFiles/msvc_bridge.dir/build.make CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o.provides.build
.PHONY : CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o.provides

CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o.provides.build: CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o


# Object files for target msvc_bridge
msvc_bridge_OBJECTS = \
"CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o"

# External object files for target msvc_bridge
msvc_bridge_EXTERNAL_OBJECTS =

/home/isaac/uav_avoidance/devel/.private/vicon_bridge/lib/libmsvc_bridge.so: CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o
/home/isaac/uav_avoidance/devel/.private/vicon_bridge/lib/libmsvc_bridge.so: CMakeFiles/msvc_bridge.dir/build.make
/home/isaac/uav_avoidance/devel/.private/vicon_bridge/lib/libmsvc_bridge.so: CMakeFiles/msvc_bridge.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/isaac/uav_avoidance/build/vicon_bridge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/isaac/uav_avoidance/devel/.private/vicon_bridge/lib/libmsvc_bridge.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/msvc_bridge.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/msvc_bridge.dir/build: /home/isaac/uav_avoidance/devel/.private/vicon_bridge/lib/libmsvc_bridge.so

.PHONY : CMakeFiles/msvc_bridge.dir/build

CMakeFiles/msvc_bridge.dir/requires: CMakeFiles/msvc_bridge.dir/src/msvc_bridge.cpp.o.requires

.PHONY : CMakeFiles/msvc_bridge.dir/requires

CMakeFiles/msvc_bridge.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/msvc_bridge.dir/cmake_clean.cmake
.PHONY : CMakeFiles/msvc_bridge.dir/clean

CMakeFiles/msvc_bridge.dir/depend:
	cd /home/isaac/uav_avoidance/build/vicon_bridge && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/isaac/uav_avoidance/src/vicon_bridge /home/isaac/uav_avoidance/src/vicon_bridge /home/isaac/uav_avoidance/build/vicon_bridge /home/isaac/uav_avoidance/build/vicon_bridge /home/isaac/uav_avoidance/build/vicon_bridge/CMakeFiles/msvc_bridge.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/msvc_bridge.dir/depend

