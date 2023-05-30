# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Produce verbose output by default.
VERBOSE = 1

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/zcx/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/zcx/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /mnt/nvme1n1p7/HKUST_ws/src/pinocchio

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/nvme1n1p7/HKUST_ws/src/pinocchio

# Include any dependencies generated for this target.
include src/CMakeFiles/pinocchio.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/CMakeFiles/pinocchio.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/pinocchio.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/pinocchio.dir/flags.make

src/CMakeFiles/pinocchio.dir/parsers/urdf/model.cpp.o: src/CMakeFiles/pinocchio.dir/flags.make
src/CMakeFiles/pinocchio.dir/parsers/urdf/model.cpp.o: src/parsers/urdf/model.cpp
src/CMakeFiles/pinocchio.dir/parsers/urdf/model.cpp.o: src/CMakeFiles/pinocchio.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/nvme1n1p7/HKUST_ws/src/pinocchio/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/pinocchio.dir/parsers/urdf/model.cpp.o"
	cd /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/pinocchio.dir/parsers/urdf/model.cpp.o -MF CMakeFiles/pinocchio.dir/parsers/urdf/model.cpp.o.d -o CMakeFiles/pinocchio.dir/parsers/urdf/model.cpp.o -c /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src/parsers/urdf/model.cpp

src/CMakeFiles/pinocchio.dir/parsers/urdf/model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pinocchio.dir/parsers/urdf/model.cpp.i"
	cd /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src/parsers/urdf/model.cpp > CMakeFiles/pinocchio.dir/parsers/urdf/model.cpp.i

src/CMakeFiles/pinocchio.dir/parsers/urdf/model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pinocchio.dir/parsers/urdf/model.cpp.s"
	cd /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src/parsers/urdf/model.cpp -o CMakeFiles/pinocchio.dir/parsers/urdf/model.cpp.s

src/CMakeFiles/pinocchio.dir/parsers/urdf/geometry.cpp.o: src/CMakeFiles/pinocchio.dir/flags.make
src/CMakeFiles/pinocchio.dir/parsers/urdf/geometry.cpp.o: src/parsers/urdf/geometry.cpp
src/CMakeFiles/pinocchio.dir/parsers/urdf/geometry.cpp.o: src/CMakeFiles/pinocchio.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/nvme1n1p7/HKUST_ws/src/pinocchio/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/pinocchio.dir/parsers/urdf/geometry.cpp.o"
	cd /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/pinocchio.dir/parsers/urdf/geometry.cpp.o -MF CMakeFiles/pinocchio.dir/parsers/urdf/geometry.cpp.o.d -o CMakeFiles/pinocchio.dir/parsers/urdf/geometry.cpp.o -c /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src/parsers/urdf/geometry.cpp

src/CMakeFiles/pinocchio.dir/parsers/urdf/geometry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pinocchio.dir/parsers/urdf/geometry.cpp.i"
	cd /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src/parsers/urdf/geometry.cpp > CMakeFiles/pinocchio.dir/parsers/urdf/geometry.cpp.i

src/CMakeFiles/pinocchio.dir/parsers/urdf/geometry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pinocchio.dir/parsers/urdf/geometry.cpp.s"
	cd /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src/parsers/urdf/geometry.cpp -o CMakeFiles/pinocchio.dir/parsers/urdf/geometry.cpp.s

src/CMakeFiles/pinocchio.dir/parsers/urdf/utils.cpp.o: src/CMakeFiles/pinocchio.dir/flags.make
src/CMakeFiles/pinocchio.dir/parsers/urdf/utils.cpp.o: src/parsers/urdf/utils.cpp
src/CMakeFiles/pinocchio.dir/parsers/urdf/utils.cpp.o: src/CMakeFiles/pinocchio.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/nvme1n1p7/HKUST_ws/src/pinocchio/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/pinocchio.dir/parsers/urdf/utils.cpp.o"
	cd /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/pinocchio.dir/parsers/urdf/utils.cpp.o -MF CMakeFiles/pinocchio.dir/parsers/urdf/utils.cpp.o.d -o CMakeFiles/pinocchio.dir/parsers/urdf/utils.cpp.o -c /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src/parsers/urdf/utils.cpp

src/CMakeFiles/pinocchio.dir/parsers/urdf/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pinocchio.dir/parsers/urdf/utils.cpp.i"
	cd /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src/parsers/urdf/utils.cpp > CMakeFiles/pinocchio.dir/parsers/urdf/utils.cpp.i

src/CMakeFiles/pinocchio.dir/parsers/urdf/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pinocchio.dir/parsers/urdf/utils.cpp.s"
	cd /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src/parsers/urdf/utils.cpp -o CMakeFiles/pinocchio.dir/parsers/urdf/utils.cpp.s

# Object files for target pinocchio
pinocchio_OBJECTS = \
"CMakeFiles/pinocchio.dir/parsers/urdf/model.cpp.o" \
"CMakeFiles/pinocchio.dir/parsers/urdf/geometry.cpp.o" \
"CMakeFiles/pinocchio.dir/parsers/urdf/utils.cpp.o"

# External object files for target pinocchio
pinocchio_EXTERNAL_OBJECTS =

src/libpinocchio.so.2.5.1: src/CMakeFiles/pinocchio.dir/parsers/urdf/model.cpp.o
src/libpinocchio.so.2.5.1: src/CMakeFiles/pinocchio.dir/parsers/urdf/geometry.cpp.o
src/libpinocchio.so.2.5.1: src/CMakeFiles/pinocchio.dir/parsers/urdf/utils.cpp.o
src/libpinocchio.so.2.5.1: src/CMakeFiles/pinocchio.dir/build.make
src/libpinocchio.so.2.5.1: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
src/libpinocchio.so.2.5.1: /usr/lib/x86_64-linux-gnu/libboost_system.so
src/libpinocchio.so.2.5.1: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
src/libpinocchio.so.2.5.1: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
src/libpinocchio.so.2.5.1: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
src/libpinocchio.so.2.5.1: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
src/libpinocchio.so.2.5.1: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
src/libpinocchio.so.2.5.1: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
src/libpinocchio.so.2.5.1: /mnt/nvme1n1p7/HKUST_ws/devel/lib/libhpp-fcl.so
src/libpinocchio.so.2.5.1: /opt/ros/noetic/lib/liboctomap.so
src/libpinocchio.so.2.5.1: /opt/ros/noetic/lib/liboctomath.so
src/libpinocchio.so.2.5.1: src/CMakeFiles/pinocchio.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/nvme1n1p7/HKUST_ws/src/pinocchio/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library libpinocchio.so"
	cd /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pinocchio.dir/link.txt --verbose=$(VERBOSE)
	cd /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src && $(CMAKE_COMMAND) -E cmake_symlink_library libpinocchio.so.2.5.1 libpinocchio.so.2.5.1 libpinocchio.so

src/libpinocchio.so: src/libpinocchio.so.2.5.1
	@$(CMAKE_COMMAND) -E touch_nocreate src/libpinocchio.so

# Rule to build all files generated by this target.
src/CMakeFiles/pinocchio.dir/build: src/libpinocchio.so
.PHONY : src/CMakeFiles/pinocchio.dir/build

src/CMakeFiles/pinocchio.dir/clean:
	cd /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src && $(CMAKE_COMMAND) -P CMakeFiles/pinocchio.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/pinocchio.dir/clean

src/CMakeFiles/pinocchio.dir/depend:
	cd /mnt/nvme1n1p7/HKUST_ws/src/pinocchio && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/nvme1n1p7/HKUST_ws/src/pinocchio /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src /mnt/nvme1n1p7/HKUST_ws/src/pinocchio /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/src/CMakeFiles/pinocchio.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/pinocchio.dir/depend
