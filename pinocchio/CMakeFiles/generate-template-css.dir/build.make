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

# Utility rule file for generate-template-css.

# Include any custom commands dependencies for this target.
include CMakeFiles/generate-template-css.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/generate-template-css.dir/progress.make

CMakeFiles/generate-template-css:
	/usr/bin/doxygen -w html /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/doc/header.html /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/doc/footer.html /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/doc/doxygen.css

generate-template-css: CMakeFiles/generate-template-css
generate-template-css: CMakeFiles/generate-template-css.dir/build.make
.PHONY : generate-template-css

# Rule to build all files generated by this target.
CMakeFiles/generate-template-css.dir/build: generate-template-css
.PHONY : CMakeFiles/generate-template-css.dir/build

CMakeFiles/generate-template-css.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/generate-template-css.dir/cmake_clean.cmake
.PHONY : CMakeFiles/generate-template-css.dir/clean

CMakeFiles/generate-template-css.dir/depend:
	cd /mnt/nvme1n1p7/HKUST_ws/src/pinocchio && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/nvme1n1p7/HKUST_ws/src/pinocchio /mnt/nvme1n1p7/HKUST_ws/src/pinocchio /mnt/nvme1n1p7/HKUST_ws/src/pinocchio /mnt/nvme1n1p7/HKUST_ws/src/pinocchio /mnt/nvme1n1p7/HKUST_ws/src/pinocchio/CMakeFiles/generate-template-css.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/generate-template-css.dir/depend

