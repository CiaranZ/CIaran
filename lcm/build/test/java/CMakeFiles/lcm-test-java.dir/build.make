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
CMAKE_SOURCE_DIR = /home/zcx/catkin_ws/src/lcm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zcx/catkin_ws/src/lcm/build

# Utility rule file for lcm-test-java.

# Include the progress variables for this target.
include test/java/CMakeFiles/lcm-test-java.dir/progress.make

test/java/CMakeFiles/lcm-test-java: test/java/lcm-test.jar


test/java/lcm-test.jar: ../test/java/hamcrest-core-1.3.jar
test/java/lcm-test.jar: ../test/java/junit-4.11.jar
test/java/lcm-test.jar: test/java/CMakeFiles/lcm-test-java.dir/java_class_filelist
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/zcx/catkin_ws/src/lcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Creating Java archive lcm-test.jar"
	cd /home/zcx/catkin_ws/src/lcm/build/test/java/CMakeFiles/lcm-test-java.dir && /usr/bin/jar -cf /home/zcx/catkin_ws/src/lcm/build/test/java/lcm-test.jar @java_class_filelist
	cd /home/zcx/catkin_ws/src/lcm/build/test/java/CMakeFiles/lcm-test-java.dir && /usr/bin/cmake -D_JAVA_TARGET_DIR=/home/zcx/catkin_ws/src/lcm/build/test/java -D_JAVA_TARGET_OUTPUT_NAME=lcm-test.jar -D_JAVA_TARGET_OUTPUT_LINK= -P /usr/share/cmake-3.16/Modules/UseJavaSymlinks.cmake

test/java/CMakeFiles/lcm-test-java.dir/java_class_filelist: test/java/CMakeFiles/lcm-test-java.dir/java_compiled_lcm-test-java
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/zcx/catkin_ws/src/lcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating CMakeFiles/lcm-test-java.dir/java_class_filelist"
	cd /home/zcx/catkin_ws/src/lcm/test/java && /usr/bin/cmake -DCMAKE_JAVA_CLASS_OUTPUT_PATH=/home/zcx/catkin_ws/src/lcm/build/test/java/CMakeFiles/lcm-test-java.dir -DCMAKE_JAR_CLASSES_PREFIX="" -P /usr/share/cmake-3.16/Modules/UseJavaClassFilelist.cmake

test/java/CMakeFiles/lcm-test-java.dir/java_compiled_lcm-test-java: ../test/java/lcmtest/LcmTestClient.java
test/java/CMakeFiles/lcm-test-java.dir/java_compiled_lcm-test-java: ../test/java/lcmtest/TestUDPMulticastProvider.java
test/java/CMakeFiles/lcm-test-java.dir/java_compiled_lcm-test-java: test/types/lcm-test-types.jar
test/java/CMakeFiles/lcm-test-java.dir/java_compiled_lcm-test-java: lcm-java/lcm.jar
test/java/CMakeFiles/lcm-test-java.dir/java_compiled_lcm-test-java: ../test/java/hamcrest-core-1.3.jar
test/java/CMakeFiles/lcm-test-java.dir/java_compiled_lcm-test-java: ../test/java/junit-4.11.jar
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/zcx/catkin_ws/src/lcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building Java objects for lcm-test-java.jar"
	cd /home/zcx/catkin_ws/src/lcm/test/java && /usr/bin/javac -classpath :/home/zcx/catkin_ws/src/lcm/test/java:/home/zcx/catkin_ws/src/lcm/build/test/java:/home/zcx/catkin_ws/src/lcm/build/test/types/lcm-test-types.jar:/home/zcx/catkin_ws/src/lcm/build/lcm-java/lcm.jar:/home/zcx/catkin_ws/src/lcm/test/java/hamcrest-core-1.3.jar:/home/zcx/catkin_ws/src/lcm/test/java/junit-4.11.jar -d /home/zcx/catkin_ws/src/lcm/build/test/java/CMakeFiles/lcm-test-java.dir @/home/zcx/catkin_ws/src/lcm/build/test/java/CMakeFiles/lcm-test-java.dir/java_sources
	cd /home/zcx/catkin_ws/src/lcm/test/java && /usr/bin/cmake -E touch /home/zcx/catkin_ws/src/lcm/build/test/java/CMakeFiles/lcm-test-java.dir/java_compiled_lcm-test-java

lcm-test-java: test/java/CMakeFiles/lcm-test-java
lcm-test-java: test/java/lcm-test.jar
lcm-test-java: test/java/CMakeFiles/lcm-test-java.dir/java_class_filelist
lcm-test-java: test/java/CMakeFiles/lcm-test-java.dir/java_compiled_lcm-test-java
lcm-test-java: test/java/CMakeFiles/lcm-test-java.dir/build.make

.PHONY : lcm-test-java

# Rule to build all files generated by this target.
test/java/CMakeFiles/lcm-test-java.dir/build: lcm-test-java

.PHONY : test/java/CMakeFiles/lcm-test-java.dir/build

test/java/CMakeFiles/lcm-test-java.dir/clean:
	cd /home/zcx/catkin_ws/src/lcm/build/test/java && $(CMAKE_COMMAND) -P CMakeFiles/lcm-test-java.dir/cmake_clean.cmake
.PHONY : test/java/CMakeFiles/lcm-test-java.dir/clean

test/java/CMakeFiles/lcm-test-java.dir/depend:
	cd /home/zcx/catkin_ws/src/lcm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zcx/catkin_ws/src/lcm /home/zcx/catkin_ws/src/lcm/test/java /home/zcx/catkin_ws/src/lcm/build /home/zcx/catkin_ws/src/lcm/build/test/java /home/zcx/catkin_ws/src/lcm/build/test/java/CMakeFiles/lcm-test-java.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/java/CMakeFiles/lcm-test-java.dir/depend

