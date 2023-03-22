# CMake generated Testfile for 
# Source directory: /home/zcx/catkin_ws/src/lcm/test/java
# Build directory: /home/zcx/catkin_ws/src/lcm/build/test/java
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(Java::client_server "/usr/bin/python" "/home/zcx/catkin_ws/src/lcm/test/java/../run_client_server_test.py" "/home/zcx/catkin_ws/src/lcm/build/test/c/test-c-server" "/usr/bin/java" "-cp" "/home/zcx/catkin_ws/src/lcm/build/test/java/lcm-test.jar:/home/zcx/catkin_ws/src/lcm/build/test/types/lcm-test-types.jar:/home/zcx/catkin_ws/src/lcm/build/lcm-java/lcm.jar:/home/zcx/catkin_ws/src/lcm/test/java/junit-4.11.jar:/home/zcx/catkin_ws/src/lcm/test/java/hamcrest-core-1.3.jar" "LcmTestClient")
set_tests_properties(Java::client_server PROPERTIES  _BACKTRACE_TRIPLES "/home/zcx/catkin_ws/src/lcm/test/java/CMakeLists.txt;30;add_test;/home/zcx/catkin_ws/src/lcm/test/java/CMakeLists.txt;0;")
