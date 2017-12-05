# CMake generated Testfile for 
# Source directory: /home/yunfantang/duckietown/catkin_ws/src/30-localization-and-planning/localization
# Build directory: /home/yunfantang/duckietown/catkin_ws/build_isolated/localization
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_localization_rostest_tests_localization_tester_node.test "/home/yunfantang/duckietown/catkin_ws/build_isolated/localization/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/yunfantang/duckietown/catkin_ws/build_isolated/localization/test_results/localization/rostest-tests_localization_tester_node.xml" "--return-code" "/opt/ros/kinetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/yunfantang/duckietown/catkin_ws/src/30-localization-and-planning/localization --package=localization --results-filename tests_localization_tester_node.xml --results-base-dir \"/home/yunfantang/duckietown/catkin_ws/build_isolated/localization/test_results\" /home/yunfantang/duckietown/catkin_ws/src/30-localization-and-planning/localization/tests/localization_tester_node.test ")
subdirs(gtest)
