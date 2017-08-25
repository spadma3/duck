

test:
	@echo "$(sep)Testing"
	@echo
	@echo "These commands run the unit tests."
	@echo
	@echo "    test-circle           The tests run on the cloud."
	@echo "    test-catkin_tests     ROS tests."
	@echo "    test-anti_instagram   anti_instagram"
	@echo "    test-comptests        Comptests"

test-circle: \
	test-comptests
	#
	# test-catkin_tests \
	# test-anti_instagram
	#

### Comptests

comptests_packages=\
	easy_node_tests\
	easy_logs_tests\
	easy_algo_tests\
	duckietown_utils_tests\
	what_the_duck_tests

comptests_out=out/comptests

test-comptests-clean:
	-rm -rf $(comptests_out)

test-comptests:
	comptests -o $(comptests_out) --nonose -c "rparmake" $(comptests_packages)

test-comptests-slow:
	comptests -o $(comptests_out) --nonose -c "rmake" $(comptests_packages)

test-comptests-collect-junit:
	mkdir - $(comptests_out)/junit
	comptests-to-junit $(comptests_out)/compmake > $(comptests_out)/junit/junit.xml
# other testss

test-catkin_tests: check-environment
	bash -c "source environment.sh; catkin_make -C $(catkin_ws) run_tests; catkin_test_results $(catkin_ws)/build/test_results/"


test-anti_instagram: check-environment
	bash -c "source environment.sh; rosrun anti_instagram annotation_tests.py"
