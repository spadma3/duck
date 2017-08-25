

test:
	@echo "$(sep)Testing"
	@echo
	@echo "These commands run the unit tests."
	@echo
	@echo '- `make test-all`:              Run all the tests.'
	@echo
	@echo '- `make test-circle`:           The tests to run in continuous integration. .'
	@echo '- `make test-catkin_tests`:     Run the ROS tests.'
	@echo '- `make test-anti_instagram`:   Run the `anti_instagram` tests.'
	@echo '- `make test-comptests`:        Run the `comptests` tests.'
	@echo '- `make test-comptests-clean`:        Run the `comptests` tests.'
	@echo '- `make test-comptests-collect-junit`: Collects the JUnit results.'

test-circle: \
	test-comptests
	#
	# test-catkin_tests \
	# test-anti_instagram
	#

test-all: \
	test-comptests \
	test-catkin_tests \
	test-anti_instagram

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
	mkdir -p $(comptests_out)/junit
	comptests-to-junit $(comptests_out)/compmake > $(comptests_out)/junit/junit.xml

test-catkin_tests: check-environment
	bash -c "source environment.sh; catkin_make -C $(catkin_ws) run_tests; catkin_test_results $(catkin_ws)/build/test_results/"


test-anti_instagram: check-environment
	bash -c "source environment.sh; rosrun anti_instagram annotation_tests.py"
