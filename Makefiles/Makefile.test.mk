

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
	@echo '- `make test-download-logs`: Downloads the logs needed for the tests.'
	@echo
	@echo

test-circle: \
	test-comptests-circle \
	test-download-logs \
	test-misc-utils


	#test-line-detector-programmatic

#
# test-catkin_tests \
# test-anti_instagram
#

test-all: \
	test-comptests \
	test-download-logs \
	test-catkin_tests \
	test-misc-utils

### Comptests

comptests_packages=\
	easy_node_tests\
	easy_logs_tests\
	easy_algo_tests\
	duckietown_utils_tests\
	line_detector2_tests\
	what_the_duck_tests\
	duckieteam_tests\
	complete_image_pipeline_tests\
	duckietown_segmaps_tests\
	lane_filter_generic_tests\
	easy_regression_tests\
	grid_helper_tests

# These take a long time
# anti_instagram_tests\

comptests_out=out/comptests

test-comptests-clean:
	-rm -rf $(comptests_out)

test-comptests-again:
	$(MAKE) test-comptests-clean
	$(MAKE) test-comptests

test-comptests:  test-download-logs
	comptests -o $(comptests_out) --nonose --contracts -c "rparmake" $(comptests_packages)

test-comptests-circle:  test-download-logs
	# comptests -o $(comptests_out) --nonose --contracts -c "rparmake n=3" $(comptests_packages)
	comptests --circle -o $(comptests_out) --nonose -c "rparmake n=4" $(comptests_packages)

test-comptests-slow:  test-download-logs
	comptests -o $(comptests_out) --nonose --contracts -c "rmake" $(comptests_packages)

test-comptests-collect-junit:
	mkdir -p $(comptests_out)/junit
	comptests-to-junit $(comptests_out)/compmake > $(comptests_out)/junit/junit.xml

test-catkin_tests: check-environment
	bash -c "source environment.sh; catkin_make -C $(catkin_ws) run_tests; catkin_test_results $(catkin_ws)/build/test_results/"

# onelog=20160223-amadoa-amadobot-RCDP2
onelog=2016-04-29-dp3auto-neptunus-1

test-download-logs:
	@echo Loading log
	rosrun easy_logs download $(onelog) tori_ETHZ_2017-12-22-17-18-41

test-misc-utils:
	rosrun complete_image_pipeline validate_calibration robbie
	rosrun complete_image_pipeline display_segmaps 'DT17*tile*'

test-cloud-logs: cloud-download
	rosrun easy_logs summary --cloud  $(onelog)

test-documentation:
	echo "<html><head></head><body></body></html>" > catkin_ws/00_main_template.html
	DISABLE_CONTRACTS=1 mcdp-render-manual \
	--src $(catkin_ws) \
	--stylesheet v_manual_split \
	--mathjax 0 \
	-o out/test-documentation \
	--output_file $(out_html).tmp -c "config echo 1; config colorize 0; rparmake; why failed"
	# compmake out/test-documentation -c "ls failed"
	# compmake out/test-documentation -c "why failed"
	rm -f catkin_ws/00_main_template.html


test-publish:
	python -m SimpleHTTPServer 8000 ..

	
