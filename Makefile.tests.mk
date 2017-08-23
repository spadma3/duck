
comptests_packages=\
	easy_node_tests\
	easy_logs_tests\
	easy_algo_tests\
	duckietown_utils_tests\
	what_the_duck_tests

comptests_out=out/comptests

comptests-clean:
	rm -rf $(comptests_out)

comptests:
	comptests -o $(comptests_out) --nonose -c "rparmake" $(comptests_packages)
