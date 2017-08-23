
comptests_packages=easy_node_tests
comptests_out=out/comptests

comptests-clean:
	rm -rf $(comptests_out)

comptests:
	comptests -o $(comptests_out) --nonose -c "rparmake" $(comptests_packages)
