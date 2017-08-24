
continuous-integration-tests:
	$(MAKE) test-comptests
	$(MAKE) test-easy_node

### Comptests

test-easy_node:
	$(MAKE) easy_node-docs

### Comptests

comptests_packages=\
	easy_node_tests\
	easy_logs_tests\
	easy_algo_tests\
	duckietown_utils_tests\
	what_the_duck_tests

comptests_out=out/comptests

comptests-clean:
	rm -rf $(comptests_out)

test-comptests:
	comptests -o $(comptests_out) --nonose -c "rparmake" $(comptests_packages)


### Docker

tag=9

docker-build:
	sudo docker build -t andreacensi/duckietown-xenial-kinetic:$(tag) .circleci/images/duckietown-xenial-kinetic/

docker-upload:
	sudo docker push andreacensi/duckietown-xenial-kinetic:$(tag)
