
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

tag=10

docker:
	@echo "make docker-build     creates the image"
	@echo "make docker-upload    uploads the image"
	@echo "make docker-clean     removes all local images"

docker-build:
	sudo docker build -t andreacensi/duckietown-xenial-kinetic:$(tag) .circleci/images/duckietown-xenial-kinetic/

docker-upload:
	sudo docker push andreacensi/duckietown-xenial-kinetic:$(tag)

docker-clean:
	# Kill all running containers:
	-sudo docker kill $(shell sudo docker ps -q)

	# Delete all stopped containers (including data-only containers):
	-sudo docker rm $(shell sudo docker ps -a -q)

	# Delete all exited containers
	-sudo docker rm $(shell sudo docker ps -q -f status=exited)

	# Delete all images
	-sudo docker rmi $(shell sudo docker images -q)
