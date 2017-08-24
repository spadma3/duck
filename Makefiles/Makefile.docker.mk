
### Docker

tag=10

docker:
	@echo "$(sep)Docker commands"
	@echo
	@echo "For using Docker images"
	@echo
	@echo "  docker-build     creates the image"
	@echo "  docker-upload    uploads the image"
	@echo "  docker-clean     removes all local images"

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
