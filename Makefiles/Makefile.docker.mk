
docker:
	@echo "$(sep)Docker commands"
	@echo
	@echo 'For using Docker images'
	@echo
	@echo '- `make docker-build`:    Creates the image.'
	@echo '- `make docker-upload`:   Uploads the image.'
	@echo '- `make docker-clean`:    Removes all local images.'
	@echo
	@echo
docker_dir=.circleci/images/duckietown-xenial-kinetic/
docker_image_name=andreacensi/duckietown-xenial-kinetic
tag=18

docker-build:
	sudo docker build -t $(docker_image_name):$(tag) $(docker_dir)

docker-upload:
	sudo docker push $(docker_image_name):$(tag)

docker-clean:
	# Kill all running containers:
	-sudo docker kill $(shell sudo docker ps -q)

	# Delete all stopped containers (including data-only containers):
	-sudo docker rm $(shell sudo docker ps -a -q)

	# Delete all exited containers
	-sudo docker rm $(shell sudo docker ps -q -f status=exited)

	# Delete all images
	-sudo docker rmi $(shell sudo docker images -q)
