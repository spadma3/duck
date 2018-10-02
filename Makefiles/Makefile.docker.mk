
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
tag=20

docker-build-unittests:
	docker build -t $(docker_image_name):$(tag) $(docker_dir)

docker-upload-unittests:
	docker push $(docker_image_name):$(tag)


branch=$(shell git rev-parse --abbrev-ref HEAD)

image_rpi_gui_tools_name=duckietown/rpi-gui-tools:$(branch)
image_rpi_gui_tools_file=Dockerfile.rpi-gui-tools

docker-build-rpi-gui-tools:
	docker build -t $(image_rpi_gui_tools_name) -f $(image_rpi_gui_tools_file) .

docker-build-rpi-gui-tools-no-cache:
	docker build -t $(image_rpi_gui_tools_name) -f $(image_rpi_gui_tools_file) --no-cache .


