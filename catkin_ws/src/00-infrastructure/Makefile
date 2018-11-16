docker_dir=.
docker_image_name=duckietown/logs
tag?=latest

docker-build:
	# docker build -t $(docker_image_name):$(tag) $(docker_dir)
	docker build -t $(docker_image_name) $(docker_dir)

docker-upload:
	# docker push $(docker_image_name):$(tag)
	docker push $(docker_image_name):latest

docker-clean:
	# Kill all running containers:
	-docker kill $(shell sudo docker ps -q)

	# Delete all stopped containers (including data-only containers):
	-docker rm $(shell sudo docker ps -a -q)

	# Delete all exited containers
	-docker rm $(shell sudo docker ps -q -f status=exited)

	# Delete all images
	-docker rmi $(shell sudo docker images -q)


#docker-run:
#	docker run -it $(docker_image_name):$(tag) python -m duckietown_utils

