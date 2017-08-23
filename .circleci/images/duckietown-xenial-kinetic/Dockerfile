# This is an auto generated Dockerfile for ros:perception
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:kinetic-ros-base-xenial

# install ros packages
RUN apt-get update

RUN apt-get install -y ros-kinetic-perception=1.3.1-0*

RUN apt-get install -y git git-extras ros-indigo-tf-conversions ros-indigo-cv-bridge ros-indigo-image-transport ros-indigo-camera-info-manager ros-indigo-theora-image-transport ros-indigo-joy ros-indigo-image-proc ros-indigo-compressed-image-transport ros-indigo-phidgets-drivers ros-indigo-imu-complementary-filter ros-indigo-imu-filter-madgwick python-numpy python-scipy

RUN rm -rf /var/lib/apt/lists/*
