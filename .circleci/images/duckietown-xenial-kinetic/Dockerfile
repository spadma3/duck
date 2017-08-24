# This is an auto generated Dockerfile for ros:perception
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:kinetic-ros-base-xenial

# install ros packages
RUN apt-get update

RUN apt-get install -y ros-kinetic-perception=1.3.1-0*

RUN apt-get install -y git git-extras ros-kinetic-desktop-full ros-kinetic-tf-conversions ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-camera-info-manager ros-kinetic-theora-image-transport ros-kinetic-joy ros-kinetic-image-proc ros-kinetic-compressed-image-transport ros-kinetic-phidgets-drivers ros-kinetic-imu-complementary-filter ros-kinetic-imu-filter-madgwick python-numpy python-scipy

RUN apt-get install -y ssh python-pip python-numpy python-scipy
# RUN rm -rf /var/lib/apt/lists/*

# Other deps for duckuments
RUN apt-get install -y pdftk bibtex2html libxml2-dev libxslt1-dev libffi6 libffi-dev python-dev python-matplotlib virtualenv

RUN pip install empy catkin_pkg numpy

# mcdp dependencies
RUN pip install pint pyparsing networkx watchdog pyramid pyramid_jinja2 pyramid_debugtoolbar bs4 nose reprep bcrypt markdown junit_xml lxml bcrypt waitress ruamel.yaml gitpython webtest PyContracts decentlogs chardet pillow

RUN pip install comptests
