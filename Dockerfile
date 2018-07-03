FROM duckietown/rpi-ros-kinetic-picam

MAINTAINER Breandan Considine breandan.considine@umontreal.ca

RUN [ "cross-build-start" ]

# RUN mkdir /home/software
COPY . /home/software/
COPY docker/machines.xml /home/software/catkin_ws/src/00-infrastructure/duckietown/machines

RUN echo "source /home/software/docker/env.sh" >> ~/.bashrc

RUN /bin/bash -c "cd /home/software/ && source environment.sh && catkin_make -C catkin_ws/"

# RUN git clone https://github.com/duckietown/duckiefleet /home/duckiefleet

RUN [ "cross-build-end" ]

WORKDIR /home/software

CMD [ "/bin/bash" ]
