FROM ros:kinetic-ros-base-xenial
MAINTAINER Andrea Censi

# needed for adding repository
RUN apt-get update
RUN apt-get install -y \
    software-properties-common \
    gnupg \
    curl

RUN add-apt-repository -y ppa:git-core/ppa
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash

RUN apt-get update

# install ros packages


RUN apt-get install -y \
    ros-kinetic-perception=1.3.1-0* \
    git \
    git-extras \
    ros-kinetic-desktop-full \
    ros-kinetic-tf-conversions \
    ros-kinetic-cv-bridge \
    ros-kinetic-image-transport \
    ros-kinetic-camera-info-manager \
    ros-kinetic-theora-image-transport \
    ros-kinetic-joy \
    ros-kinetic-image-proc \
    ros-kinetic-compressed-image-transport \
    ros-kinetic-phidgets-drivers \
    ros-kinetic-imu-complementary-filter \
    ros-kinetic-imu-filter-madgwick \
    ssh \
    pdftk \
    bibtex2html \
    libxml2-dev \
    libxslt1-dev \
    libffi6 \
    libffi-dev \
    libxml2-dev \
    libxslt1-dev \
    pdftk \
    bibtex2html \
    build-essential \
    graphviz \
    idle \
    virtualenv \
    python-pmw \
    python-imaging \
    python-yaml \
    python-dev \
    python-matplotlib \
    python-numpy \
    python-matplotlib \
    python-setproctitle \
    python-psutil \
    python-lxml \
    python-pillow \
    python-matplotlib \
    python-pip \
    python-tk \
    python-scipy \
    python-frozendict \
    imagemagick \
    graphviz \
    ghostscript \
    git-lfs

# RUN rm -rf /var/lib/apt/lists/*

# Other deps for duckuments


# mcdp dependencies
RUN pip install -U \
    empy\
    catkin_pkg\
    pint \
    pyparsing \
    networkx \
    watchdog \
    pyramid \
    pyramid_jinja2 \
    pyramid_debugtoolbar \
    bs4 \
    nose \
    reprep \
    bcrypt \
    markdown \
    junit_xml \
    lxml \
    bcrypt \
    waitress \
    ruamel.yaml \
    gitpython \
    webtest \
    PyContracts \
    decentlogs \
    chardet

# RUN pip install comptests



RUN wget https://www.princexml.com/download/prince_11.3-1_ubuntu16.04_amd64.deb && dpkg -i prince_11.3-1_ubuntu16.04_amd64.deb
