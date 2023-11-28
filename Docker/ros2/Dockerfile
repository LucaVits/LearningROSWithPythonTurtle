# This is an auto generated Dockerfile for ros:desktop
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
# please refer to https://github.com/osrf/docker_images/blob/df19ab7d5993d3b78a908362cdcd1479a8e78b35/ros/foxy/ubuntu/focal/ros-core/Dockerfile
# for more info
# ubuntu 20.04, ROS2 foxy
FROM ros:foxy-ros-base-focal

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO foxy

###################################### user #####################################

ENV SHELL=/bin/bash \
    USER=rpirobotics \
    UID=1000 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8

ENV HOME=/home/${USER}

RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${UID} \
    ${USER} 

RUN echo "root:root" | chpasswd
RUN echo "${USER}:111111" | chpasswd

###################################### basic tools #####################################

RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -yq dist-upgrade \
    && apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends \
    locales \
    cmake \
    make \
    git \
    vim \
    gedit \
    wget \
    sudo \
    lsb-release \
    build-essential \
    net-tools \
    dirmngr \
    gnupg2 \
    mercurial \
    software-properties-common \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libpcap-dev \
    python3-pip \
    python3-setuptools \
    python3-opencv \
    python3-numpy \
    python3-empy  \
    python3-tk \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

##################################### ros2 foxy desktop ################

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-desktop=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*

##################################### PIP ######################################

RUN pip3 install --upgrade pip setuptools

RUN pip3 install \
    pyyaml \
    cython \
    matplotlib \
    pandas \
    requests \
    scikit-image \
    scikit-learn \
    gdown \
    seaborn \
    pygame

##################################### Robot Raconteur ################

RUN add-apt-repository ppa:robotraconteur/ppa && apt-get update

RUN apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends python3-robotraconteur

##################################### setting ###################################################
RUN chown -R ${USER}:${USER} ${HOME}/
RUN echo "${USER} ALL=(ALL)  ALL" >> /etc/sudoers

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

USER ${USER}

# setup ROS2
RUN rosdep update

WORKDIR ${HOME}