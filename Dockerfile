FROM ros:kinetic-robot

# If this argument It accepts the default answer for all questions.
ARG DEBIAN_FRONTEND=noninteractive

# Install dependencies.
RUN apt-get -qq update \
    && apt-get -qq --no-install-recommends install protobuf-compiler \
    && apt-get -qq --no-install-recommends install libprotobuf-dev \
    && apt-get -qq --no-install-recommends install ros-kinetic-gazebo-ros-pkgs \
    && apt-get -qq --no-install-recommends install ros-kinetic-gazebo-ros-control \
    && apt-get -qq --no-install-recommends install openssh-server \
    && apt-get -qq --no-install-recommends install nano \
    && apt-get -qq clean    \
    && rm -rf /var/lib/apt/lists/*

# Open port 22 for SSH and SFTP.
EXPOSE 22   

# Start ssh service for SSH and SFTP.
ENTRYPOINT service ssh restart && bash  

# Create new username.
RUN useradd -ms /bin/bash username

# give privilege to username.
RUN usermod -aG sudo username

# Change user
USER username

WORKDIR /home/username

# make sure changed password before changing user: passwd username.
USER root

# source your ros distro
RUN echo "source /opt/ros/kinetic/setup.bash" >> .bashrc

RUN mkdir catkin_ws && cd catkin_ws/ && mkdir src && cd src && git clone https://github.com/hyfan1116/pgm_map_creator.git



