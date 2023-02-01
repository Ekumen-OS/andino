# Download base image ubuntu 20.04
FROM ubuntu:20.04

# Arguments for building
ARG USERID
ARG USER

# Image name
ARG "Ubuntu base for ros noetic development and Gazebo 11 in ubuntu focal."

# Setup environment
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Copy installation scripts, and perform the dependency installation.
COPY docker/install.sh .
COPY docker/requirements.txt .
COPY docker/requirements_ign.txt .
COPY docker/requirements_ros.txt .
RUN ./install.sh
RUN rm install.sh requirements.txt requirements_ign.txt requirements_ros.txt

# Create a user with passwordless sudo
RUN adduser --uid $USERID --gecos "developer" --disabled-password $USER
RUN adduser $USER sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USER/.bashrc
USER $USER

# Run rosdep init and update.
RUN sudo rosdep init && rosdep update

# Creates the src folder of the workspace.
RUN mkdir -p /home/$USER/ws/src

# Adds to bashrc the ros noetic overlay sourcing.
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USER/.bashrc

# Create ignition fuel config to avoid Gazebo taking long on startup
RUN mkdir -p /home/$USER/.ignition/fuel
COPY docker/gazebo_config/config.yaml /home/$USER/.ignition/fuel

# Defines a workspace folder.
WORKDIR /home/$USER/ws

# This is a workaround to install the repo dependencies in the building process.
# We copy the repo on a temporary folder to execute 'rosdep install', then we remove
# the generated files from the apt-update and the temporary folder 
COPY $PWD /home/$USER/ws/src/temp_repo
RUN sudo apt-get update && \
    rosdep update && rosdep install -i -y --rosdistro noetic --from-paths src && \
    sudo apt-get upgrade -y && \
    sudo rm -rf /var/lib/apt/lists/* && \
    sudo rm -rf /home/$USER/ws/src/temp_repo

CMD ["/bin/bash"]
