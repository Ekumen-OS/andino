FROM osrf/ros:humble-desktop-full

# Arguments for building
ARG USERID
ARG USER

# Setup environment
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Create a user with passwordless sudo
RUN adduser --uid $USERID --gecos "ekumen developer" --disabled-password $USER
RUN adduser $USER sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USER/.bashrc
USER $USER

# Adds USER to dialout and plugdev group.
# This is needed to access the serial ports, for further references check
# the libserial documentation.
RUN sudo usermod -a -G dialout $USER
RUN sudo usermod -a -G plugdev $USER

# Creates the src folder of the workspace.
RUN mkdir -p /home/$USER/ws/src

# Adds to bashrc the ros humble overlay sourcing.
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc
# Adds colcon autocomplete
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USER/.bashrc

# Updates
RUN sudo apt upgrade -y && sudo apt update && rosdep update

# Defines a workspace folder.
WORKDIR /home/$USER/ws

CMD ["/bin/bash"]
