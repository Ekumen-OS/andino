FROM osrf/ros:jazzy-desktop-full

# Arguments for building
ARG USER=ubuntu

# Setup environment
ENV TERM=linux
ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Copy requirement files and install dependencies
COPY docker/requirements.txt .
RUN apt-get update && apt-get install --no-install-recommends -y $(cat requirements.txt)
RUN rm requirements.txt

# Create a user with passwordless sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USER/.bashrc

# Install fixuid and configure it
RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.6.0/fixuid-0.6.0-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $USER\n" > /etc/fixuid/config.yml

# Adds USER to dialout and plugdev group.
# This is needed to access the serial ports, for further references check
# the libserial documentation.
RUN sudo usermod -a -G dialout $USER
RUN sudo usermod -a -G plugdev $USER

USER $USER

# Creates the src folder of the workspace.
RUN mkdir -p /home/$USER/ws/src

# Adds to bashrc the ros jazzy overlay sourcing.
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USER/.bashrc

# Updates
RUN sudo apt update && sudo apt upgrade -y && rosdep update

# Defines a workspace folder.
WORKDIR /home/$USER/ws

# Use fixuid as the entrypoint in quiet mode
ENTRYPOINT ["fixuid", "-q"]

CMD ["/bin/bash"]
