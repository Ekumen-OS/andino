# docker

## Description

This folder contains the necessary files to build and run a [`docker`](https://www.docker.com/) container with Gazebo11 and ROS Noetic.

## Instructions

### Prerequisites

It is a requirement to have `docker engine` already installed in the host machine.

* See [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/)

For NVIDIA GPU support, `nvidia-container-toolkit` should be installed. *Skip this step if you don't have an NVIDIA graphics card*


* Make sure you have the drivers installed:
  ```sh
  nvidia-smi
  ```
* See [NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)


### Building image and running container

- Build the docker image whose name is `ros_melodic`:

```sh
./docker/build.sh
```

You can also try to set a specific image name:

```sh
./docker/build.sh -i my_fancy_image_name
```

- Run a docker container called `carpincho-sim`:

```sh
./docker/run.sh
```

- **IMPORTANT**: If you are using nvidia drivers add the `--use_nvidia` flag:

```sh
./docker/run.sh --use_nvidia
```

You can also try to set specific image and container names:

```sh
./docker/run.sh -i my_fancy_image_name -c my_fancy_container_name
```

- Build the workspace:

```sh
catkin_make
```
