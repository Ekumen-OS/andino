### Docker

#### Prerequisites

It is a requirement to have `docker engine` already installed in the host machine.

* See [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/)

For NVIDIA GPU support, `nvidia-container-toolkit` should be installed. *Skip this step if you don't have an NVIDIA graphics card*


* Make sure you have the drivers installed:
  ```sh
  nvidia-smi
  ```
* See [NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

#### Building image and running container

The Docker setup supports both ROS 2 Humble and Jazzy distributions.

- Build the docker image (defaults to Humble):

```sh
./docker/build.sh
```

To build for Jazzy:

```sh
./docker/build.sh --ros_distro jazzy
```

You can also set a specific image name:

```sh
./docker/build.sh -i my_fancy_image_name
```

- Run a docker container:

```sh
./docker/run.sh
```

To run Jazzy:

```sh
./docker/run.sh --ros_distro jazzy
```

- **IMPORTANT**: If you are using nvidia drivers add the `--use_nvidia` flag:

```sh
./docker/run.sh --use_nvidia
```

You can also set specific image and container names:

```sh
./docker/run.sh --use_nvidia -i my_fancy_image_name -c my_fancy_container_name
```

- Inside the container, install dependencies via `rosdep`:

  For Humble:
  ```sh
  rosdep install -i -y --rosdistro humble --from-paths src
  ```

  For Jazzy:
  ```sh
  rosdep install -i -y --rosdistro jazzy --from-paths src
  ```

Note that the repository is mounted into a workspace. That is convenient if you
are working in a single repository project. Note that for multi-repository
workspace you should use another tool like vcs-tool to control via a `.repos`
file the repositories in your workspace.

- To build:

  ```sh
  colcon build
  ```
