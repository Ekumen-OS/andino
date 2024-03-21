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

- Build the docker image whose default name is `ros2_humble_andino`:

```sh
./docker/build.sh
```

You can also try to set a specific image name:

```sh
./docker/build.sh -i my_fancy_image_name
```

- Run a docker container from `ros2_humble_andino` called `ros2_humble_andino_container`:

```sh
./docker/run.sh
```

- **IMPORTANT**: If you are using nvidia drivers add the `--use_nvidia` flag:

```sh
./docker/run.sh --use_nvidia
```

You can also try to set specific image and container names:

```sh
./docker/run.sh --use_nvidia -i my_fancy_image_name -c my_fancy_container_name
```

- Inside the container, install dependencies via `rosdep`:

  ```sh
  rosdep install -i -y --rosdistro humble --from-paths src
  ```

Note that the repository is mounted into a workspace. That is convenient if you
are working in a single repository project. Note that for multi-repository
workspace you should use another tool like vcs-tool to control via a `.repos`
file the repositories in your workspace.

- To build:

  ```sh
  colcon build
  ```
### Building a docker image for the Raspberry PI

This is convenient to avoid running the installation process in the Raspberry Pi, and to have a unified environment. The following will guide you to cross compile the docker image from an x86/x64 computing platform into an arm64 compatible image.

#### Configure the `buildx` driver

Run the following to see the `docker buildx` nodes:

```
$ docker buildx ls
NAME/NODE            DRIVER/ENDPOINT             STATUS  BUILDKIT             PLATFORMS
default              docker                                                   
  default            default                     running v0.11.7+d3e6c1360f6e linux/amd64, linux/amd64/v2, linux/amd64/v3, linux/amd64/v4, linux/386
```

As you can see, the `default` cannot build images for `linux/arm64/v8`. Those are required for running in the Raspberry Pi. You can create a node that works for that the following way:


```
docker buildx create --name andino_arm64_arch --platform "linux/arm64/v8" --driver "docker-container"
```

And select it then:

```
$ docker buildx use andino_arm64_arch
$ docker buildx ls
NAME/NODE            DRIVER/ENDPOINT             STATUS  BUILDKIT             PLATFORMS
andino_arm64_arch *  docker-container                                         
  andino_arm64_arch0 unix:///var/run/docker.sock running v0.12.4              linux/arm64*, linux/amd64, linux/amd64/v2, linux/amd64/v3, linux/amd64/v4, linux/386
default              docker                                                   
  default            default                     running v0.11.7+d3e6c1360f6e linux/amd64, linux/amd64/v2, linux/amd64/v3, linux/amd64/v4, linux/386
```

#### Build the arm64/v8 image

At this point, the only required step is building the image:


Run the following (known bug: https://github.com/multiarch/alpine/issues/32 ) so sudo can execute:

```
docker run --rm --privileged multiarch/qemu-user-static:register --reset --credential yes
```
and then:

```
docker buildx build -t andino:arm64 \
  --platform linux/arm64/v8 \
  --build-arg USERID=$(id -u) \
  --build-arg USER=$(whoami) \
  --load \
  --file docker/Dockerfile.arm64 .
```

The previous command will create a docker image called `andino` and tag it with `arm64`.

That's it! You have an andino docker image to run in your Raspberry Pi.

#### Transfer it to the Raspberry Pi

After a successful build, consider running the following to save to a tarball the docker image and copying it to the Raspberry Pi:

```
docker save andino:arm64 > andino_arm64.tar
```

You can copy it via `scp`:

```
scp andino_arm64.tar <andino_ip>@<andino_ip>:/tmp
```

And then load and execute the docker image there:

```
ssh <andino_ip>@<andino_ip>
docker load < andino_arm64.tar
docker run \
  --rm \
  --privileged \
  --net=host \
  -it \
  -v /dev:/dev \
  --name andino_container \
  andino:arm64
```

As you can see, you will need docker installed in the Raspberry Pi, [here](https://docs.docker.com/engine/install/ubuntu/) are the installation docs.
