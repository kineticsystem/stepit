# Developing inside a docker container

In this tutorial, we explain how to develop, build and run this project inside a docker container.

The container user and password are:

**developer:developer**

## Prerequisites

First, you must install `docker`.

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

## Build and start up a container

From the root of the repo, run this script to create an image and a container:

```bash
./docker/dock.sh [container-name] build
```

Run this to start the container with an interactive shell:

```bash
./docker/dock.sh [container-name] start
```

Run this to stop the container:

```bash
./docker/dock.sh [container-name] stop
```

Finally, run this to remove container and image:

```bash
./docker/dock.sh [container-name] clean
```

## Working with the code

Using the terminal, move to the root of your project and run the following commands:

```bash
rosdep update
rosdep install --ignore-src --from-paths . -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --symlink-install --event-handlers log-
```

Alternatively, you can run these scripts:

```bash
./bin/update.sh
./bin/build.sh
```
