# Developing inside a docker container

In this tutorial, we explain how to develop, build and run this project inside a docker container.

## Prerequisites

First, you must install `docker` and `docker-compose`.

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

```bash
sudo apt install docker-compose
```

Additionally, you must export the environment variable `STEPIT_PATH` to point to the root of your repository, for example:

```bash
export STEPIT_PATH=$HOME/repo/stepit
```

## Build and start up a container

Run the script located in this same folder:

```bash
./run.sh
```

This script will build the image and start the container with an interactive shell.

## Working with the code

Using the terminal, move to the root of your project and run the following commands:

```bash
rosdep install --ignore-src --from-paths . -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --symlink-install --event-handlers log-
```