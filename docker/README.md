# Developing inside a docker container

In this tutorial, we explain how to develop, build and run this project inside a docker container.

## Prerequisites

First, you must install `docker`.

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

## Build and start up a container

Run this script to create an image and a container:

```bash
./stepit.sh docker build --repo=absolute-repo-path
```

Run this to start the container with an interactive shell:

```bash
./stepit.sh docker start
```

Run this to stop the container:

```bash
./stepit.sh docker stop
```

Finally, run this to remove container an image:

```bash
./stepit.sh docker clean
```

## Working with the code

Using the terminal, move to the root of your project and run the following commands:

```bash
rosdep update
rosdep install --ignore-src --from-paths . -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --symlink-install --event-handlers log-
```
