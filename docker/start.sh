#! /bin/bash -e

# Use this script to start a docker container and mount your local code.
# Remember to export the variable STEPIT_PATH to point to the root of your
# ROS2 project.

# Allows any local user to connect to your X server, including the Docker container.
xhost +local:

# Stop the current container.
docker-compose down

# Remove the image.
docker rmi stepit:latest 2>/dev/null || true

# Build the image.
docker-compose build

# Create a container and start it in background.
docker-compose up -d

# Install missing packages and build.
docker-compose exec app /home/developer/repo/stepit/src/stepit/bin/update.sh
docker-compose exec app /home/developer/repo/stepit/src/stepit/bin/build.sh

# To connect to a running container use:
docker-compose exec app bash

