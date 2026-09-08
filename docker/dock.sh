#! /bin/bash -e

# Use this script to create, start, stop and remove a docker container to
# build your local ROS2 project.
#
# The container itself is defined in docker-compose.yml. This script only adds
# what compose cannot express: the X server permission, the host uid/gid, and
# the container name passed on the command line.

function container_exists() {
    local name="$1"
    [[ $(docker ps -aq --filter name=^/${name}$) ]]
}

# Whether the container was created from this compose file, and can therefore
# be managed by compose. A container of the same name left over from an older
# version of this script, or from another project, is not one of ours: compose
# refuses to reuse the name, so it has to be handled with plain docker.
function image_exists() {
    local name="$1"
    [[ $(docker images -q $name) ]]
}

function compose_owns_container() {
    local name="$1"
    local project
    project=$(docker inspect "$name" \
      --format '{{index .Config.Labels "com.docker.compose.project"}}' 2>/dev/null)
    [[ "$project" == "$name" ]]
}

function display_usage() {
    echo -e "\nUsage: ./dock.sh <container-name> <command> [options]\n
    Commands:
    build   Build a container without starting it
            Usage: ./dock.sh container-name build
    start   Start the container and open an interactive terminal
            Usage: ./dock.sh container-name start
    stop    Stop the container
            Usage: ./dock.sh container-name stop
    clean   Stop the container and clean everything including images
            Usage: ./dock.sh container-name clean\n"
}

# Compose resolves the paths in docker-compose.yml against the directory that
# holds it, so every command has to run from there.
cd "$(dirname "$0")"

# Check for at least two arguments (container name and command).
if [ "$#" -lt 2 ]; then
    echo "Missing required arguments."
    display_usage
    exit 1
fi

name="$1"
command="$2"

# The container name doubles as the compose project name, so several
# differently named containers can coexist from this same compose file.
export CONTAINER_NAME="$name"
export COMPOSE_PROJECT_NAME="$name"

# Build args for the Dockerfile: a container user matching the host user.
export USER_UID=$(id -u)
export USER_GID=$(id -g)

case "$command" in
    build)
        # Compose cannot take over a container it did not create, so drop it
        # and build a fresh one, as this command has always done.
        if container_exists $name && ! compose_owns_container $name; then
            echo "Removing container '$name', which predates docker-compose.yml"
            docker rm --force $name > /dev/null
        fi

        # Unlike a plain `docker build`, this rebuilds the layers that the
        # Dockerfile changed since last time, so there is no need to clean
        # first.
        docker compose build
        docker compose create
        ;;
    start)
        # Allow any local user, including the container, to connect to the X
        # server.
        xhost +local: &>/dev/null
        if container_exists $name && ! compose_owns_container $name; then
            # Attach to the container that is already there rather than fail on
            # the name conflict. Run `build` to replace it with one that matches
            # the current compose file.
            echo "Warning: container '$name' predates docker-compose.yml; using it as is." >&2
            docker start $name > /dev/null
        else
            docker compose up --detach
        fi
        echo "Opening interactive terminal into $name"
        docker exec -it $name bash
        ;;
    stop)
        docker compose stop
        ;;
    clean)
        docker compose down --rmi local --remove-orphans
        # A container or an image made before docker-compose.yml existed belongs
        # to no compose project, so the command above leaves it behind.
        if container_exists $name; then
            echo "Removing container '$name', which predates docker-compose.yml"
            docker rm --force $name > /dev/null
        fi
        if image_exists $name:latest; then
            echo "Removing image: $name:latest"
            docker rmi $name:latest > /dev/null
        fi
        ;;
    *)
        echo "Unknown parameter: $command"
        display_usage
        ;;
esac
