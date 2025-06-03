#! /bin/bash -e

# Use this script to create, start, stop and remove a docker container to
# build your local ROS2 project.
# It is also used to build your code inside the docker container.

function container_exists() {
    local name="$1"
    [[ $(docker ps -aq --filter name=^/${name}$) ]]
}

function image_exists() {
    local name="$1"
    [[ $(docker images -q $name) ]]
}

function is_container_running() {
    local name="$1"
    docker ps --filter name=^/${name}$ --format '{{.Names}}' | grep -q "^${name}$"
}

function stop_container() {
    local name="$1"
    if container_exists $name; then
        echo "Stopping container: $name"
        docker stop $name > /dev/null 2>&1
    fi
}

function remove_container() {
    local name="$1"
    if container_exists $name; then
        echo "Removing container: $name"
        docker rm $name > /dev/null 2>&1
    else
        echo "Warning: Container '$name' does not exist." >&2
    fi
}

function remove_image() {
    local name="$1"
    if image_exists $name:latest; then
        echo "Removing image: $name:latest"
        docker rmi $name:latest > /dev/null 2>&1
    else
        echo "Warning: Image '$name:latest' does not exist." >&2
    fi
}

function build_image() {
    local name="$1"
    if ! image_exists $name:latest; then
        echo "Building image: $name:latest"
        docker build -t $name:latest .
    fi
}

function create_container() {
    local name="$1"
    local repo_path="$2"
    # Convert repo_path to absolute path if it's relative
    repo_path=$(realpath "$repo_path")
    # Start the container and keep it running.
    docker create \
      --name $name \
      --privileged \
      -v ${repo_path}:/home/developer/ws \
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -e DISPLAY=${DISPLAY} \
      -e QT_X11_NO_MITSHM=1 \
      --network host \
      --hostname $name \
      $name:latest \
      sleep infinity
}

function start_container() {
    local name="$1"
    # Start the container and keep it running.
    docker start $name
}

function attach_container() {
    local name="$1"
    # Start an interactive bash shell.
    echo "Openining interactive terminal into $name"
    docker exec -it $name bash
}

function display_usage() {
    echo -e "\nUsage: ./dock.sh <container-name> <command> [options]\n
    Commands:
    build   Build a container without starting it
            Usage: ./dock.sh container-name build [path-to-repo]
            Note: If path-to-repo is omitted, current working directory is used
    start   Start the container and open an interactive terminal
            Usage: ./dock.sh container-name start
    stop    Stop the container
            Usage: ./dock.sh container-name stop
    clean   Stop the container and clean everything including images
            Usage: ./dock.sh container-name clean\n"
}

# Capture the user's current working directory before changing directories
USER_PWD="$PWD"

cd "$(dirname "$0")"

# Allows any local user to connect to your X server, including the Docker container.
xhost +local: &>/dev/null

# Check for at least two arguments (container name and command)
if [ "$#" -lt 2 ]; then
    echo "Missing required arguments."
    display_usage
    exit 1
fi

# Assign the first two arguments to descriptive variables
name="$1"
command="$2"

case "$command" in
    build)
        # If a third argument exists, use it as repo_path, otherwise use user's working directory
        if [ "$#" -ge 3 ]; then
            repo_path="$3"
            # Handle special case of "." to mean user's current directory
            if [ "$repo_path" = "." ]; then
                repo_path="$USER_PWD"
            # If the path is relative, make it relative to the user's original directory
            elif [[ "$repo_path" != /* ]]; then
                repo_path="$USER_PWD/$repo_path"
            fi
        else
            repo_path="$USER_PWD"
        fi
        stop_container $name
        remove_container $name
        remove_image $name
        build_image $name
        create_container $name $repo_path
        ;;
    start)
        if ! is_container_running $name; then
            start_container $name
        fi
        attach_container $name
        ;;
    stop)
        stop_container $name
        ;;
    clean)
        stop_container $name
        remove_container $name
        remove_image $name
        ;;
    *)
        echo "Unknown parameter: $command"
        display_usage
        ;;
esac
