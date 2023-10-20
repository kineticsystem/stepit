#! /bin/bash -e

# Use this script to create, start, stop and remove a docker container to
# build your local ROS2 project.
# It is also used to build your code inside the docker container.

function stop_container() {
    local name="$1"
    # Check if the container is running.
    if docker ps -a | grep -q $name; then
        # Stop the container
        echo "Stopping container: $name"
        docker stop $name > /dev/null
    fi
}

function remove_container() {
    local name="$1"
    # Check if the container is running.
    if docker ps -a | grep -q $name; then
        # Check if the container exists
        echo "Removing container: $name"
        docker rm $name > /dev/null
    fi
}

function remove_image() {
    local name="$1"
    # Check if the image exists.
    if [[ $(docker images -q $name:latest) ]]; then
        # Remove the image
        echo "Removing image: $name:latest"
        docker rmi $name:latest > /dev/null
    fi
}

function build_image() {
    local name="$1"
    # Check if the image exists.
    if ! docker images | grep -q $name:latest; then
        # Build the image
        echo "Building image: $name:latest"
        docker build -t $name:latest .
    fi
}

function create_container() {
    local name="$1"
    local repo_path="$2"
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

parse_options() {
    while [ "$#" -gt 0 ]; do
        case "$1" in
            --repo=*)
                repo="${1#*=}"
                shift
                ;;
            *)
                echo "Unknown option: $1"
                return 1
                ;;
        esac
    done
    return 0
}

function is_container_running() {
    local name="$1"
    # Check if the container is running.
    if docker ps | grep -q $name; then
        echo "true"
    else
        echo "false"
    fi
}

function print_help() {
    echo -e "Usage: ./dock.sh <container-name> <command> [options]\n
    Commands:
    build   Build a container without starting it
            Usage: ./dock.sh container-name build path-to-repo
    start   Start the container and open an interactive terminal
            Usage: ./dock.sh container-name start
    stop    Stop the container
            Usage: ./dock.sh container-name stop
    clean   Stop the container and clean everything including images
            Usage: ./dock.sh container-name clean\n"
}

cd "$(dirname "$0")"

# Allows any local user to connect to your X server, including the Docker container.
xhost +local: &>/dev/null

name="$1"
if [[ -z "$name" ]]; then
    echo -e "Please provide a container name.\n"
    print_help
    exit 1
fi
shift # Remove the container name from the arguments.

command="$1"
if [[ -z "$command" ]]; then
    echo -e "Please provide a command.\n"
    print_help
    exit 1
fi
shift # Remove the container name from the arguments.

case $command in
    build)
        repo="$1"
        if [[ -n "$repo" ]]; then
            echo "Repository path: $repo"
        else
            echo -e "Please provide a repository path.\n"
            print_help
            exit 1
        fi
        shift # Remove the container name from the arguments.

        stop_container $name   # If running.
        remove_container $name # If one exists.
        build_image $name      # If not already built.
        create_container $name $repo
        ;;
    start)
        # Check if a container is already running.
        if [ "$(is_container_running $name)" = "false" ]; then
            start_container $name
        fi

        attach_container $name
        ;;
    stop)
        stop_container $name
        ;;
    clean)
        stop_container $name   # If running.
        remove_container $name # If one exists.
        remove_image $name     # If one exists.
        ;;
    *)
        echo "Unknown parameter: $1"
        echo
        print_help
        ;;
esac
