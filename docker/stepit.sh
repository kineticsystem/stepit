#! /bin/bash -e

# Use this script to start a docker container and mount your local code.

IMAGE_NAME="stepit:latest"
CONTAINER_NAME="stepit"

function stop_container() {
    # Check if the container is running.
    if docker ps -a | grep -q $CONTAINER_NAME; then
        # Stop the container
        echo "Stopping container: $CONTAINER_NAME"
        docker stop $CONTAINER_NAME > /dev/null
    fi
}

function remove_container() {
    # Check if the container is running.
    if docker ps -a | grep -q $CONTAINER_NAME; then
        # Check if the container exists
        echo "Removing container: $CONTAINER_NAME"
        docker rm $CONTAINER_NAME > /dev/null
    fi
}

function remove_image() {
    # Check if the image exists.
    if [[ $(docker images -q $IMAGE_NAME) ]]; then
        # Remove the image
        echo "Removing image: $IMAGE_NAME"
        docker rmi $IMAGE_NAME > /dev/null
    fi
}

function build_image() {
    # Check if the image exists.
    if ! docker images | grep -q $IMAGE_NAME; then
        # Build the image
        echo "Building image: $IMAGE_NAME"
        docker build -t $IMAGE_NAME .
    fi
}

function create_container() {
    local repo_path="$1"

    # Start the container and keep it running.
    docker create \
      --name stepit \
      --privileged \
      -v ${repo_path}:/home/developer/repo/stepit \
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -e DISPLAY=${DISPLAY} \
      -e QT_X11_NO_MITSHM=1 \
      --network host \
      --hostname stepit \
      stepit:latest \
      sleep infinity
}

function start_container() {
    local repo_path="$1"

    # Start the container and keep it running.
    docker start $CONTAINER_NAME
}

function attach_container() {
    # Start an interactive bash shell.
    echo "Openining interactive terminal into $CONTAINER_NAME"
    docker exec -it $CONTAINER_NAME bash
}

parse_options() {
    while [ "$#" -gt 0 ]; do
        case "$1" in
            --repo=*)
                repo="${1#*=}"
                shift
                ;;
            --mode=*)
                mode="${1#*=}"
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
    # Check if the container is running.
    if docker ps | grep -q $CONTAINER_NAME; then
        echo "true"
    else
        echo "false"
    fi
}

# Define a function to check if running inside a Docker container
function is_inside_docker() {
    if [ -f "/.dockerenv" ]; then
        echo "true"  # Running inside a Docker container
    else
        echo "false"  # Not running inside a Docker container
    fi
}

function print_help() {
    echo -e "Usage: ./stepit.sh docker <command> [options]\n
    Commands:
    build   Build a container without starting it
            Usage: ./stepit.sh docker build --repo=path-to-repo
    start   Start the container and open an interactive terminal
            Usage: ./stepit.sh docker start
    stop    Stop the container
            Usage: ./stepit.sh docker stop
    clean   Stop the container and clean everything including images
            Usage: ./stepit.sh docker clean\n"
}

# Allows any local user to connect to your X server, including the Docker container.
xhost +local: &>/dev/null

# Default values for options.
repo=""
mode=""

# Check the first parameter.
if [ "$1" == "docker" ]; then

    # Check if I am inside a container.
    if [ "$(is_inside_docker)" = "true" ]; then
        echo "You cannot run this command inside the container."
    fi

    shift  # Remove "docker" from the arguments.

    case $1 in
        build)
            shift  # Remove "start" from the arguments.

            # Parse the remaining options.
            parse_options "$@"

            # Check if path was set
            if [[ -n "$repo" ]]; then
                echo "The repository path is set to: $repo"
            else
                echo "Please provide a repository path."
                exit 1
            fi

            stop_container   # If running.
            remove_container # If one exists.
            build_image      # If not already built.
            create_container $repo
            ;;
        start)
            # Check if a container is already running.
            if [ "$(is_container_running)" = "false" ]; then
                start_container
            fi

            attach_container
            ;;
        stop)
            stop_container
            ;;
        clean)
            remove_container # If one exists.
            remove_image     # If one exists.
            ;;
        *)
            echo "Unknown parameter: $1"
            echo
            print_help
            ;;
    esac
else
    print_help
fi
