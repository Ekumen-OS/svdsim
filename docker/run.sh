#!/bin/bash

# Copyright 2024 Ekumenlabs Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -o errexit

cd $(dirname "$(readlink -f "$0")")

[[ ! -z "${WITHIN_DEV}" ]] && echo "Already in the development environment!" && exit 1

# Check for stop command
if [[ "$1" == "stop" ]]; then
    SERVICE_NAME="${2:-svdsim_devel}"
    CONTAINER_NAME="${USER}_${SERVICE_NAME}"
    echo "Stopping container: ${CONTAINER_NAME}"
    docker stop ${CONTAINER_NAME} 2>/dev/null && echo "Container stopped." || echo "No running container found."
    exit 0
fi

# function to print help options
function print_help {
    echo "Usage: $(basename $0) [-b|--build] [-D|--daemon] [-s|--service SERVICE_NAME] [-d|--disable-localhost-only] [-h|--help] [-- COMMAND [ARGS...]]"
    echo "       $(basename $0) stop [SERVICE_NAME]"
    echo ""
    echo "Options:"
    echo "  --build                      Build the image before starting the container."
    echo "  --daemon                     Run container in detached mode (background)."
    echo "  --service                    Set the service name (default: svdsim_devel)."
    echo "  --disable-localhost-only     Set ROS_AUTOMATIC_DISCOVERY_RANGE to \"SUBNET\"."
    echo "  --help                       Display this help message."
    echo "  --                           Pass everything after this as command and arguments to docker run."
    echo ""
    echo "Commands:"
    echo "  stop [SERVICE_NAME]          Stop the running container (default service: svdsim_devel)."
    echo ""
    echo "Examples:"
    echo "  $(basename $0)                          # Run default container"
    echo "  $(basename $0) --daemon                 # Run in background"
    echo "  $(basename $0) -- bash                  # Run bash in container"
    echo "  $(basename $0) -- ros2 topic list       # Run ROS 2 command"
    echo "  $(basename $0) --build -- bash          # Build image then run bash"
    echo "  $(basename $0) stop                     # Stop default container"
    echo "  $(basename $0) stop svdsim_devel        # Stop specific container"
}

set +o errexit
VALID_ARGS=$(OPTERR=1 getopt -o bhDds: --long build,help,daemon,disable-localhost-only,service: -- "$@")
RET_CODE=$?
set -o errexit

if [[ $RET_CODE -eq 1 ]]; then
    print_help
    exit 1;
fi
if [[ $RET_CODE -ne 0 ]]; then
    >&2 echo "Unexpected getopt error"
    exit 1;
fi

BUILD=false
DAEMON=false
SERVICE_NAME="svdsim_devel"
ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST"
DOCKER_COMMAND=()

eval set -- "$VALID_ARGS"
while [[ "$1" != "" ]]; do
    case "$1" in
    -b | --build)
        BUILD=true
        shift
        ;;
    -D | --daemon)
        DAEMON=true
        shift
        ;;
    -s | --service)
        SERVICE_NAME="$2"
        shift 2
        ;;
    -d | --disable-localhost-only)
        ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"
        shift
        ;;
    -h | --help)
        print_help
        exit 0
        ;;
    --) # start of positional arguments
        shift
        # Capture remaining arguments as docker command
        DOCKER_COMMAND=("$@")
        break
        ;;
    *)
        >&2 echo "Unrecognized positional argument: $1"
        print_help
        exit 1
        ;;
    esac
done

# if the ROS_DOMAIN_ID variable is not set, abort
if [[ -z "${ROS_DOMAIN_ID}" ]]; then
  echo
  echo "The ROS_DOMAIN_ID environment variable must be set."
  echo
  exit 1
fi

CONTAINER_NAME="${USER}_${SERVICE_NAME}"

# Build compose file arguments
COMPOSE_FILES="-f docker-compose.yml -f docker-compose.host.yml"

if [[ "$BUILD" = true ]]; then
 USERID=$(id -u) GROUPID=$(id -g) \
    docker compose ${COMPOSE_FILES} build ${SERVICE_NAME} \
 && echo "Built ${SERVICE_NAME} image." || \
    { echo "Failed to build ${SERVICE_NAME} image."; exit 1; }
fi

# Add --detach flag if daemon mode is enabled
if [[ "$DAEMON" = true ]]; then
    DETACH_FLAG="--detach"
else
    DETACH_FLAG=""
fi

USERID=$(id -u) GROUPID=$(id -g) ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE} \
    docker compose ${COMPOSE_FILES} run \
    --service-ports \
    --name ${CONTAINER_NAME} \
    --remove-orphans \
    ${DETACH_FLAG} \
    --rm ${SERVICE_NAME} \
    "${DOCKER_COMMAND[@]}"

if [[ "$DAEMON" = true ]]; then
    echo "Container starting in daemon mode: ${CONTAINER_NAME}"
    echo "Waiting for container initialization..."

    # Wait for the fixuid_complete file to be created by the entrypoint
    TIMEOUT=90
    ELAPSED=0
    while [[ $ELAPSED -lt $TIMEOUT ]]; do
        if docker exec ${CONTAINER_NAME} test -f fixuid_complete 2>/dev/null; then
            echo "Container initialized successfully."
            break
        fi
        sleep 0.5
        ELAPSED=$((ELAPSED + 1))
    done

    if [[ $ELAPSED -ge $TIMEOUT ]]; then
        echo "Warning: Timeout waiting for container initialization."
        exit 1
    fi

    echo "Container running in background: ${CONTAINER_NAME}"
    echo "Use '$(basename $0) stop' to stop the container."
fi
