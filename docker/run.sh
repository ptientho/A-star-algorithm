#!/bin/bash

set +e

# Location of the repo
REPO_FOLDER_PATH="$(cd "$(dirname "$0")"; cd .. ; pwd)"
WORKSPACE_SRC_CONTAINER=/home/$(whoami)/A-star-algorithm
CONTAINER_NAME="astar-search-container"
IMAGE_NAME="astar-search-program"

sudo docker run -it --privileged \
                -v ${REPO_FOLDER_PATH}/images:$WORKSPACE_SRC_CONTAINER/images:rw \
                --name $CONTAINER_NAME $IMAGE_NAME