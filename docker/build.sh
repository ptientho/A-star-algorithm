#!/bin/bash
set -e

echo "Building the docker image for A* search program"
IMAGE_NAME="astar-search-program"
DOCKERFILE_PATH="$(cd "$(dirname "$0")"; pwd)"/dockerfile
PARENT_PATH="$(cd "$(dirname "$0")" ; cd .. ; pwd)"
USERID=$(id -u)
USER=$(whoami)

sudo docker build -t $IMAGE_NAME \
    --file $DOCKERFILE_PATH \
    --build-arg USERID=$USERID \
    --build-arg USER=$USER \
    $PARENT_PATH

