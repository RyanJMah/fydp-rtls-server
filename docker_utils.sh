#!/usr/bin/env bash

IMAGE_NAME="rtls_server_debian_bullseye"
CONTAINER_NAME="rtls_server"

set -e

function clean_docker() {
    set +e
    docker stop $CONTAINER_NAME &> /dev/null

    docker rm $CONTAINER_NAME &> /dev/null
    docker rmi $IMAGE_NAME &> /dev/null

    set -e
}

function build_img_and_container() {
    set +e
    docker stop $CONTAINER_NAME &> /dev/null
    docker rm $CONTAINER_NAME &> /dev/null
    set -e

    docker build -t $IMAGE_NAME .
    docker container create -p 1883:1883 -it --name $CONTAINER_NAME $IMAGE_NAME
}

if [[ $1 == "help" ]]
then
    echo "Commands:"
    echo "    clean      - purge server docker image and container, and remove cache"
    echo "    build      - build server docker image and container"
    echo "    start      - (Re)start the container"
    echo "    stop       - stop the container"
    echo "    shell      - get a shell into the container"
    echo "    logs       - tail the server logs"
    echo "    mqtt_logs  - tail the mqtt broker logs"
fi

if [[ $1 == "clean" ]]
then
    clean_docker
    exit 0
fi

if [[ $1 == "stop" ]]
then
    set +e
    docker stop $CONTAINER_NAME
    set -e

    exit 0
fi

if [[ $1 == "build" ]]
then
    build_img_and_container
    exit 0
fi

if [[ $1 == "start" ]]
then
    docker start $CONTAINER_NAME
    exit 0
fi

if [[ $1 == "shell" ]]
then
    docker exec -it $CONTAINER_NAME bash
fi

if [[ $1 == "logs" ]]
then
    docker exec rtls_server bash -c "tail -f /app/logs/rtls_server.log"
fi

if [[ $1 == "mqtt_logs" ]]
then
    docker exec rtls_server bash -c "tail -f /app/logs/mosquitto.log"
fi
