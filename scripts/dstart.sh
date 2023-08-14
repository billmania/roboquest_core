#!/usr/bin/env bash

#
# Start the docker container for roboquest_core
#

IMAGE=$1
NAME=rq_core

printf "Starting %s on %s\n" "$IMAGE" $DOCKER_HOST

docker run -d --rm \
        --privileged \
        --network host \
        --ipc host \
        --device /dev/gpiomem:/dev/gpiomem \
        --device /dev/i2c-1:/dev/i2c-1 \
        --device /dev/i2c-6:/dev/i2c-6 \
        --device /dev/ttyS0:/dev/ttyS0 \
        -v /dev/shm:/dev/shm \
        -v /var/run/dbus:/var/run/dbus \
        -v /run/udev:/run/udev \
        -v ros_logs:/root/.ros/log \
        --name $NAME \
        "$IMAGE"

if [[ $? = 0 ]]
then
    docker container ls
    printf "\nStarted\n"
else
    printf "Failed to start %s\n" $NAME
fi

exit 0
