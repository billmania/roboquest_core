#!/usr/bin/env bash

#
# Start the docker container for roboquest_core
#

IMAGE=$1
NAME=rq_core
PERSIST_DIR="/usr/src/ros2ws/install/roboquest_core/share/roboquest_core/persist"

printf "Starting %s on %s\n" "$IMAGE" "$(docker context show)"

docker run -d --rm \
        --privileged \
        --network host \
        --ipc host \
        --env "ROS_DOMAIN_ID=72" \
        --device /dev/gpiomem:/dev/gpiomem \
        --device /dev/i2c-1:/dev/i2c-1 \
        --device /dev/i2c-6:/dev/i2c-6 \
        --device /dev/ttyS0:/dev/ttyS0 \
        -v /dev/shm:/dev/shm \
        -v /var/run/dbus:/var/run/dbus \
        -v /run/udev:/run/udev \
        -v /opt/persist:${PERSIST_DIR} \
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
