#/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <image_name>"
    exit 1
fi

docker stop magnipoc || true
docker rm magnipoc || true
docker run \
    --device /dev/serial0:/dev/ttyAMA0 \
    --device /dev/i2c-1 \
    --device /dev/ttyUSB0:/dev/ttyUSB0 \
    -v /dev/input:/dev/input \
    --device-cgroup-rule 'c 13:* rmw' \
    --cap-add=sys_nice \
    --ulimit rtprio=99 \
    --ulimit memlock=-1 \
    --name magnipoc $1