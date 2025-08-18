#/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <image_name>"
    exit 1
fi

docker stop magnipoc || true
docker rm magnipoc || true
docker run -d \
    --device /dev/ttyAMA0 \
    --cap-add=sys_nice \
    --ulimit rtprio=99 \
    --ulimit memlock=-1 \
    --name magnipoc $1