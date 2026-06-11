#/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <image_name>"
    exit 1
fi

docker run -it \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    --name control-station $1
