#!/bin/bash

xhost +

docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/root/.Xauthority:ro \
    --device /dev/dri \
    --group-add video \
    -v $(pwd)/src:/app/src \
    cyclone-dearpygui