#!/bin/bash

# Note, the device links created by udev on the host side of the container are
# mounted when running the container otherwise they aren't visible in the container.
# See elsabot_4wd/hwsetup/99-usb-serial.rules for the links that are created for the
# various devices used.

# USB device handling
#   The host udev rules for the USB serial devices create sym links under the directory:
#     /dev/elsabot_dev_links
#   That directory is mapped to the container.  See robot.env where env variables are
#   defined that specify the symbol links for each device.  Those env vars are used by the
#   launch scripts.
# /dev/input is mapped so /dev/input/js0 is available.
# /dev/bus/usb and cgroup rule for Oakd device
# Sound devices for speech_input_server and speech_output_server projects.

SCRIPT_DIR=$(dirname "$0")
WS_DIR=$(readlink -f ${SCRIPT_DIR}/..)
echo "WS_DIR = $WS_DIR"

xhost +local:docker

# FIX, is privileged still needed?
docker run -it --privileged --net=host  --pid=host --ipc=host \
  -e HOST_WS_DIR=${WS_DIR} \
  -e DISPLAY=unix:0 \
  -v ${WS_DIR}:/robot_ws \
  -v /dev/elsabot_dev_links:/dev/elsabot_dev_links \
  -v /dev/input:/dev/input \
  -v /dev/i2c-0:/dev/i2c-0 \
  -v /dev/i2c-1:/dev/i2c-1 \
  -v /dev/i2c-2:/dev/i2c-2 \
  -v /dev/i2c-3:/dev/i2c-3 \
  -v /dev/i2c-4:/dev/i2c-4 \
  -v /dev/i2c-5:/dev/i2c-5 \
  -v /dev/i2c-6:/dev/i2c-6 \
  -v /dev/i2c-7:/dev/i2c-7 \
  -v /dev/i2c-10:/dev/i2c-10 \
  -v /dev/bus/usb:/dev/bus/usb \
  --device-cgroup-rule='c 189:* rmw' \
  -v /dev/snd:/dev/snd \
  -e PULSE_SERVER=unix:/run/user/1000/pulse/native -v /run/user/1000/pulse:/run/user/1000/pulse \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/.Xauthority:$HOME/.Xauthority -e XAUTHORITY=$HOME/.Xauthority \
  elsabot/jazzy \
   /bin/bash
  
