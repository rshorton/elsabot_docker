#!/bin/bash

# Note, the device links created by udev on the host side of the container are
# mounted when running the container otherwise they aren't visible in the container.
# See elsabot_4wd/hwsetup/99-usb-serial.rules for the links that are created for the
# various devices used.

docker run -it --privileged --net=host  --pid=host --ipc=host \
  -e DISPLAY=unix:0 \
  -v ~/robot_ws:/robot_ws \
  -v /dev/teensy:/dev/teensy \
  -v /dev/PX1122R_gps:/dev/PX1122R_gps \
  -v /dev/PX1122R_gps_rtcm:/dev/PX1122R_gps_rtcm \
  -v /dev/rplidar:/dev/rplidar \
  -v /dev/ublox7_gps:/dev/ublox7_gps \
  -v /dev/create2:/dev/create2 \
  -v /dev/servo_driver:/dev/servo_driver \
  elsabot/jazzy \
   /bin/bash