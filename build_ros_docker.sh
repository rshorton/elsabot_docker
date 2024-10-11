#!/bin/bash

IMAGE_NAME=elsabot/jazzy

NO_CACHE_ARG=""
while getopts ":hn" option; do
   case $option in
      h) # display Help
         echo "Syntax: -n  New image"
         exit
         ;;
      n) # New image
         NO_CACHE_ARG="--no-cache"
         ;;
     \?) # Invalid option
         echo "Error: Invalid option"
         exit
         ;;
   esac
done

# Change to the directory where this script is and the Docker file
THIS_SCRIPT=$(readlink -e "$0")
SCRIPT_DIR=$(dirname "$THIS_SCRIPT")
cd $SCRIPT_DIR


# Docker file expects this script so create a dummy
# if not present.
ROS_DEPS_INSTALL_SCRIPT="install_ros_dep_packages.sh"
if [ ! -e $ROS_DEPS_INSTALL_SCRIPT ]; then
  touch $ROS_DEPS_INSTALL_SCRIPT
  chmod +x $ROS_DEPS_INSTALL_SCRIPT
fi

docker build $NO_CACHE_ARG --build-arg USERNAME=elsabot -t $IMAGE_NAME . 2>&1 | tee build.log
