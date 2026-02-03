#!/bin/bash

THIS_SCRIPT=$(readlink -e "$0")
SAVE_DIR=$(dirname "$THIS_SCRIPT")

ROS_DEPS_INSTALL_SCRIPT=$SAVE_DIR/install_ros_dep_packages.sh
echo "Saving deps to: $ROS_DEPS_INSTALL_SCRIPT"

echo "#!/bin/bash" > $ROS_DEPS_INSTALL_SCRIPT

rosdep update
rosdep install -si --reinstall --from-path src | \
  awk '/#[apt]*/,0' >> $ROS_DEPS_INSTALL_SCRIPT

sed -i -e 's/get install/get install -y/g' $ROS_DEPS_INSTALL_SCRIPT

chmod +x $ROS_DEPS_INSTALL_SCRIPT
echo "Built ros dep install script for use by the Docker build file."
echo "You should now exit the container, and then rerun the docker build script."