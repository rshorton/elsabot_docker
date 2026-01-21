# Elsabot ROS2 Jazzy Docker Environment

This repo contains files to build and run a ROS docker container for the Elsabot robot.

## Prepare source

Create a ROS workspace directory (such as robot_ws):

```
mkdir -p ~/robot_ws/src
```
and clone this repo under it.

```
cd ~/robot_ws
git clone git@github.com:rshorton/elsabot_docker.git
```

You should have:
 - ~/robot_ws
     - src
     - elsabot_docker

Clone any other repos needed such as elsabot_4wd and its dependencies in the src folder.
 
micro_ros is used so clone it also:
```
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```

As an alternative to the above, use the get_elsabot_repos.sh script from
the *elsabot* repo git@github.com:rshorton/elsabot.git
to clone all the necessary repos.

## Build the docker image

Run:
```
cd ~/robot_ws
elsabot_docker/build_ros_docker.sh
```
To build for arm64 specify the -a option.
```
elsabot_docker/build_ros_docker.sh -a
```
This will take a while to build the docker image.


## Create and open a shell in the container:

````
cd ~/robot_ws
elsabot_docker/run_ros_docker.sh
````

This will open a shell inside the docker container.  The shell inside the container will automatically source the file *elsabot_docker/robot.env*.  Revise that file to export any environment variables you need.  It will also source the ROS workspace install directory and the *elsabot_docker/device_env_vars.env* file.

Build a list of the ROS dep packages required for building the Elsabot packages:
```
elsabot_docker/build_ros_deps_for_docker.sh
```

Update the ROS docker with those dependent packages.  This will exit the shell running inside the container so the docker image can be updated:
```
exit
elsabot_docker/build_ros_docker.sh
```

This will install the dependencies into the image.  After it finishes, run the docker again, and then build the Elsabot packages (ros2_build is a convenience alias):

```
ros2_build
source install/setup.bash
```

## micros_ros related
See:
    https://github.com/micro-ROS/micro_ros_setup/blob/jazzy/README.md

The micro_ros source is cloned in a step above and the dependencies are installed above too. To create the agent:

```
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

For the Elsabot firmware for the Arduino controller, see https://github.com/rshorton/linorobot2_hardware

## ROS Foxglove Bridge

### Install

Install using the Foxglove-provided ROS package if available using:

```
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

### Build

Since no package available for jazzy, build it from source.  Use the steps below for this docker.  Here's a link to the directions: https://github.com/foxglove/foxglove-sdk/tree/main/ros/src/foxglove_bridge#install


Open a shell into the docker container:

```
cd ~/robot_ws
elsabot_docker/run_ros_docker.sh
```

Clone and build the source

```
git clone https://github.com/foxglove/foxglove-sdk
cd foxglove-sdk/ros
make
```

### Run

```
cd ~/robot_ws
source foxglove-sdk/ros/install/local_setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

### Open Foxgove dashboard

Browse to

```
https://app.foxglove.dev/<your user name>/dashboard
```
Select Open Connection from the drop-down menu and specify the connection url such as:

```
ws://your_robot_ip_address:8765
```