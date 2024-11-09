# Elsabot ROS2 Jazzy Docker Environment

This repo contains file to build and run a ROS docker container for the Elsabot robot.

## Prepare source

Create a ROS workspace directory (such as robot_ws):

```
mkdir -p ~robot_ws/src
```
and clone this repo under it.

```
git clone git@github.com:rshorton/elsabot_docker.git
```

You should have:
 - ~/robot_ws
     - src
     - elsabot_docker

Clone any other repos needed such as elsabot_wd and its dependencies in the src folder.
 
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
elsabot_docker/build_ros_docker.sh
```

This will take a while to build the docker image.


## Create and open a shell in the container:

````
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

This will install the dependencies into the image.  After it finishes, run the docker again, and then build the Elsabot packages (using a convenience alias):

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

