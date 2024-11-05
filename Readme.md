# Elsabot ROS2 Jazzy Docker Environment

These files are used to build a ROS docker for the Elsabot robot.

Clone this repo under your ROS work space directory.  Ex:
 - robot_ws
     - elsabot_docker

To build the docker image, in the robot_ws directory run:
```
elsabot_docker/build_ros_docker.sh
```

To run the created docker, in the robot_ws directory run:
````
  elsabot_docker/run_ros_docker.sh
````

This will open a shell inside the docker container.  The shell inside the container will automatically source the file elsabot_docker/robot.env.  Revise that file to export any Environment variables you need.  It will also source the ROS workspace install directory and the elsabot_docker/device_env_vars.env.

To build a list of the ROS dep packages required for building the Elsabot packages, run:
```
  elsabot_docker/build_ros_deps_for_docker.sh
```

To update the ROS docker with those dependent packages, run:
```
  elsabot_docker/build_ros_docker.sh
```

This will install those packages in the image.  After it finishes, run the docker again, and then build the Elsabot packages.  

micros_ros related
  See:
    https://github.com/micro-ROS/micro_ros_setup/blob/jazzy/README.md

  Steps (run in docker):

    cd /robot_ws
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
    rosdep update && rosdep install --from-paths src --ignore-src -y
    colcon build --packages-select micro_ros_setup
    source install/local_setup.bash

    Build the microros agent:
      ros2 run micro_ros_setup create_agent_ws.sh
      ros2 run micro_ros_setup build_agent.sh

  For Elsabot firmware for the Arduino controller, see https://github.com/rshorton/linorobot2_hardware

10/10/2024

Needed to build rosbridge_suite from source due to issue related to Jazzy.
  cd src 
  git clone https://github.com/RobotWebTools/rosbridge_suite.git
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
  colcon build
  