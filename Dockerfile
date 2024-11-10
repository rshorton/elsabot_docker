ARG BASE_IMAGE=""
FROM ${BASE_IMAGE}
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -a -G dialout $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip

RUN apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-moveit \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rplidar-ros \
    ros-jazzy-laser-filters \
    ros-jazzy-robot-localization \
    ros-jazzy-joy-linux \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers
ENV SHELL=/bin/bash

# For ros2_control so real-time process priorities can be used
# https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html
RUN addgroup realtime \
    && usermod -a -G realtime $USERNAME

RUN echo "@realtime soft rtprio 99 \n\
@realtime soft priority 99 \n\
@realtime soft memlock 102400 \n\
@realtime hard rtprio 99 \n\
@realtime hard priority 99 \n\
@realtime hard memlock 102400 \n" > /etc/security/limits.conf

# Build libusb hidapi (for xArm Lewansoul package)
RUN apt-get install -y libudev-dev libusb-1.0-0-dev libfox-1.6-dev \
    autotools-dev autoconf automake libtool
RUN mkdir -p /opt/extra && cd /opt/extra && \
    git clone https://github.com/libusb/hidapi.git && \
    cd hidapi && \
    ls -ls && \
    ./bootstrap && \
    ./configure && \
    make && \
    make install

# Note - installing python packages below without a virtual env since inside a Docker (at least until
# this is found to cause an issue)

# Used by 'face' node of robot_head package
RUN pip3 install adafruit-circuitpython-servokit --break-system-packages
RUN apt-get install -y python3-libgpiod gpiod python3-smbus python3-dev i2c-tools libi2c-dev wget
RUN addgroup gpio \
    && usermod -a -G gpio $USERNAME

# Also needed by Roby Head Face node.  These are placed below with versions that support the SBC used by Elsabot.
# However, install the official versions to get other dependencies installed also.    
RUN pip3 install Adafruit-Blinka --break-system-packages && \
    pip3 install Adafruit-PlatformDetect --break-system-packages

# Use modified Adafruit python package to support Odyssey Blue with J4125
# (Supports already exists for J4105 variant)
RUN mkdir -p /opt/adafruit && cd /opt/adafruit && \
    git clone https://github.com/rshorton/Adafruit_Python_PlatformDetect.git && \
    cd Adafruit_Python_PlatformDetect && \
    git checkout seeed_j4125 && \
    cd .. && \
    pip3 install --upgrade Adafruit_Python_PlatformDetect/ --break-system-packages

# Install Luxonis Depthai sdk
RUN wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
RUN python3 -m pip install depthai --break-system-packages

# Used by Robot Head Vision node
RUN pip3 install pyzmq --break-system-package

# MS Speech SDK
RUN mkdir -p /opt/ms_speech && cd /tmp && \
    wget -O SpeechSDK-Linux.tar.gz https://aka.ms/csspeech/linuxbinary && \
    tar --strip 1 -xzf SpeechSDK-Linux.tar.gz -C /opt/ms_speech && \
    ls -R /opt/ms_speech && \
    rm /tmp/SpeechSDK-Linux.tar.gz

# Install ROS dep packages last since the script will be generated after initially building the docker
# and in the future when deps need to be updated.
COPY install_ros_dep_packages.sh /opt
RUN /opt/install_ros_dep_packages.sh

WORKDIR /robot_ws
USER $USERNAME

COPY image_bashrc /home/$USERNAME/.bashrc
COPY image_bash_profile /home/$USERNAME/.bash_profile

CMD ["/bin/bash"]
