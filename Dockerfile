FROM osrf/ros:jazzy-desktop-full
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
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

WORKDIR /robot_ws


# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
COPY image_bashrc /home/$USERNAME/.bashrc
COPY image_bash_profile /home/$USERNAME/.bash_profile

CMD ["/bin/bash"]

COPY install_ros_dep_packages.sh /opt
RUN /opt/install_ros_dep_packages.sh

