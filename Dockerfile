FROM ros:humble-ros-base AS base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
  apt-get install -y curl ros-humble-rviz2

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && \
  apt-get install -y ignition-fortress && \
  rm -rf /var/lib/apt/lists/*

#===========================================================
FROM base AS dev
# Install your package here 
RUN apt-get update && \
  apt-get -y install ros-humble-xacro ros-humble-ros-gz-bridge \
  ros-humble-joint-state-publisher ros-humble-ros-gz-sim 


RUN apt-get update && \
  apt-get install -y \
  build-essential cmake git make g++ \
  python3-colcon-common-extensions python3-pip \
  libx11-dev libxt-dev libxext-dev libgl1-mesa-dev \
  # doxygen tmux \
  # ros-humble-xacro \
  # ros-humble-ros-gz-bridge \
  ros-humble-joint-state-publisher \
  # ros-humble-ros-gz-sim \
  # ros-humble-tf-transformations \
  ros-humble-rmw-cyclonedds-cpp \
  # ros-humble-depthai-ros \
  # ros-humble-navigation2 \
  # ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-rqt-robot-steering \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-navigation2 \
  python3-pandas \
  && rm -rf /var/lib/apt/lists/*


# Install Python libraries
# RUN pip3 install pyserial pynmea2 numpy==1.21.6 ultralytics


# Install Phidgets and SICK drivers if not present
RUN mkdir -p ${WORKSPACE}/src && \
  if [ ! -d ${WORKSPACE}/src/phidgets_drivers ]; then \
  git clone -b humble https://github.com/ros-drivers/phidgets_drivers.git ${WORKSPACE}/src/phidgets_drivers; \
  fi && \
  if [ ! -d ${WORKSPACE}/src/sick_scan_xd ]; then \
  git clone -b master https://github.com/SICKAG/sick_scan_xd.git ${WORKSPACE}/src/sick_scan_xd; \
  fi

ENV WS=gazebo_ws
ENV WORKSPACE=/workspaces/${WS}
WORKDIR /workspaces

COPY --chown=root:root --chmod=700 . /workspaces/gazebo_ws
COPY --chown=root:root ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

WORKDIR ${WORKSPACE}
RUN cat .bashconfig >> ~/.bashrc
RUN ./build.sh