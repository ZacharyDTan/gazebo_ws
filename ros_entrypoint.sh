#!/bin/bash
# set -e 
export DISPLAY=${DISPLAY}
export XAUTHORITY=/tmp/.X11-unix

source /opt/ros/humble/setup.bash
source /workspaces/gazebo_ws/install/setup.bash

exec "$@"