#!/usr/bin/bash
set -e
source /opt/ros/humble/setup.bash
colcon build
source ./install/setup.bash