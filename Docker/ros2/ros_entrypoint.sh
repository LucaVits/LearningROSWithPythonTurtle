#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"
export ROS_DOMAIN_ID=30

## colcon ##
source /usr/share/colcon_cd/function/colcon_cd.sh
