#!/bin/bash
set -e
 
# ROS_DISTRO and WS are set as environment variables in the image
source "/opt/ros/${ROS_DISTRO}/setup.bash"
 
# Workspace overlay, available once you've run `colcon build`
if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
fi
 
exec "$@"