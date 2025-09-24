#!/bin/bash

echo "Starting sync_time_start.sh"

source /opt/ros/jazzy/setup.bash

source /sync_time_ws/install/setup.sh

exec ros2 run sync_time sync_time_node
