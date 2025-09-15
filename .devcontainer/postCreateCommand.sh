#!/bin/bash

cd /sync_time_ws
sudo -E rosdep install --from-paths src --ignore-src -y
colcon build
source /sync_time_ws/install/setup.sh