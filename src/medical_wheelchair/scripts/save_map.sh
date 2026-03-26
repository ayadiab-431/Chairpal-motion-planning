#!/bin/bash
# Save the map from slam_toolbox/nav2_map_server

MAP_NAME=${1:-map}
MAP_DIR="${MAP_DIR:-$HOME/.ros/maps}"

mkdir -p $MAP_DIR

echo "Saving map to $MAP_DIR/$MAP_NAME (using sim time)..."
# Pass use_sim_time:=true as a ROS parameter to the map_saver_cli
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$MAP_NAME" --ros-args -p save_map_timeout:=15.0 -p use_sim_time:=true
