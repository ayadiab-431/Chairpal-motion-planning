# walkthrough - Phase 3: Autonomous Navigation

I have completed all the requirements for **Phase 3: Autonomous Navigation**. Here is a summary of the work done:

## 1. Navigation Stack (Nav2) Configuration
- **Param Optimization**: Updated [nav2_params.yaml](file:///home/salwa/chair_ws/src/medical_wheelchair/config/nav2_params.yaml) to sync velocity limits with the simulation (0.2 m/s linear, 0.4 rad/s angular).
- **Inflation Layers**: Increased the inflation radius to `0.65m` to ensure safe operation given the wheelchair's dimensions.
- **Topic Consistency**: Now all nodes (Nav2 and Safety Monitor) use the `/scan_fixed` topic produced by the `scan_republisher`.

## 2. A* Path Planning
- **Global Planner**: Confirmed that `NavfnPlanner` is configured with `use_astar: true` in the `planner_server` section of [nav2_params.yaml](file:///home/salwa/chair_ws/src/medical_wheelchair/config/nav2_params.yaml). This ensures optimal path planning.

## 3. Obstacle Avoidance
- **Costmap Layers**: Both global and local costmaps are configured with an `obstacle_layer` that tracks LiDAR data for dynamic avoidance.
- **Safety Monitor**: Updated the `obstacle_detector` node to use `/scan_fixed` for consistent obstacle detection.

## 4. Localization & Mapping
- **Map Portability**: Moved the map files from `~/.ros/maps/` to the package's internal `maps/` directory.
- **Launch Updates**: Updated [nav2_amcl.launch.py](file:///home/salwa/chair_ws/src/medical_wheelchair/launch/nav2_amcl.launch.py) and [map_server.launch.py](file:///home/salwa/chair_ws/src/medical_wheelchair/launch/map_server.launch.py) to automatically load the map from the package.
- **TF Fixes**: Modified [wheelchair.xacro](file:///home/salwa/chair_ws/src/medical_wheelchair/urdf/wheelchair.xacro) to disable redundant TF broadcasting from the Gazebo plugin, allowing the custom `odom_to_tf` node to handle frame normalization correctly.

## 5. Verification Results
- **Build**: Successfully rebuilt the `medical_wheelchair` package with the new structure.
- **Symlink Install**: Verified that maps are correctly linked and accessible by the launch files.

### Instructions to Run:
1.  **Launch Simulation**:
    ```bash
    ros2 launch medical_wheelchair spawn_wheelchair.launch.py
    ```
2.  **Launch Navigation**:
    ```bash
    ros2 launch medical_wheelchair nav2_amcl.launch.py
    ```
3.  **In RViz**:
    - Use the **2D Pose Estimate** tool to set the initial position.
    - Use the **Nav2 Goal** tool to send navigation commands.
