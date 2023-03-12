# d2dtracker_sim
ROS 2 simulation packge of the D2DTracker system

# Dependencies
* ROS 2 humble
* PX4 Atuopilot

# Installation
* Follow the instrucitons in the [px4_ros2_humble](https://github.com/mzahana/px4_ros2_humble)
* Inside the container 
    * Export the `PX4_DIR`: `export PX4_DIR=path_to_PX4-Aitopilot_directory_inside_container`
    * Run the `cp_px4_models.sh` script 
        ```bash
        cd d2dtracker_sim
        ./cp_px4_models.sh
        ```
    * Build the workspace `colcon build`
    * Source the workspace `source install setup.bash`

# Run
To run the Gazebo simulation execute the following command after you build and source the workspace as mentioned above.
```bash
ros2 launch d2dtracker_sim run_sim.launch.py
```