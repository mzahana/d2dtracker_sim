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

* Combile the workspace using `colcon build`

* Source the workspace `source install/setup.bash`

* In the 1st terminal, run the interceptor simulation
    ```bash
    ros2 launch d2dtracker_sim interceptor.launch.py
    ```
    You should see one quadcopter in Gazebo

* In 2nd terminal, run the target simulation
    ```bash
    ros2 launch d2dtracker_sim target.launch.py
    ```
    You should see a second quadcopter spawned in Gazebo

* In 3rd terminal run the DDS agent node to receive ros topics
    ```bash
    ros2 run d2dtracker_sim microdds
    ```

* In 4th terminal, check the available ros topics
    ```bash
    ros2 topic list
    ```
* In a new terminal, you can check the `tf` tree using
    ```bash
    ros2 run rqt_tf_tree rqt_tf_tree
    ```