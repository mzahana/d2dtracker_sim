# d2dtracker_sim
ROS 2 simulation packge of the D2DTracker system

# Dependencies
* ROS 2 humble + Gazebo `garden`
* PX4 Atuopilot

# Installation
The simulation deveopment environment is available in a self-contained docker image. First follow the instructions at [d2dtracker_sim_docker](https://github.com/mzahana/d2dtracker_sim_docker)

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

* In 4th terminal, check the available ros topics
    ```bash
    ros2 topic list
    ```
* In a new terminal, you can check the `tf` tree using
    ```bash
    ros2 run rqt_tf_tree rqt_tf_tree
    ```

# References
* Building custom world SDF models: [webpage](https://gazebosim.org/docs/latest/sdf_worlds) , [video](https://youtu.be/48TX-XJ14Gs)
* [Online gazebo model library](https://app.gazebosim.org/fuel)