#!/bin/bash -e

# This script sets up the D2DTracker simulation environment

if [ -z "${DEV_DIR}" ]; then
  echo "Error: DEV_DIR environment variable is not set. Set it using export DEV_DIR=<DEV_DIR_deirectory_that_should_contain_PX4-Autopilot_and_ros2_ws>"
  exit 1
fi

ROS2_WS=$DEV_DIR/ros2_ws
ROS2_SRC=$DEV_DIR/ros2_ws/src
PX4_DIR=$DEV_DIR/PX4-Autopilot

# # Make sure that PX4 root directory is set
# if [ -z "${PX4_ROOT}" ]; then
#   echo "Error: PX4_ROOT environment variable is not set. Set it using export PX4_ROOT=<PX4-ROOT_deirectory_that_contains_PX4-Autopilot>"
#   exit 1
# fi

# # Make sure that ROS2_WS directory is set
# if [ -z "${ROS2_WS}" ]; then
#   echo "Error: ROS2_WS environment variable is not set. Set it using export ROS2_WS=<ROS2_WS_deirectory_that_contains_ros2_ws>"
#   exit 1
# fi

if [ ! -d "$ROS2_WS" ]; then
  echo "Creating $ROS2_SRC"
  mkdir -p $ROS2_SRC
fi

SIM_PKG_URL=''
if [[ -n "$GIT_USER" ]] && [[ -n "$GIT_TOKEN" ]]; then
    SIM_PKG_URL=https://$GIT_USER:$GIT_TOKEN@github.com/mzahana/d2dtracker_sim.git
else
    SIM_PKG_URL=https://github.com/mzahana/d2dtracker_sim.git
fi

# Clone the d2dtracker_sim if it doesn't exist
if [ ! -d "$ROS2_SRC/d2dtracker_sim" ]; then
    cd $ROS2_SRC
    git clone $SIM_PKG_URL
else
    cd $ROS2_SRC/d2dtracker_sim && git pull origin main
fi

# Clone and build PX4-Autopilot if it doesn't exist
if [ ! -d "$PX4_DIR" ]; then
    echo "Cloning $PX4_DIR..."
    cd $DEV_DIR
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    cd $PX4_DIR
    make clean
    make distclean
    git checkout main
    make submodulesclean
else
    echo "PX4_DIR=$PX4_DIR already exists"
fi

# Copy files to $PX4_DIR
echo && echo  "Copying files to ${PX4_DIR}" && echo
sleep 1
cp -r ${ROS2_SRC}/d2dtracker_sim/models/* ${PX4_DIR}/Tools/simulation/gz/models/
cp -r ${ROS2_SRC}/d2dtracker_sim//config/px4/* ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/

# Clone some PX4 rose-related packages
if [ ! -d "$ROS2_SRC/px4_msgs" ]; then
    cd $ROS2_SRC
    git clone https://github.com/PX4/px4_msgs.git
else
    cd $ROS2_SRC/px4_msgs && git pull origin main
fi

if [ ! -d "$ROS2_SRC/px4_ros_com" ]; then
    cd $ROS2_SRC
    git clone https://github.com/PX4/px4_ros_com.git
else
    cd $ROS2_SRC/px4_ros_com && git pull origin main
fi

cd $ROS2_WS && colcon build

echo "DONE. Pkgs are built. Models and airframe config files are copied to the respective folder in the ${PX4_DIR} directory"
echo "Source the ros2_ws and ruse os2 launch d2dtracker_sim run_sim.launch.py to run the simulation"
cd $HOME