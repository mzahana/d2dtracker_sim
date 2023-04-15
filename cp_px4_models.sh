#!/bin/bash -eu

# Make sure that PX4 directory is set
if [ -z "${PX4_DIR}" ]; then
  echo "Error: PX4_DIR environment variable is not set. Set it using export PX4_DIR=<PX4-Autopilot_directory>"
  exit 1
fi

# Find the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Script directory = ${SCRIPT_DIR}"

sleep 2

# Copy files to $PX4_DIR
# echo  && echo "Cleanup ${PX4_DIR} ... "
# sleep 1
# cd ${PX4_DIR} && make clean && make distclean && git checkout main && make submodulesclean

echo && echo  "Copying files to ${PX4_DIR}" && echo
sleep 1
cp -r ${SCRIPT_DIR}/models/* ${PX4_DIR}/Tools/simulation/gz/models/
cp -r ${SCRIPT_DIR}/worlds/* ${PX4_DIR}/Tools/simulation/gz/worlds/
cp -r ${SCRIPT_DIR}/config/px4/* ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/

# Build px4_sitl
cd $PX4_DIR && make px4_sitl

echo "DONE. Models and airframe config files are copied to the respective folder in the ${PX4_DIR} directory"
cd
