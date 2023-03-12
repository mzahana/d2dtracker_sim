#!/bin/bash -eu

# Make sure that PX4 directory is set
if [ -z "${PX4_DIR}" ]; then
  echo "Error: PX4_DIR environment variable is not set. Set it using export PX4_DIR=<PX4-Autopilot_directory>"
  exit 1
fi

# Find the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Copy files to $PX4_DIR
cd ${PX4_DIR} && make clean && make distclean && git checkout main && make submodulesclean

cp ${SCRIPT_DIR}/models/* ${PX4_DIR}/Tools/simulations/gz/models/
cp ${SCRIPT_DIR}/config/px4/* ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/

echo "Models and airframe config files are copied to the respective folder in the ${PX4_DIR} directory"
cd
