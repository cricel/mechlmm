#!/bin/bash
sudo apt update; sudo apt install ros-noetic-joint-trajectory-* ros-noetic-control* ros-noetic-dwa-local-planner ros-noetic-turtlebot3* ros-noetic-moveit* ros-noetic-gazebo-*

SCRIPT_DIR="$(dirname "$(realpath "$0")")"

# Install Mechlmm Py
pip install -e $SCRIPT_DIR/mechlmm_py/.

# Set environment variable
echo "========== Env Check =============="

if ! grep -q 'export TURTLEBOT3_MODEL' ~/.bashrc; then
  echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
  echo "TURTLEBOT3_MODEL environment variable added to .bashrc"
else
  echo "TURTLEBOT3_MODEL is already set to:=========>" $TURTLEBOT3_MODEL
fi

if ! grep -q 'export MECHLMM_IP' ~/.bashrc; then
  echo 'export MECHLMM_IP="172.21.255.131"' >> ~/.bashrc
  echo "MECHLMM_IP environment variable added to .bashrc"
else
  echo "MECHLMM_IP is already set to:=========>" $MECHLMM_IP
fi

source ~/.bashrc