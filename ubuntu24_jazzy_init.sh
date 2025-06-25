#!/bin/bash


# sudo apt update; sudo apt install postgresql ros-jazzy-turtlebot3* 



# sudo apt update; sudo apt install ros-noetic-joint-trajectory-* ros-noetic-control* ros-noetic-dwa-local-planner ros-noetic-turtlebot3* ros-noetic-moveit* ros-noetic-gazebo-*

# SCRIPT_DIR="$(dirname "$(realpath "$0")")"

# Install Mechlmm Py
# pip install -e $SCRIPT_DIR/mechlmm_py/.

# Set environment variable
echo "========== Env Check =============="

if ! grep -q 'export TURTLEBOT3_MODEL' ~/.bashrc; then
  echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
  echo "TURTLEBOT3_MODEL environment variable added to .bashrc"
else
  echo "TURTLEBOT3_MODEL is already set to:=========>" $TURTLEBOT3_MODEL
fi

if ! grep -q "alias cb" ~/.bashrc; then
  echo "alias cb='colcon build --symlink-install && source install/setup.bash'" >> ~/.bashrc
  echo "set colcon build alias => .bashrc"
else
  echo "colcon build alias is already set "
fi

if ! grep -q "alias si" ~/.bashrc; then
  echo "alias si='source install/setup.bash'" >> ~/.bashrc
  echo "set source install/setup.bash alias => .bashrc"
else
  echo "source install/setup.bash alias is already set "
fi

source ~/.bashrc