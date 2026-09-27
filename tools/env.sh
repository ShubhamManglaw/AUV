#!/usr/bin/env bash

# ROS 2 Jazzy setup
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi

# Python virtual environment
if [ -f "$HOME/venvs/ps11/bin/activate" ]; then
  source "$HOME/venvs/ps11/bin/activate"
fi

# Local workspace setup if built
if [ -f "$HOME/ps11-auv/ros2_ws/install/setup.bash" ]; then
  source "$HOME/ps11-auv/ros2_ws/install/setup.bash"
fi

# Project root and ROS domain settings
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # stops other ROS machines on venue Wi-Fi from interfering
export PS11_ROOT="$HOME/ps11-auv"

# Handle DISPLAY and XAUTHORITY if DISPLAY is empty
if [ -z "$DISPLAY" ]; then
  if [ -S /tmp/.X11-unix/X0 ]; then
    export DISPLAY=":0"
  elif [ -S /tmp/.X11-unix/X1 ]; then
    export DISPLAY=":1"
  else
    export DISPLAY=":0"
  fi
  _USER_ID=$(id -u)
  export XAUTHORITY="/run/user/${_USER_ID}/gdm/Xauthority"
  unset _USER_ID
fi
