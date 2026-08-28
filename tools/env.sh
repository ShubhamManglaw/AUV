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

# Handle DISPLAY and XAUTHORITY
if [ -z "$DISPLAY" ]; then
  _USER_ID=$(id -u)
  _SESSION_ID=$(timeout 2 loginctl list-sessions --no-legend 2>/dev/null | awk -v uid="$_USER_ID" '$2 == uid && $4 == "seat0" {print $1}' | head -n 1 || true)
  if [ -n "$_SESSION_ID" ]; then
    _DISP=$(timeout 2 loginctl show-session "$_SESSION_ID" -p Display --value 2>/dev/null || true)
  fi
  if [ -z "$_DISP" ]; then
    _X_SOCK=$(ls -1 /tmp/.X11-unix/X* 2>/dev/null | head -n 1 || true)
    if [ -n "$_X_SOCK" ]; then
      _DISP=":${_X_SOCK##*/X}"
    fi
  fi
  export DISPLAY="${_DISP:-:0}"
  export XAUTHORITY="/run/user/${_USER_ID}/gdm/Xauthority"
  unset _USER_ID _SESSION_ID _DISP _X_SOCK
fi
export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins
echo "[env.sh] DISPLAY=$DISPLAY"
