#!/usr/bin/env bash
# Cleanup script for PS11 simulation processes (§8, T1.2).
set -e

patterns=(
  "gz sim"
  "parameter_bridge"
  "image_bridge"
  "robot_state_publisher"
  "gz topic"
  "ros2 launch ps11"
  "waypoint_follower"
  "odom_noise"
  "depth_sim"
  "range_adapter"
  "detector"
  "tracker"
  "geolocator"
  "contact_db"
  "scheduler"
  "link_emulator"
  "surface_decoder"
  "metrics"
  "foxglove_bridge"
  "fake_vehicle"
)

get_pids() {
  local pids=()
  for pat in "${patterns[@]}"; do
    while IFS= read -r pid; do
      if [ -n "$pid" ] && [ "$pid" != "$$" ] && [ "$pid" != "$PPID" ]; then
        pids+=("$pid")
      fi
    done < <(pgrep -f "$pat" 2>/dev/null || true)
  done
  if [ ${#pids[@]} -gt 0 ]; then
    printf "%s\n" "${pids[@]}" | sort -u
  fi
}

pids_to_kill=($(get_pids))

if [ ${#pids_to_kill[@]} -gt 0 ]; then
  kill -INT "${pids_to_kill[@]}" 2>/dev/null || true
fi

sleep 2

remaining_pids=($(get_pids))
if [ ${#remaining_pids[@]} -gt 0 ]; then
  kill -9 "${remaining_pids[@]}" 2>/dev/null || true
fi

echo "cleanup done"

final_pids=($(get_pids))
if [ ${#final_pids[@]} -gt 0 ]; then
  echo "Still running:"
  for pid in "${final_pids[@]}"; do
    ps -p "$pid" -o pid,args --no-headers 2>/dev/null || true
  done
fi
