#!/usr/bin/env bash

set -euo pipefail

# Avoid `ROS_DISTRO: unbound variable` in shell wrappers that are strict about unset vars.
export ROS_DISTRO="${ROS_DISTRO:-noetic}"

delay="${SERVO_START_DELAY:-0}"

sleep "${delay}"

echo "[delayed_servo_server] starting servo_server for node '$*'" >&2
exec /opt/ros/noetic/lib/moveit_servo/servo_server "$@"
