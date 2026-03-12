#!/usr/bin/env bash

set -eo pipefail

WORKSPACE="${1:-$HOME/movo_servo_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "[setup] Repository root: ${REPO_ROOT}"
echo "[setup] Workspace: ${WORKSPACE}"

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${VERSION_ID:-}" != "20.04" ]]; then
    echo "[setup] Warning: this setup targets Ubuntu 20.04, detected ${PRETTY_NAME:-unknown}."
  fi
fi

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "[setup] ROS Noetic not found. Installing ROS Noetic base tooling."
  sudo apt-get update
  sudo apt-get install -y gnupg2 lsb-release ca-certificates wget

  if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
    wget -qO- https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
      | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
  fi

  echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
    | sudo tee /etc/apt/sources.list.d/ros1-latest.list >/dev/null

  sudo apt-get update
  sudo apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    build-essential
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash

if ! command -v rosdep >/dev/null 2>&1 || ! command -v catkin >/dev/null 2>&1; then
  echo "[setup] Installing missing ROS build tools."
  sudo apt-get update
  sudo apt-get install -y python3-rosdep python3-catkin-tools
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  echo "[setup] Initializing rosdep."
  sudo rosdep init
fi
rosdep update

mkdir -p "${WORKSPACE}/src"

for legacy_pkg in gazebo_force_based_move gazebo_grasp_plugin roboticsgroup_gazebo_plugins; do
  rm -rf "${WORKSPACE}/src/${legacy_pkg}"
done

link_pkg() {
  local source_dir="$1"
  local target_name
  target_name="$(basename "${source_dir}")"
  ln -sfn "${source_dir}" "${WORKSPACE}/src/${target_name}"
}

echo "[setup] Linking required packages into workspace."
link_pkg "${REPO_ROOT}/movo_common/movo_description"
link_pkg "${REPO_ROOT}/movo_7dof_moveit_config"
link_pkg "${REPO_ROOT}/movo_simulation/movo_gazebo"
link_pkg "${REPO_ROOT}/movo_servo"

echo "[setup] Installing package dependencies with rosdep."
rosdep install --from-paths "${WORKSPACE}/src" --ignore-src --rosdistro noetic -r -y

echo "[setup] Building workspace."
catkin config --workspace "${WORKSPACE}" --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build --workspace "${WORKSPACE}"

cat <<EOF
[setup] Done.

Run in each terminal before launching:
  source /opt/ros/noetic/setup.bash
  source ${WORKSPACE}/devel/setup.bash

Launch MOVO + MoveIt + Servo:
  roslaunch movo_servo movo_servo_sim.launch arm:=right

Launch the Twist bridge:
  roslaunch movo_servo movo_servo_input.launch

Run keyboard teleop:
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/movo_servo/cmd_vel
EOF
