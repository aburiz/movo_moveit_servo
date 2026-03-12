#!/usr/bin/env bash

set -eo pipefail

WORKSPACE="${1:-$HOME/movo_servo_teleop_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "[setup] Repository root: ${REPO_ROOT}"
echo "[setup] Workspace: ${WORKSPACE}"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "[setup] ROS Noetic not found. Installing."
  sudo apt-get update
  sudo apt-get install -y gnupg2 lsb-release ca-certificates wget

  if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
    wget -qO- https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
      | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
  fi

  echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
    | sudo tee /etc/apt/sources.list.d/ros1-latest.list >/dev/null

  sudo apt-get update
  sudo apt-get install -y ros-noetic-desktop-full python3-rosdep python3-catkin-tools
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash

echo "[setup] Installing required ROS packages for this demo."
sudo apt-get update
sudo apt-get install -y \
  ros-noetic-moveit \
  ros-noetic-moveit-servo \
  ros-noetic-joy

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

mkdir -p "${WORKSPACE}/src"
ln -sfn "${REPO_ROOT}/movo_common/movo_description" "${WORKSPACE}/src/movo_description"
ln -sfn "${REPO_ROOT}/movo_7dof_moveit_config" "${WORKSPACE}/src/movo_7dof_moveit_config"
ln -sfn "${REPO_ROOT}/movo_servo_teleop_demo" "${WORKSPACE}/src/movo_servo_teleop_demo"

rosdep install --from-paths "${WORKSPACE}/src" --ignore-src --rosdistro noetic -r -y

catkin config --workspace "${WORKSPACE}" --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build --workspace "${WORKSPACE}"

cat <<EOF
[setup] Complete.

Use in each terminal:
  source /opt/ros/noetic/setup.bash
  source ${WORKSPACE}/devel/setup.bash
EOF
