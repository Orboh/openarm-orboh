#!/usr/bin/env bash
# OpenArm teleop 依存インストール（Ubuntu 24.04 noble / teleop 最小構成）
#   build tools + KDL/Eigen/urdfdom/yaml-cpp/can-utils + OpenArm PPA + ROS Jazzy の kdl_parser/ament（フルROSは入れない）
# 実行: sudo は対話入力。setup.sh から呼ばれる。単体なら: bash scripts/install_deps.sh
set -euo pipefail
log() { printf '\n\033[1;36m=== %s ===\033[0m\n' "$*"; }

log "P1: Ubuntu 標準依存（build tools + KDL/Eigen/urdfdom/yaml-cpp/can-utils）"
sudo apt update
sudo apt install -y \
  build-essential cmake git pkg-config \
  libeigen3-dev liburdfdom-dev liburdfdom-headers-dev \
  libyaml-cpp-dev liborocos-kdl-dev can-utils ethtool \
  software-properties-common curl

log "P2: OpenArm PPA + openarm-can-utils"
sudo add-apt-repository -y ppa:openarm/main
sudo apt update
sudo apt install -y libopenarm-can-dev openarm-can-utils

log "P3: ROS 2 apt source + kdl_parser + ament_cmake のみ（フル ROS は入れない）"
# Ubuntu 24.04(noble)=Jazzy / 22.04(jammy)=Humble のどちらでも kdl_parser が使えればよい
sudo add-apt-repository -y universe
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F'"' '{print $4}')
CODENAME=$(. /etc/os-release && echo "$VERSION_CODENAME")
case "$CODENAME" in
  noble) ROS_DISTRO=jazzy ;;
  jammy) ROS_DISTRO=humble ;;
  *) echo "[ERROR] 未対応の codename: $CODENAME（jazzy/humble 以外）"; exit 1 ;;
esac
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y "ros-${ROS_DISTRO}-kdl-parser" "ros-${ROS_DISTRO}-ament-cmake"

log "検証"
echo -n "openarm-can-cli: "; command -v openarm-can-cli || echo "NOT FOUND"
echo -n "cansend:         "; command -v cansend || echo "NOT FOUND"
echo -n "ethtool:         "; command -v ethtool || echo "NOT FOUND"
echo -n "kdl_parser:      "; dpkg -l "ros-${ROS_DISTRO}-kdl-parser" 2>/dev/null | awk '/^ii/{print $3}' || echo "NOT FOUND"
echo -n "ament_cmake:     "; dpkg -l "ros-${ROS_DISTRO}-ament-cmake" 2>/dev/null | awk '/^ii/{print $3}' || echo "NOT FOUND"
echo -n "libopenarm-can:  "; dpkg -l libopenarm-can-dev 2>/dev/null | awk '/^ii/{print $3}' || echo "NOT FOUND"
log "完了"
