#!/usr/bin/env bash
# openarm_teleop を CMake ビルド（Ubuntu 22.04/ROS Humble・24.04/ROS Jazzy のkdl_parserを利用）。リポ相対。
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
SRC="$REPO/openarm_teleop"
BUILD="$SRC/build"

[ -f "$SRC/CMakeLists.txt" ] || { echo "[ERROR] $SRC が空。先に: git submodule update --init --recursive"; exit 1; }

# conda が混ざると CMake が Python を誤検出する。明示的に外す。
conda deactivate 2>/dev/null || true

# インストール済みの ROS ディストリを検出（jazzy優先、無ければhumble）。
ROS_SETUP=""
for distro in jazzy humble; do
  if [ -f "/opt/ros/$distro/setup.bash" ]; then
    ROS_SETUP="/opt/ros/$distro/setup.bash"
    break
  fi
done
[ -n "$ROS_SETUP" ] || { echo "[ERROR] /opt/ros/{jazzy,humble}/setup.bash が見つからない。先に: bash scripts/install_deps.sh" >&2; exit 1; }
echo "[build_teleop] using $ROS_SETUP"

# ROS の setup.bash は未定義変数を参照するため nounset を一時解除して source
set +u
# shellcheck source=/dev/null
source "$ROS_SETUP"
set -u

echo "=== CMake configure ==="
cmake -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
echo "=== Build ==="
cmake --build "$BUILD" -j"$(nproc)"
echo "=== 生成バイナリ ==="
ls -la "$BUILD" | grep -E "unilateral_control|bilateral_control|gravity_comp|comm_test" || true
