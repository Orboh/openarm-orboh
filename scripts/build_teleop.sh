#!/usr/bin/env bash
# openarm_teleop を CMake ビルド（Ubuntu 24.04 / ROS 2 Jazzy の kdl_parser を利用）。リポ相対。
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
SRC="$REPO/openarm_teleop"
BUILD="$SRC/build"

[ -f "$SRC/CMakeLists.txt" ] || { echo "[ERROR] $SRC が空。先に: git submodule update --init --recursive"; exit 1; }

# conda が混ざると CMake が Python を誤検出する。明示的に外す。
conda deactivate 2>/dev/null || true

# ROS の setup.bash は未定義変数を参照するため nounset を一時解除して source
set +u
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
set -u

echo "=== CMake configure ==="
cmake -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
echo "=== Build ==="
cmake --build "$BUILD" -j"$(nproc)"
echo "=== 生成バイナリ ==="
ls -la "$BUILD" | grep -E "unilateral_control|bilateral_control|gravity_comp|comm_test" || true
