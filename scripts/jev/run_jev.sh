#!/usr/bin/env bash
# Jev 判断層（scripts/jev/jev_arm.py）の実行ラッパー（Thor 上で実行）。
# 1試行 = 1ディレクトリ（~/rollouts/jev/<UTC>-<tag>/）に log / decisions.jsonl / traj.csv / command を集約する。
#
# 使い方:
#   run_jev.sh <tag> -- <jev_arm.py への引数...>
#
# 例1) 腕なし・rules（配線と安全経路の検証。通電しない）
#   scripts/jev/run_jev.sh smoke-rules -- --no_robot=true --decider=rules \
#       --initial_pose_path=$HOME/projects/openarm-orboh/config/initial_pose_right.yaml \
#       --target_offsets_deg='{"joint_4": 10.0}'
#
# 例2) 腕なし・Jev（API キーが届いたら最初にこれ。通電しない）
#   TYPESAFE_API_KEY=... scripts/jev/run_jev.sh smoke-jev -- --no_robot=true --decider=jev \
#       --initial_pose_path=$HOME/projects/openarm-orboh/config/initial_pose_right.yaml \
#       --target_offsets_deg='{"joint_4": 10.0}'
#
# 例3) 実機・右腕・dry-run（通電・自動整列はする。send_action は送らない）
#   scripts/jev/run_jev.sh dryrun-rules -- --decider=rules --dry_run=true \
#       --robot.type=openarm_follower --robot.port=can_fol_r --robot.side=right --robot.id=follower_right \
#       --robot.cameras='{}' --robot.max_relative_target="$JEV_CLAMP" \
#       --target_offsets_deg='{"joint_4": 10.0}'
#
# 例4) 実機・右腕・Jev・実駆動（DC PSU に手を置いて。別端末で `touch /tmp/openarm_estop` で止まる）
#   TYPESAFE_API_KEY=... scripts/jev/run_jev.sh jev-real -- --decider=jev --dry_run=false \
#       --robot.type=openarm_follower --robot.port=can_fol_r --robot.side=right --robot.id=follower_right \
#       --robot.cameras='{}' --robot.max_relative_target="$JEV_CLAMP" \
#       --goal="肘（joint_4）を 10 度曲げる" --target_offsets_deg='{"joint_4": 10.0}'
#
# max_relative_target について（2026-09-20 実機の教訓）:
#   MIT 位置制御ではクランプが「1 tick の移動量」でなく「復元トルク上限 = kp × clamp」として効く。
#   小数 1.0 だと手首 kp25×1°=0.44Nm・肩 kp240×1°=4.2Nm で、開始姿勢の重力負荷（手首≈1.2Nm・肩≈6.5Nm）に負けて
#   hold が垂れる。関節別 dict を使う（8 関節すべて必須・lerobot 側の制約）。JEV_CLAMP は本スクリプトが export する。
set -euo pipefail

VENV="$HOME/projects/lerobot/.venv/bin/python"
# 実機の既定クランプ（関節別 dict）。肩肘 2.0°→8.4Nm・手首 4.0°→1.75Nm・グリッパ 1.0°。詳細は冒頭コメント。
export JEV_CLAMP='{"joint_1":2.0,"joint_2":2.0,"joint_3":2.0,"joint_4":2.0,"joint_5":4.0,"joint_6":4.0,"joint_7":4.0,"gripper":1.0}'

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="$HERE/jev_arm.py"

TAG="${1:-untagged}"; shift || true
if [[ "${1:-}" == "--" ]]; then shift; fi
TAG=$(printf '%s' "$TAG" | tr -c 'A-Za-z0-9-' '-' | sed 's/-\{2,\}/-/g; s/^-//; s/-$//')
[[ -z "$TAG" ]] && TAG="untagged"

# 実機（--no_robot 無し）で開始姿勢 YAML が未指定なら右腕の既定を自動付与（run_rollout.sh と同じ理由）
_side=""; _has_pose=false; _no_robot=false
for _a in "$@"; do
  case "$_a" in
    --robot.side=*)              _side="${_a#*=}" ;;
    --robot.initial_pose_path=*) _has_pose=true ;;
    --no_robot=true)             _no_robot=true ;;
  esac
done
POSE_ARGS=()
if [[ "$_no_robot" == false && "$_has_pose" == false && -n "$_side" ]]; then
  _pose="$HOME/projects/openarm-orboh/config/initial_pose_${_side}.yaml"
  [[ -f "$_pose" ]] && POSE_ARGS+=(--robot.initial_pose_path="$_pose") && echo "[jev] 開始姿勢 YAML を自動付与: $_pose"
fi

RUN_DIR="$HOME/rollouts/jev/$(date -u +%Y%m%dT%H%M%SZ)-$TAG"
mkdir -p "$RUN_DIR"
{
  echo "date_utc: $(date -u +%FT%TZ)"
  echo "host: $(hostname)"
  echo "git: $(git -C "$HERE" rev-parse --short HEAD 2>/dev/null || echo n/a) $(git -C "$HERE" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
  echo "cmd: $VENV $SCRIPT $* ${POSE_ARGS[*]:-} --run_dir=$RUN_DIR"
  echo "TYPESAFE_API_KEY: $([[ -n "${TYPESAFE_API_KEY:-}" ]] && echo set || echo unset)"
} > "$RUN_DIR/command.txt"
echo "[jev] run_dir = $RUN_DIR"

exec "$VENV" "$SCRIPT" "$@" "${POSE_ARGS[@]}" --run_dir="$RUN_DIR" 2>&1 | tee "$RUN_DIR/console.log"
