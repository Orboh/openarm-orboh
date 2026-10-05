#!/usr/bin/env bash
# 両腕 bimanual lerobot teleop（重力補償＋vel FF＋fps200）。停止は Ctrl-C。
# 起動時整列: 各腕 leader→follower が初期姿勢（肘36°）へ自動で動く（公式AdjustPosition移植・各約2.2秒×4・lerobot fork の align_on_connect=true 既定）。
# 使い方: bash scripts/run_teleop_bimanual.sh [left_scale] [right_scale]  （既定 両腕1.0＝2026-09-22 Kento の運用値。旧既定 1.2。左だけ上げる例: ... 1.1 1.0）
# ⚠ 起動時は両腕を真下に垂らす（零位が焼かれる）。右系統CANはbus-off既往。
# NOTE: leader 側は左右とも gravity_side を明示する（未指定だと既定 left＝右腕の重力補償が左モデルで誤計算される。2026-06-23 修正）。
# 前提: lerobot venv は ~/projects/lerobot/.venv（Vector/Thor 共通）。URDF はリポ相対参照。
# Damiao CAN フィードバックの自動記録（2026-10-05・scripts/lib/dmlog.sh）: 起動と同時に受信専用ロガーを裏で起こし、
# テレオペの終了後（Ctrl-C・esc とも、脱力の瞬間まで残して）止めて要約を出す。別端末でロガーを立て直す手間は不要。
# 記録は ~/collections/dmlog-teleop-<MMDD-HHMMSS>.csv（DMLOG_DIR で置き場所を変える）。DMLOG=off で従来どおり
#   （例: DMLOG=off bash scripts/run_teleop_bimanual.sh）。重いときは DMLOG_IFACES="can_fol_l can_fol_r"。
#   解析は既定で異常があったときだけ裏で書く（2026-10-05・DMLOG_REPORT=anomaly|on|off）。テレオペしながら見るときは
#   別端末で bash scripts/dmlog_live.sh、あとからレポートを作るときは bash scripts/dmlog_report.sh（どちらも latest が既定）。
LSCALE="${1:-1.0}"
RSCALE="${2:-$LSCALE}"
# 3 番目以降の引数は lerobot-teleoperate へそのまま渡す（例: グリッパの kp を下げる
#   --robot.left_arm_config.position_kp=[240,240,240,240,24,31,25,10] --robot.right_arm_config.position_kp=[...]
#   2026-09-21: 木材把持で follower gripper がコイル過温ラッチ（kp 25 で握り込み→ストール電流）。kp を下げて飽和トルクを抑える）
EXTRA_ARGS=("${@:3}")
# 終了は esc: トルクを切る前に開始姿勢へ戻り、経由地を逆にたどって下ろした構えまで戻す（経由地つきの
# セットだけ・2026-10-02）。Ctrl-C は即時脱力（非常停止）のまま。戻りの途中で Ctrl-C を押してもその場で
# 脱力する。esc でも戻さないなら RETURN_ON_EXIT=false。
RETURN_ON_EXIT="${RETURN_ON_EXIT:-true}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
URDF="$REPO/urdf/openarm_v10_bimanual.urdf"
BIN="$HOME/projects/lerobot/.venv/bin/lerobot-teleoperate"
# 開始姿勢: 開始姿勢セット（config/pose_sets/<POSE_SET>/・未指定なら CURRENT）の left/right.yaml。
# セットに無い側の腕は公式既定(肘36°)にフォールバック。
source "$REPO/scripts/lib/pose_set.sh"
source "$REPO/scripts/lib/dmlog.sh"
pose_set_load current || exit 1
pose_set_check teleop
POSE_ARGS=()
[ -f "$LPOSE" ] && POSE_ARGS+=(--robot.left_arm_config.initial_pose_path="$LPOSE" --teleop.left_arm_config.initial_pose_path="$LPOSE")
[ -f "$RPOSE" ] && POSE_ARGS+=(--robot.right_arm_config.initial_pose_path="$RPOSE" --teleop.right_arm_config.initial_pose_path="$RPOSE")
dmlog_start "${DMLOG_DIR:-$HOME/collections}/dmlog-teleop-$(date +%m%d-%H%M%S)" "$HOME/projects/lerobot/.venv/bin/python" "" \
  can_fol_l can_fol_r can_lead_l can_lead_r
dmlog_mark "TELEOP_START lscale=$LSCALE rscale=$RSCALE pose_set=$POSE_SET"
"$BIN" \
  --robot.type=bi_openarm_follower \
  --robot.left_arm_config.port=can_fol_l --robot.left_arm_config.side=left \
  --robot.right_arm_config.port=can_fol_r --robot.right_arm_config.side=right \
  --robot.left_arm_config.align_gravity_urdf_path="$URDF" --robot.right_arm_config.align_gravity_urdf_path="$URDF" \
  --robot.left_arm_config.return_on_disconnect="$RETURN_ON_EXIT" --robot.right_arm_config.return_on_disconnect="$RETURN_ON_EXIT" \
  --teleop.left_arm_config.return_on_disconnect="$RETURN_ON_EXIT" --teleop.right_arm_config.return_on_disconnect="$RETURN_ON_EXIT" \
  --robot.id=follower \
  --teleop.type=bi_openarm_leader \
  --teleop.left_arm_config.port=can_lead_l \
  --teleop.right_arm_config.port=can_lead_r \
  --teleop.id=leader \
  --teleop.left_arm_config.gravity_compensation=true \
  --teleop.left_arm_config.gravity_urdf_path="$URDF" \
  --teleop.left_arm_config.gravity_side=left \
  --teleop.left_arm_config.gravity_scale="$LSCALE" \
  --teleop.left_arm_config.use_velocity_and_torque=true \
  --teleop.right_arm_config.gravity_compensation=true \
  --teleop.right_arm_config.gravity_urdf_path="$URDF" \
  --teleop.right_arm_config.gravity_side=right \
  --teleop.right_arm_config.gravity_scale="$RSCALE" \
  --teleop.right_arm_config.use_velocity_and_torque=true \
  --fps=200 \
  "${POSE_ARGS[@]}" \
  ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}
RC=$?
dmlog_stop "rc=$RC"
exit "$RC"
