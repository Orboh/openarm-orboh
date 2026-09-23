#!/usr/bin/env bash
# 板橋 両腕 v2（右腕をカメラ役に固定・左腕だけで積む）の GR00T rollout を、学習時と同じ条件で起動する。
#
#   bash scripts/g4/rollout_bi_leftarm.sh server          # 端末 A（tmux policy 内）: PolicyServer を起動
#   bash scripts/g4/rollout_bi_leftarm.sh dry  <tag>      # 端末 B: 指令を送らない確認（接続で両腕が整列する）
#   bash scripts/g4/rollout_bi_leftarm.sh run  <tag>      # 端末 B: 実駆動 1 本
#
# 学習と揃えるもの（ここを 1 文字でも変えると条件が割れる）:
#   - 指示文   : データセットの single_task と一字一句同じ（LANG）
#   - 右腕     : 収集と同じカメラ役の姿勢 config/initial_pose_right_rightcam.yaml（経由地つき）。
#                run_rollout.sh は明示しないと v1 の initial_pose_right.yaml を自動で付けるので必ず渡す
#   - カメラ   : chest / wrist_right / wrist_left の 3 眼・640x480@30（modality.json と同じ順）
#   - gripper  : 収集と同じ kp 12（開いたまま保持すると過温ラッチ 0xC が立つ。v1 rollout で 2 回）
# 設定の出典: 2026-09-23 の収集（引き継ぎ §3）と v1 rollout R10 の max_relative_target（台帳「実行変数の変更履歴」）。
set -euo pipefail
MODE="${1:-}"; TAG="${2:-}"
REPO="$HOME/projects/openarm-orboh"
MODEL="${MODEL:-$HOME/models/gr00t-bi-blockstack-leftarm-v1}"
LANG_TEXT="Stack the three wooden blocks with the left arm while the right arm holds the camera steady"
export OPENARM_EMBODIMENT=bimanual
export OPENARM_CAMERAS=chest,wrist_right,wrist_left

case "$MODE" in
  server)
    [ -f "$MODEL/config.json" ] || { echo "モデルが無い: $MODEL" >&2; exit 1; }
    export HF_TOKEN="$(cat "$HOME/.cache/huggingface/token")"
    export HF_HOME="$HOME/.cache/huggingface"
    R="$HOME/projects/Isaac-GR00T-n1.7"
    cd "$R"
    echo "[serve] model=$MODEL embodiment=$OPENARM_EMBODIMENT cameras=$OPENARM_CAMERAS"
    exec "$R/.venv/bin/python" gr00t/eval/run_gr00t_server.py \
      --model-path "$MODEL" --embodiment-tag NEW_EMBODIMENT \
      --modality-config-path "$R/openarm_embodiment/openarm_config.py" \
      --host 127.0.0.1 --port 5555
    ;;
  dry|run) ;;
  *) echo "usage: $0 <server|dry|run> [tag]" >&2; exit 1 ;;
esac

[ -n "$TAG" ] || TAG="$MODE"
RPOSE="$REPO/config/initial_pose_right_rightcam.yaml"
LPOSE="$REPO/config/initial_pose_left.yaml"
for f in "$RPOSE" "$LPOSE"; do [ -f "$f" ] || { echo "開始姿勢が無い: $f" >&2; exit 1; }; done
DRY=false; [ "$MODE" = dry ] && DRY=true
KP="[240,240,240,240,24,31,25,12]"
CAMS='{"chest": {"type":"intelrealsense","serial_number_or_name":"344422070898","width":640,"height":480,"fps":30}, "wrist_right": {"type":"intelrealsense","serial_number_or_name":"352122273417","width":640,"height":480,"fps":30}, "wrist_left": {"type":"intelrealsense","serial_number_or_name":"409122272149","width":640,"height":480,"fps":30}}'

echo "[rollout-bi-leftarm] mode=$MODE tag=$TAG model=$MODEL"
echo "[rollout-bi-leftarm] lang='$LANG_TEXT'"
echo "[rollout-bi-leftarm] 右腕の開始姿勢=$RPOSE（カメラ役・経由地つき）"
echo "[rollout-bi-leftarm] 接続すると両腕が自走して開始姿勢へ整列する。両腕と机から手を引くこと"

MODEL="$MODEL" exec bash "$REPO/scripts/g4/run_rollout.sh" "bi-blockstack-leftarm-$TAG" -- \
  --robot.type=bi_openarm_follower \
  --robot.left_arm_config.port=can_fol_l --robot.left_arm_config.side=left \
  --robot.right_arm_config.port=can_fol_r --robot.right_arm_config.side=right \
  --robot.id=follower \
  --robot.cameras="$CAMS" \
  --robot.left_arm_config.initial_pose_path="$LPOSE" \
  --robot.right_arm_config.initial_pose_path="$RPOSE" \
  --robot.left_arm_config.position_kp="$KP" \
  --robot.right_arm_config.position_kp="$KP" \
  --robot.right_arm_config.max_relative_target='{joint_1: 2.8, joint_2: 2.8, joint_3: 2.0, joint_4: 3.5, joint_5: 4.2, joint_6: 3.2, joint_7: 5.5, gripper: 20.0}' \
  --robot.left_arm_config.max_relative_target='{joint_1: 3.4, joint_2: 1.8, joint_3: 1.4, joint_4: 2.9, joint_5: 2.9, joint_6: 2.6, joint_7: 3.9, gripper: 20.0}' \
  --lang_instruction="$LANG_TEXT" \
  --policy_host=127.0.0.1 --policy_port=5555 \
  --dry_run=$DRY --fps=30 --max_steps=1500 --action_horizon=16 --log_obs_every_step=true
