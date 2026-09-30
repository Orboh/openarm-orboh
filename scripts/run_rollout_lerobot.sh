#!/usr/bin/env bash
# OpenArm 学習済みポリシーの実機推論 (lerobot-rollout)。停止は Ctrl-C / ESC。
#
# ---- 推論ランナーは2本ある。取り違えないこと --------------------------------
#   このファイル scripts/run_rollout_lerobot.sh
#     → lerobot 標準の `lerobot-rollout` を起動。ACT 等の lerobot ポリシー用。
#       開始姿勢固定を配線済み(initial_pose_path・reset_to_initial_position)。
#       run_dir/tee の記録機構は無い。実機未実走。
#   scripts/g4/run_rollout.sh
#     → 自前の `g4/eval_openarm.py` を起動。GR00T(外部 PolicyServer)用。
#       run_dir + tee の記録機構を持ち、実機で実走済み。
#   2026-08-03 に同名 run_rollout.sh 2本の状態を解消してこの名前にした。
# -----------------------------------------------------------------------------
#
# 開始姿勢の固定(収集時と同一化):
#   - robot.connect() 時に align_on_connect(既定 true)が発火し、キャプチャ済み ready 姿勢
#     (config/initial_pose_<side>.yaml、record と同じファイル)へソフト整列する。
#   - lerobot-rollout はその整列「後」に initial_position を記録するため、
#     エピソード間リセット(--strategy.reset_to_initial_position=true)と終了時復帰
#     (--return_to_initial_position、既定 true)の戻り先 = ready 姿勢 = 収集時の開始姿勢になる。
#   - グリッパは resolve_initial_pose が常に閉鎖ゼロへピン留めする(起動時の非ゼロ保持指令が
#     過去の実機グリッパ破損の原因)。
#   - 前提: 恒久ゼロ較正済み(bash scripts/calibrate_openarm.sh <side> [bump_to_stop])。
#     較正が無い/旧式だと connect() が毎回零位を焼き、開始フレームが収集時とずれる。
#
# ハンドオーバ: OpenArm leader は send_feedback 未実装(feedback_features 空)のため、
# leader→follower 引き渡しは使えない。smooth_leader_to_follower_handover=false を明示し、
# リセット時は follower が teleop 現在姿勢へスライドする側の分岐(episodic 実装のフォールバック)を使う。
#
# カメラは record_pick_place_openarm.sh と同一構成(chest D435i、WRIST=true で D405 追加)。
# 学習時と入力カメラ集合が一致しないとポリシーは動かない(visual feature mismatch で起動失敗)。
#
# 使い方:
#   POLICY_PATH=outputs/train/act_okra/checkpoints/last/pretrained_model bash scripts/run_rollout_lerobot.sh
#   SIDE=right POLICY_PATH=... bash scripts/run_rollout_lerobot.sh
#   SIDE=bimanual POLICY_PATH=... bash scripts/run_rollout_lerobot.sh
#   STRATEGY=base POLICY_PATH=... bash scripts/run_rollout_lerobot.sh      # 記録なし連続実行
#   TELEOP=true POLICY_PATH=... bash scripts/run_rollout_lerobot.sh        # リセット手動運転用に leader 接続
set -euo pipefail

SIDE="${SIDE:-left}"                       # left | right | bimanual
STRATEGY="${STRATEGY:-episodic}"           # episodic(記録あり・lerobot-record 相当) | base(記録なし)
POLICY_PATH="${POLICY_PATH:?set POLICY_PATH=<pretrained_model dir or hub id>}"
TELEOP="${TELEOP:-false}"                  # true => leader を接続(リセットフェーズの手動運転)

# ---- Cameras (record_pick_place_openarm.sh と同一) ----------------------------
CAM_SERIAL="${CAM_SERIAL:-344422070898}"      # D435i (chest)
WRIST="${WRIST:-false}"
WRIST_SERIAL="${WRIST_SERIAL:-260322272302}"  # D405 (wrist)
CAMERA_W="${CAMERA_W:-640}"
CAMERA_H="${CAMERA_H:-480}"
FPS="${FPS:-30}"

CAMERAS="{ chest: {type: intelrealsense, serial_number_or_name: '$CAM_SERIAL', width: $CAMERA_W, height: $CAMERA_H, fps: $FPS}"
if [ "$WRIST" = "true" ] && [ "$SIDE" != "bimanual" ]; then
  CAMERAS="$CAMERAS, wrist_${SIDE}: {type: intelrealsense, serial_number_or_name: '$WRIST_SERIAL', width: $CAMERA_W, height: $CAMERA_H, fps: $FPS}"
fi
CAMERAS="$CAMERAS }"

# ---- Dataset (episodic のみ。rollout_ プレフィックス必須) ----------------------
NUM_EPISODES="${NUM_EPISODES:-10}"
EPISODE_TIME_S="${EPISODE_TIME_S:-20}"
RESET_TIME_S="${RESET_TIME_S:-10}"
REPO_ID="${REPO_ID:-Orboh/rollout_openarm_okra_chest_v1}"
TASK="${TASK:-Pick up the okra and place it in the bin}"
DISPLAY_DATA="${DISPLAY_DATA:-true}"

# ---- Leader gravity comp (TELEOP=true のとき) ---------------------------------
GRAVITY_SCALE="${GRAVITY_SCALE:-1.2}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
# 開始姿勢は開始姿勢セット（config/pose_sets/<POSE_SET>/）から付ける。推論なので POSE_SET 必須・較正一致必須
# （モデルは学習データを録った時の開始姿勢・較正と組でしか正しく動かない・2026-09-30）。
source "$REPO/scripts/lib/pose_set.sh"
pose_set_load required || exit 1
pose_set_check_calib strict || exit 1
GRAVITY_URDF="${GRAVITY_URDF:-$REPO/urdf/openarm_v10_bimanual.urdf}"

# ---- Binary resolution (teleop venv → conda lerobot env) ----------------------
BIN=""
for CAND in \
  "$HOME/projects/lerobot/.venv/bin/lerobot-rollout" \
  "$HOME/miniconda3/envs/lerobot/bin/lerobot-rollout"; do
  if [ -x "$CAND" ]; then BIN="$CAND"; break; fi
done
[ -n "$BIN" ] || { echo "[rollout] lerobot-rollout not found" >&2; exit 1; }

# ---- Robot / teleop / pose args per SIDE --------------------------------------
ARGS=()
POSE_ARGS=()
if [ "$SIDE" = "bimanual" ]; then
  ARGS+=(
    --robot.type=bi_openarm_follower
    --robot.left_arm_config.port=can_fol_l --robot.left_arm_config.side=left
    --robot.right_arm_config.port=can_fol_r --robot.right_arm_config.side=right
    --robot.id=follower
    --robot.cameras="$CAMERAS"
  )
  # 収集と同じファイルを指すこと。record 側と同じ env 名で差し替える（2026-09-23）。
  LPOSE="${LEFT_POSE_PATH:-$LPOSE}"
  RPOSE="${RIGHT_POSE_PATH:-$RPOSE}"
  for _v in LEFT_POSE_PATH RIGHT_POSE_PATH; do
    _p="${!_v:-}"
    if [ -n "$_p" ] && [ ! -f "$_p" ]; then
      echo "[rollout] $_v に指定した開始姿勢ファイルが無い: $_p" >&2
      exit 1
    fi
  done
  [ -f "$LPOSE" ] && POSE_ARGS+=(--robot.left_arm_config.initial_pose_path="$LPOSE")
  [ -f "$RPOSE" ] && POSE_ARGS+=(--robot.right_arm_config.initial_pose_path="$RPOSE")
  if [ "$TELEOP" = "true" ]; then
    ARGS+=(
      --teleop.type=bi_openarm_leader
      --teleop.left_arm_config.port=can_lead_l
      --teleop.right_arm_config.port=can_lead_r
      --teleop.id=leader
      --teleop.left_arm_config.gravity_compensation=true
      --teleop.left_arm_config.gravity_urdf_path="$GRAVITY_URDF"
      --teleop.left_arm_config.gravity_side=left
      --teleop.left_arm_config.gravity_scale="$GRAVITY_SCALE"
      --teleop.right_arm_config.gravity_compensation=true
      --teleop.right_arm_config.gravity_urdf_path="$GRAVITY_URDF"
      --teleop.right_arm_config.gravity_side=right
      --teleop.right_arm_config.gravity_scale="$GRAVITY_SCALE"
    )
    [ -f "$LPOSE" ] && POSE_ARGS+=(--teleop.left_arm_config.initial_pose_path="$LPOSE")
    [ -f "$RPOSE" ] && POSE_ARGS+=(--teleop.right_arm_config.initial_pose_path="$RPOSE")
  fi
else
  case "$SIDE" in
    left)  FOLLOWER_PORT=can_fol_l; LEADER_PORT=can_lead_l ;;
    right) FOLLOWER_PORT=can_fol_r; LEADER_PORT=can_lead_r ;;
    *) echo "[rollout] SIDE must be 'left', 'right' or 'bimanual', got '$SIDE'" >&2; exit 1 ;;
  esac
  ARGS+=(
    --robot.type=openarm_follower
    --robot.port="$FOLLOWER_PORT"
    --robot.side="$SIDE"
    --robot.id="follower_${SIDE}"
    --robot.cameras="$CAMERAS"
  )
  if [ "$SIDE" = left ]; then POSE="$LPOSE"; else POSE="$RPOSE"; fi
  [ -f "$POSE" ] && POSE_ARGS+=(--robot.initial_pose_path="$POSE")
  if [ "$TELEOP" = "true" ]; then
    ARGS+=(
      --teleop.type=openarm_leader
      --teleop.port="$LEADER_PORT"
      --teleop.id="leader_${SIDE}"
      --teleop.gravity_compensation=true
      --teleop.gravity_urdf_path="$GRAVITY_URDF"
      --teleop.gravity_side="$SIDE"
      --teleop.gravity_scale="$GRAVITY_SCALE"
    )
    [ -f "$POSE" ] && POSE_ARGS+=(--teleop.initial_pose_path="$POSE")
  fi
fi

# ---- Strategy args -------------------------------------------------------------
# base には dataset/reset フィールドが無い(渡すと draccus がエラー)ので episodic のみ付与。
STRAT_ARGS=(--strategy.type="$STRATEGY")
if [ "$STRATEGY" = "episodic" ]; then
  STRAT_ARGS+=(
    --strategy.reset_to_initial_position=true
    --strategy.smooth_leader_to_follower_handover=false
    --dataset.repo_id="$REPO_ID"
    --dataset.single_task="$TASK"
    --dataset.fps="$FPS"
    --dataset.num_episodes="$NUM_EPISODES"
    --dataset.episode_time_s="$EPISODE_TIME_S"
    --dataset.reset_time_s="$RESET_TIME_S"
    --dataset.push_to_hub=false
  )
fi

echo "[rollout] side=$SIDE strategy=$STRATEGY policy=$POLICY_PATH teleop=$TELEOP"
echo "[rollout] cameras=$CAMERAS"
[ "$STRATEGY" = "episodic" ] && echo "[rollout] repo_id=$REPO_ID episodes=$NUM_EPISODES fps=$FPS"

"$BIN" \
  "${ARGS[@]}" \
  --policy.path="$POLICY_PATH" \
  --fps="$FPS" \
  --task="$TASK" \
  --display_data="$DISPLAY_DATA" \
  --return_to_initial_position=true \
  "${STRAT_ARGS[@]}" \
  "${POSE_ARGS[@]}"
