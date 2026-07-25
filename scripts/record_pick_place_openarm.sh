#!/usr/bin/env bash
# Record a single-arm pick-and-place dataset for OpenArm with lerobot.
#
# LEFT arm (Follower-L driven by Leader-L). CAN uses the channel_id fixed names
# (can_fol_l / can_lead_l) assigned by scripts/canup.sh or the installed udev
# rule (USB-port / PC independent -- PR #1). The `[damiao]` lerobot extra must
# be installed.
#
# Cameras: D435i (chest) always, via RealSense native, addressed by serial.
#   Optional D405 wrist cam: set WRIST=true (adds wrist_<SIDE>). NOTE: adding a
#   camera changes the input dims => a DIFFERENT model; monocular (chest-only)
#   data is NOT reusable with a multi-cam model. WRIST=true writes to a separate
#   dataset repo_id so the two are never mixed.
# Depth is NOT recorded (use_depth default False) -- ACT observation is RGB only.
#
# Prereqs:
#   1. CAN up as fixed names (udev auto on plug/reboot, or: sudo bash scripts/canup.sh)
#   2. 恒久ゼロ較正: bash scripts/calibrate_openarm.sh <left|right>
#      (follower/leader 両方の homing offset を保存。以後 connect() は毎セッションの
#      零位焼きをスキップし、開始基準がセッション・電源断をまたいで固定される。
#      モータ側零位を外部ツールで焼き直したら再較正すること)
#
# Usage:
#   bash scripts/record_pick_place_openarm.sh                                     # chest-only (monocular baseline)
#   WRIST=true bash scripts/record_pick_place_openarm.sh                          # chest + D405 wrist (2-cam, separate dataset)
#   NUM_EPISODES=2 DISPLAY_DATA=false bash scripts/record_pick_place_openarm.sh   # dry run
set -euo pipefail

# ---- Arm pair (channel_id fixed CAN names -- USB/PC independent) -------------
SIDE="${SIDE:-left}"                        # left | right
case "$SIDE" in
  left)  FOLLOWER_PORT=can_fol_l; LEADER_PORT=can_lead_l ;;
  right) FOLLOWER_PORT=can_fol_r; LEADER_PORT=can_lead_r ;;
  *) echo "[record] SIDE must be 'left' or 'right', got '$SIDE'" >&2; exit 1 ;;
esac

# ---- Cameras: D435i (chest) always; optional D405 wrist (WRIST=true) ----------
CAM_SERIAL="${CAM_SERIAL:-344422070898}"      # D435i (chest)
WRIST="${WRIST:-false}"                       # true => add D405 wrist cam (different model! separate dataset)
WRIST_SERIAL="${WRIST_SERIAL:-260322272302}"  # D405 (wrist) -- verified streaming 2026-06-19
CAMERA_W="${CAMERA_W:-640}"
CAMERA_H="${CAMERA_H:-480}"
FPS="${FPS:-30}"                            # = dataset.fps AND teleop loop rate (record forces them equal); camera-capped (~<=60 @640x480 RGB). FPS=60 reduces follower jitter

# Build the cameras dict: chest always; append wrist_<SIDE> only when WRIST=true
CAMERAS="{ chest: {type: intelrealsense, serial_number_or_name: '$CAM_SERIAL', width: $CAMERA_W, height: $CAMERA_H, fps: $FPS}"
if [ "$WRIST" = "true" ]; then
  CAMERAS="$CAMERAS, wrist_${SIDE}: {type: intelrealsense, serial_number_or_name: '$WRIST_SERIAL', width: $CAMERA_W, height: $CAMERA_H, fps: $FPS}"
fi
CAMERAS="$CAMERAS }"

# ---- Leader gravity compensation (light leader for comfortable collection) ---
# Inject G(q) feed-forward on the leader (joints 1-7; gripper excluded) so it can
# be moved by hand. Independent of FPS. Requires Thor's lerobot venv on the
# feat/gravity-comp branch with pinocchio installed. scale 1.2 validated 2026-06-17.
GRAVITY_COMP="${GRAVITY_COMP:-true}"
GRAVITY_SCALE="${GRAVITY_SCALE:-1.2}"
GRAVITY_URDF="${GRAVITY_URDF:-$HOME/openarm-orboh/urdf/openarm_v10_bimanual.urdf}"

# ---- Dataset tunables --------------------------------------------------------
NUM_EPISODES="${NUM_EPISODES:-50}"
EPISODE_TIME_S="${EPISODE_TIME_S:-20}"
RESET_TIME_S="${RESET_TIME_S:-10}"
# Dataset name encodes the camera set so monocular and multi-cam never mix (plan: openarm_<task>_<cams>_vN)
if [ "$WRIST" = "true" ]; then
  REPO_ID="${REPO_ID:-Orboh/openarm_okra_chest_wrist_v1}"
else
  REPO_ID="${REPO_ID:-Orboh/openarm_okra_chest_v1}"
fi
TASK="${TASK:-Pick up the okra and place it in the bin}"
DISPLAY_DATA="${DISPLAY_DATA:-true}"        # set false for headless dry-run

# ---- Start pose alignment (captured "ready" pose, optional) ------------------
# キャプチャ済み(scripts/capture_initial_pose.sh $SIDE)なら follower/leader の
# 整列先に使う。未キャプチャなら公式既定(肘36°)にフォールバック。
#
# 開始姿勢の固定はこう働く:
#   - セッション開始: robot/teleop の connect() 時に align_on_connect が発火し、
#     両腕がこの ready 姿勢へ自動整列する(= セッション開始姿勢は自動で固定)。
#     グリッパは常に閉鎖ゼロへピン留め。
#   - エピソード間: lerobot-record のリセットは reset_time_s の手動ウィンドウ
#     (teleop 運転)。leader を手でこの ready 姿勢に戻してから次エピソードを
#     開始する。戻し忘れを防ぐため下の開始姿勢ゲートが判定し、許容外なら
#     録り始めない(推論側 run_rollout.sh は reset_to_initial_position で
#     完全自動復帰。record は teleop 運転中で人の手が leader に載るため、
#     無断で腕を動かさずゲート+キー押しアラインにしてある)。
#   - 収集セッションを跨ぐ場合も同じ ready YAML を使い続ける(較正=恒久ゼロ +
#     同一 YAML で、セッション間の座標系ずれ・開始姿勢ずれの両方を防ぐ)。
REPO="$(cd "$(dirname "$0")/.." && pwd)"
POSE="$REPO/config/initial_pose_${SIDE}.yaml"
POSE_ARGS=()
[ -f "$POSE" ] && POSE_ARGS+=(--robot.initial_pose_path="$POSE" --teleop.initial_pose_path="$POSE")

# ---- Per-episode start-pose gate (収集データ品質) ----------------------------
# 接続直後の姿勢(= align 済みの ready 姿勢)を基準に、次エピソードを始める前に
# 全関節の逸脱を判定する。許容外なら録り始めず、ターミナルに逸脱関節を表示:
#   a キー   -> leader を ready 姿勢へ自動で戻す(follower は teleop で追従)
#   右矢印   -> そのまま録る(逸脱量はログに残る)
#   esc      -> 収集を止める
# 既定 8° の根拠: 2026-07-20 収集の実測ばらつきは J4 で range 27.3°/std 9.7°、
# J1/J6/J7 も可動域の 2 割前後。8° なら大外れは弾き、手で戻せる範囲には収まる。
# START_POSE_TOL_DEG="" で無効化(従来の手動リセットのみ)。
# ":-" ではなく "-" を使う: 前者は明示的な空文字も既定値で上書きしてしまい、
# 無効化できなくなる。
START_POSE_TOL_DEG="${START_POSE_TOL_DEG-8}"
START_POSE_MAX_WAIT_S="${START_POSE_MAX_WAIT_S:-120}"
START_POSE_LOG="${START_POSE_LOG:-$REPO/data-collection-logs/start-pose-$(date +%Y%m%d-%H%M%S)-${SIDE}.jsonl}"
GATE_ARGS=()
if [ -n "$START_POSE_TOL_DEG" ]; then
  GATE_ARGS+=(
    --start_pose_tolerance_deg="$START_POSE_TOL_DEG"
    --start_pose_max_wait_s="$START_POSE_MAX_WAIT_S"
    --start_pose_log_path="$START_POSE_LOG"
  )
fi

echo "[record] side=$SIDE  follower=$FOLLOWER_PORT  leader=$LEADER_PORT"
echo "[record] cameras=$CAMERAS"
echo "[record] wrist=$WRIST  repo_id=$REPO_ID  episodes=$NUM_EPISODES  ${CAMERA_W}x${CAMERA_H}@${FPS}fps"
echo "[record] gravity_comp=$GRAVITY_COMP scale=$GRAVITY_SCALE side=$SIDE urdf=$GRAVITY_URDF"
if [ -n "$START_POSE_TOL_DEG" ]; then
  echo "[record] start-pose gate: tolerance=${START_POSE_TOL_DEG}deg  max_wait=${START_POSE_MAX_WAIT_S}s  log=$START_POSE_LOG"
  echo "[record]   keys while the gate waits: a = align leader to ready pose, right arrow = record anyway, esc = stop"
else
  echo "[record] start-pose gate: disabled (manual reset only)"
fi

lerobot-record \
  --robot.type=openarm_follower \
  --robot.port="$FOLLOWER_PORT" \
  --robot.side="$SIDE" \
  --robot.id="follower_${SIDE}" \
  --robot.cameras="$CAMERAS" \
  --teleop.type=openarm_leader \
  --teleop.port="$LEADER_PORT" \
  --teleop.id="leader_${SIDE}" \
  --teleop.gravity_compensation="$GRAVITY_COMP" \
  --teleop.gravity_urdf_path="$GRAVITY_URDF" \
  --teleop.gravity_side="$SIDE" \
  --teleop.gravity_scale="$GRAVITY_SCALE" \
  --display_data="$DISPLAY_DATA" \
  --dataset.repo_id="$REPO_ID" \
  --dataset.single_task="$TASK" \
  --dataset.fps="$FPS" \
  --dataset.num_episodes="$NUM_EPISODES" \
  --dataset.episode_time_s="$EPISODE_TIME_S" \
  --dataset.reset_time_s="$RESET_TIME_S" \
  --dataset.push_to_hub=false \
  "${POSE_ARGS[@]}" \
  "${GATE_ARGS[@]}"
