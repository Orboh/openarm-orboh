#!/usr/bin/env bash
# Record a single-arm pick-and-place dataset for OpenArm with lerobot.
#
# OpenArm differs from SO-ARM101: it talks CAN-FD (not USB serial) to Damiao
# motors, so lerobot needs --robot.port=<canX> and --robot.side, and the
# `[damiao]` extra installed (`pip install -e ".[damiao]"`).
#
# This script targets the RIGHT arm (Follower-R driven by Leader-R). CAN
# interface indices are NOT hardcoded: they are resolved at runtime from the
# role map that scripts/openarm_canup.sh writes, because the kernel canN index
# depends on USB enumeration order (see SETUP_LOG.md, Phase 3).
#
# Prereqs:
#   1. sudo ./scripts/openarm_canup.sh        # brings up CAN + writes can_map.env
#   2. lerobot-calibrate the follower & leader once (same --robot.id / --teleop.id)
#   3. Identify the camera index:  ls /dev/video*  /  v4l2-ctl --list-devices
#
# Usage:
#   ./scripts/record_pick_place_openarm.sh
#   CAMERA_DEV=/dev/video2 NUM_EPISODES=60 ./scripts/record_pick_place_openarm.sh
set -euo pipefail

# ---- CAN role resolution -----------------------------------------------------
CAN_MAP_ENV="${CAN_MAP_ENV:-$HOME/.config/openarm/can_map.env}"
if [[ ! -f "$CAN_MAP_ENV" ]]; then
  echo "[record] CAN role map not found: $CAN_MAP_ENV" >&2
  echo "[record] Run 'sudo ./scripts/openarm_canup.sh' first to bring up CAN." >&2
  exit 1
fi
# shellcheck source=/dev/null
source "$CAN_MAP_ENV"

# Which arm pair to use. Default = left, because the right follower's joint_6
# (wrist pitch) is currently non-responsive on this rig (2026-06-04). The left
# follower + left leader are fully healthy (all 8 motors).
SIDE="${SIDE:-left}"                       # left | right
case "$SIDE" in
  left)  SUF=L ;;
  right) SUF=R ;;
  *) echo "[record] SIDE must be 'left' or 'right', got '$SIDE'" >&2; exit 1 ;;
esac
FOLLOWER_VAR="OPENARM_FOLLOWER_${SUF}"     # e.g. OPENARM_FOLLOWER_L -> can3
LEADER_VAR="OPENARM_LEADER_${SUF}"         # e.g. OPENARM_LEADER_L   -> can1
FOLLOWER_PORT="${!FOLLOWER_VAR:-}"
LEADER_PORT="${!LEADER_VAR:-}"
if [[ -z "$FOLLOWER_PORT" || -z "$LEADER_PORT" ]]; then
  echo "[record] $SIDE-arm CAN roles unresolved ($FOLLOWER_VAR / $LEADER_VAR)." >&2
  echo "[record] Run 'sudo ./scripts/openarm_canup.sh' then 'source ~/.config/openarm/can_map.env'." >&2
  exit 1
fi

# ---- Tunable parameters (no control gains here — those live in the robot cfg) -
# Tenveo FHD workspace cam, addressed by its STABLE by-id path so it survives
# /dev/videoN renumbering on USB replug (the index0 node is the capture node).
# NOTE: close any camera app (e.g. Cheese/pipewire client) first — it holds the
# device and OpenCV will fail with "can't open camera".
CAMERA_DEV="${CAMERA_DEV:-/dev/v4l/by-id/usb-Tenveo_FHD_Cam_Tenveo_FHD_Cam_2399_2093_IQ30_sdk3.2.2-video-index0}"
CAMERA_W="${CAMERA_W:-640}"               # [px]
CAMERA_H="${CAMERA_H:-480}"               # [px]
FPS="${FPS:-30}"                          # [frames/s] control + capture rate
FOURCC="${FOURCC:-MJPG}"                   # Tenveo only does 30fps in MJPG (YUYV caps at 25fps @640x480)
NUM_EPISODES="${NUM_EPISODES:-50}"        # >=50 recommended for ACT pick-and-place
EPISODE_TIME_S="${EPISODE_TIME_S:-20}"    # [s] per-episode recording window
RESET_TIME_S="${RESET_TIME_S:-10}"        # [s] scene reset between episodes
REPO_ID="${REPO_ID:-Orboh/openarm-pick-place-v1}"
TASK="${TASK:-Pick up the object and place it in the bin}"

echo "[record] side=$SIDE  follower=$FOLLOWER_PORT  leader=$LEADER_PORT"
echo "[record] camera=$CAMERA_DEV ${CAMERA_W}x${CAMERA_H}@${FPS}fps  episodes=$NUM_EPISODES"

lerobot-record \
  --robot.type=openarm_follower \
  --robot.port="$FOLLOWER_PORT" \
  --robot.side="$SIDE" \
  --robot.id="openarm_follower_${SUF,,}" \
  --robot.cameras="{ front: {type: opencv, index_or_path: $CAMERA_DEV, width: $CAMERA_W, height: $CAMERA_H, fps: $FPS, fourcc: $FOURCC}}" \
  --teleop.type=openarm_leader \
  --teleop.port="$LEADER_PORT" \
  --teleop.id="openarm_leader_${SUF,,}" \
  --display_data=true \
  --dataset.repo_id="$REPO_ID" \
  --dataset.single_task="$TASK" \
  --dataset.fps="$FPS" \
  --dataset.num_episodes="$NUM_EPISODES" \
  --dataset.episode_time_s="$EPISODE_TIME_S" \
  --dataset.reset_time_s="$RESET_TIME_S" \
  --dataset.push_to_hub=false
