#!/usr/bin/env bash
# Record a single-arm pick-and-place dataset for OpenArm with lerobot.
#
# LEFT arm (Follower-L driven by Leader-L). CAN uses the channel_id fixed names
# (can_fol_l / can_lead_l) assigned by scripts/canup.sh or the installed udev
# rule (USB-port / PC independent -- PR #1). The `[damiao]` lerobot extra must
# be installed.
#
# Camera: D435i (chest) via RealSense native, addressed by serial number.
# Depth is NOT recorded (use_depth default False) -- ACT observation is RGB only.
#
# Prereqs:
#   1. CAN up as fixed names (udev auto on plug/reboot, or: sudo bash scripts/canup.sh)
#   2. lerobot-calibrate follower & leader once with ids: follower_left / leader_left
#
# Usage:
#   bash scripts/record_pick_place_openarm.sh
#   NUM_EPISODES=2 DISPLAY_DATA=false bash scripts/record_pick_place_openarm.sh   # dry run
set -euo pipefail

# ---- Arm pair (channel_id fixed CAN names -- USB/PC independent) -------------
SIDE="${SIDE:-left}"                        # left | right
case "$SIDE" in
  left)  FOLLOWER_PORT=can_fol_l; LEADER_PORT=can_lead_l ;;
  right) FOLLOWER_PORT=can_fol_r; LEADER_PORT=can_lead_r ;;
  *) echo "[record] SIDE must be 'left' or 'right', got '$SIDE'" >&2; exit 1 ;;
esac

# ---- Camera: D435i (chest), RealSense native, by serial ----------------------
CAM_SERIAL="${CAM_SERIAL:-344422070898}"    # D435i (chest)
CAMERA_W="${CAMERA_W:-640}"
CAMERA_H="${CAMERA_H:-480}"
FPS="${FPS:-30}"

# ---- Dataset tunables --------------------------------------------------------
NUM_EPISODES="${NUM_EPISODES:-50}"
EPISODE_TIME_S="${EPISODE_TIME_S:-20}"
RESET_TIME_S="${RESET_TIME_S:-10}"
REPO_ID="${REPO_ID:-Orboh/openarm_okra_chest_v1}"
TASK="${TASK:-Pick up the okra and place it in the bin}"
DISPLAY_DATA="${DISPLAY_DATA:-true}"        # set false for headless dry-run

echo "[record] side=$SIDE  follower=$FOLLOWER_PORT  leader=$LEADER_PORT"
echo "[record] camera=chest(D435i $CAM_SERIAL) ${CAMERA_W}x${CAMERA_H}@${FPS}fps  episodes=$NUM_EPISODES"

lerobot-record \
  --robot.type=openarm_follower \
  --robot.port="$FOLLOWER_PORT" \
  --robot.side="$SIDE" \
  --robot.id="follower_${SIDE}" \
  --robot.cameras="{ chest: {type: intelrealsense, serial_number_or_name: '$CAM_SERIAL', width: $CAMERA_W, height: $CAMERA_H, fps: $FPS}}" \
  --teleop.type=openarm_leader \
  --teleop.port="$LEADER_PORT" \
  --teleop.id="leader_${SIDE}" \
  --display_data="$DISPLAY_DATA" \
  --dataset.repo_id="$REPO_ID" \
  --dataset.single_task="$TASK" \
  --dataset.fps="$FPS" \
  --dataset.num_episodes="$NUM_EPISODES" \
  --dataset.episode_time_s="$EPISODE_TIME_S" \
  --dataset.reset_time_s="$RESET_TIME_S" \
  --dataset.push_to_hub=false
