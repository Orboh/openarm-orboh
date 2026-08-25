#!/usr/bin/env bash
# 別室テレオペ（操作側）: 左腕リーダーを 200Hz で読み、関節角を follower ホスト（Thor）へ ZMQ 送信する。
# 右腕版（run_leader_agent_right.sh）の左腕版。ポートは 左=5702/echo 5712（右=5701/5711 と共存＝両腕同時可）。
# 重力補償はこの PC 内で完結。設計書= toyota-body-orboh vault "openarm-remote-teleop-design"。
# 停止は Ctrl-C（リーダーは脱力する＝先に腕を下ろす）。
# 使い方: THOR_IP=<followerホストのIP> bash scripts/run_leader_agent_left.sh [scale]
# scale 既定 1.0（右と同じ統制値。旧 run_teleop_left.sh の既定 1.2 は過補償の経緯があり採らない）
SCALE="${1:-1.0}"
: "${THOR_IP:?THOR_IP=<followerホストのIP> を指定してください}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
URDF="$REPO/urdf/openarm_v10_bimanual.urdf"
PY="$HOME/projects/lerobot/.venv/bin/python"
# 開始姿勢: キャプチャ済み(scripts/capture_initial_pose.sh left)なら使う。無ければ公式既定(肘36°)。
POSE="$REPO/config/initial_pose_left.yaml"
POSE_ARGS=()
[ -f "$POSE" ] && POSE_ARGS+=(--teleop.initial_pose_path="$POSE")
"$PY" -m lerobot.teleoperators.openarm_leader_remote.leader_agent \
  --remote_host="$THOR_IP" --remote_port=5702 --echo_port=5712 \
  --arm=left --fps=200 \
  --teleop.port=can_lead_l --teleop.id=leader_left \
  --teleop.gravity_compensation=true \
  --teleop.gravity_urdf_path="$URDF" \
  --teleop.gravity_side=left \
  --teleop.gravity_scale="$SCALE" \
  --teleop.use_velocity_and_torque=false \
  "${POSE_ARGS[@]}"
