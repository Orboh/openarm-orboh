#!/usr/bin/env bash
# 北九州 タケエイ 木材 L1（両腕・木材を拾って箱に入れる）の GR00T rollout を、学習時と同じ条件で起動する。
# rollout_bi_leftarm.sh を写し、runbook §6.1 のタスクごとの 6 項目だけを差し替えた。
#
#   bash scripts/g4/rollout_bi_wood_l1.sh server          # 端末 A（tmux policy 内）: PolicyServer を起動
#   bash scripts/g4/rollout_bi_wood_l1.sh dry  <tag>      # 端末 B: 指令を送らない確認（接続で両腕が整列する）
#   bash scripts/g4/rollout_bi_wood_l1.sh run  <tag>      # 端末 B: 実駆動 1 本
#
# 学習と揃えるもの（ここを 1 文字でも変えると条件が割れる）:
#   - 指示文   : データセットの single_task と一字一句同じ（LANG）
#   - 開始姿勢 : 開始姿勢セット kitakyushu-wood-l1-v1（config/pose_sets/・経由地つき）。
#                モデル・較正・指示文もセットと照合される（run_rollout.sh の POSE_SET guard）
#   - カメラ   : chest / wrist_right / wrist_left の 3 眼・640x480@30（modality.json と同じ順）
#   - gripper  : 収集と同じ kp 12（開いたまま保持すると過温ラッチ 0xC が立つ。v1 rollout で 2 回）
#   - カメラ   : 両腕の整列が終わってから開く（右腕の整列で右 wrist のケーブルが引かれ一時的に
#                USB から抜けるため。2026-09-24 に整列 8 回中 5 回。腕が止まると 10 秒前後で戻る）
#   - モデル   : Kota0612/gr00t-bi-takeei-wood-l1-v1（2026-10-23 学習・10000 step。途中の checkpoint は
#                s3://orboh-models/gr00t-ckpt/ft_bi_takeei_wood_l1_v1/。MODEL= で差し替える）
# 設定の出典: 2026-10-20〜21 の収集データ Orboh/bi_openarm_takeei_wood_l1_chest_wristr_wristl_v1_20261020_171837
#   （39 本・13062 frame）から 2026-10-24 に算出:
#   - max_relative_target = action の 1 step 変化量 |a[t+1]-a[t]| の P99.9（関節別・腕別）[deg]
#   - max_steps = エピソード長の P75（386 frame ≒ 12.9 s。P50 325・最長 508）
#   P99.9 が小さい関節（左 J2 0.7・右 J2/J3 1.2）は hold が重力で垂れるおそれがある（runbook §6 の 6）。垂れたら小刻みに上げる。
set -euo pipefail
MODE="${1:-}"; TAG="${2:-}"
REPO="$HOME/projects/openarm-orboh"
export POSE_SET=kitakyushu-wood-l1-v1
source "$REPO/scripts/lib/pose_set.sh"
pose_set_load required || exit 1
MODEL="${MODEL:-$PS_MODEL_DIR}"
LANG_TEXT="Pick up the wood and place it in the box"
export OPENARM_EMBODIMENT=bimanual
export OPENARM_CAMERAS=chest,wrist_right,wrist_left

case "$MODE" in
  server)
    [ -f "$MODEL/config.json" ] || { echo "モデルが無い: $MODEL" >&2; exit 1; }
    export HF_TOKEN="$(cat "$HOME/.cache/huggingface/token")"
    export HF_HOME="$HOME/.cache/huggingface"
    R="$HOME/projects/Isaac-GR00T-n1.7"
    # Thor の ~/.bashrc は LD_LIBRARY_PATH の先頭に /usr/local/cuda/lib64 を足す。対話端末から起動すると
    # libcublas は venv 同梱（nvidia/cu13）、libcublasLt だけシステムの CUDA 13.0 が読まれて版がずれ、
    # 最初の推論で CUBLAS_STATUS_NOT_INITIALIZED になる（2026-09-24 01:12 に実際に発生）。
    # `tmux new-session -d "..."` は .bashrc を読まないので、以前の起動では表面化しなかった。
    # venv 同梱の 2 つで揃えるため、サーバだけシステム CUDA を読み込み経路から外す。
    unset LD_LIBRARY_PATH
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
DRY=false; [ "$MODE" = dry ] && DRY=true
KP="[240,240,240,240,24,31,25,12]"
CAMS='{"chest": {"type":"intelrealsense","serial_number_or_name":"344422070898","width":640,"height":480,"fps":30}, "wrist_right": {"type":"intelrealsense","serial_number_or_name":"352122273417","width":640,"height":480,"fps":30}, "wrist_left": {"type":"intelrealsense","serial_number_or_name":"409122272149","width":640,"height":480,"fps":30}}'

echo "[rollout-bi-wood-l1] mode=$MODE tag=$TAG model=$MODEL"
echo "[rollout-bi-wood-l1] lang='$LANG_TEXT'"
echo "[rollout-bi-wood-l1] 開始姿勢セット=$POSE_SET（経由地つき）"
echo "[rollout-bi-wood-l1] 接続すると両腕が自走して開始姿勢へ整列する。両腕と机から手を引くこと"

MODEL="$MODEL" exec bash "$REPO/scripts/g4/run_rollout.sh" "bi-wood-l1-$TAG" -- \
  --robot.type=bi_openarm_follower \
  --robot.left_arm_config.port=can_fol_l --robot.left_arm_config.side=left \
  --robot.right_arm_config.port=can_fol_r --robot.right_arm_config.side=right \
  --robot.id=follower \
  --robot.cameras="$CAMS" \
  --robot.left_arm_config.position_kp="$KP" \
  --robot.right_arm_config.position_kp="$KP" \
  --robot.right_arm_config.max_relative_target='{joint_1: 2.6, joint_2: 1.2, joint_3: 1.2, joint_4: 3.2, joint_5: 2.7, joint_6: 3.1, joint_7: 4.0, gripper: 7.4}' \
  --robot.left_arm_config.max_relative_target='{joint_1: 3.3, joint_2: 0.7, joint_3: 1.4, joint_4: 3.6, joint_5: 2.7, joint_6: 2.8, joint_7: 4.8, gripper: 7.8}' \
  --lang_instruction="$LANG_TEXT" \
  --policy_host=127.0.0.1 --policy_port=5555 \
  --dry_run=$DRY --fps=30 --max_steps=386 --action_horizon=16 --log_obs_every_step=true \
  --defer_camera_connect=true --camera_wait_s=30 \
  --save_wrist_frames=true
