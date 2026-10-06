#!/usr/bin/env bash
# タケエイ 10/7（両腕・4 品目の片付け）の GR00T rollout を、学習時と同じ条件で起動する。
#
#   bash scripts/g4/rollout_bi_takeei.sh server          # 端末 A（tmux policy 内）: PolicyServer を起動
#   bash scripts/g4/rollout_bi_takeei.sh server-rtc      # 端末 A: RTC 対応の PolicyServer（gr00t_rtc_server.py）を起動。
#                                                        #   options.rtc の無い呼び出しは server と同じ推論なので、比較の間は常にこちらでよい
#   bash scripts/g4/rollout_bi_takeei.sh dry  <tag>      # 端末 B: 指令を送らない確認（接続で両腕が整列する）
#   bash scripts/g4/rollout_bi_takeei.sh run  <tag>      # 端末 B: 実駆動 1 本
#
# 学習と揃えるもの（ここを 1 文字でも変えると条件が割れる）:
#   - 指示文   : データセットの single_task と一字一句同じ（LANG）
#   - 開始姿勢 : 開始姿勢セット takeei-v2（config/pose_sets/・両腕とも経由地つき）。
#                モデル・較正・指示文もセットと照合される（run_rollout.sh の POSE_SET guard）
#   - カメラ   : chest / wrist_right / wrist_left の 3 眼・640x480@30（modality.json と同じ順）
#   - gripper  : 収集と同じ kp 12（開いたまま保持すると過温ラッチ 0xC が立つ。v1 rollout で 2 回）
#   - カメラ   : 両腕の整列が終わってから開く（右腕の整列で右 wrist のケーブルが引かれ一時的に
#                USB から抜けるため。2026-09-24 に整列 8 回中 5 回。腕が止まると 10 秒前後で戻る）
# 推論の方式（2026-10-07・Vault の gr00t-async-inference-rtc-design）。環境変数で選ぶ。既定は従来の直列:
#   INFER_MODE=sync|async（既定 sync）  RTC=true|false（既定 false・true なら server-rtc で起動したサーバが要る）
#   ACTION_HORIZON（sync で 1 回に流す step 数・既定 16。sync で RTC を使うときはチャンク長 16 より小さくする。例 12）
#   RTC_OVERLAP（既定 8）RTC_FROZEN（既定 8。async は推論 220ms の step 数・sync は残りの step をすべて固定）RTC_RAMP（既定 5.0）
#   例: INFER_MODE=async bash scripts/g4/rollout_bi_takeei.sh dry a1
#       INFER_MODE=async RTC=true bash scripts/g4/rollout_bi_takeei.sh run ar1
#       RTC=true ACTION_HORIZON=12 bash scripts/g4/rollout_bi_takeei.sh run sr1
#   方式が sync 以外か RTC のときは、タグの後ろに方式名（-async・-async-rtc・-sync-rtc）を付ける。
# 別のモデル（例: チャンク 32 step 版）を回すとき:
#   端末 A: MODEL=~/models/gr00t-bi-takeei-sort-h32-v1 POLICY_PORT=5556 bash scripts/g4/rollout_bi_takeei.sh server-rtc
#   端末 B: MODEL=~/models/gr00t-bi-takeei-sort-h32-v1 ALLOW_MODEL_MISMATCH=true POLICY_PORT=5556 bash scripts/g4/rollout_bi_takeei.sh run <tag>
#   （開始姿勢セット takeei-v2 に登録したモデルは 16 step 版なので、照合を外す。使ったモデルは試走の記録に残る）
# 設定の出典: 2026-10-06 の収集（Orboh/bi_openarm_takeei_sort_chest_wristr_wristl_v1・50 本）。
#   max_relative_target = 指令の 1 フレーム変化量の P99.9（関節別・腕別。gripper は保持力のため左腕版と同じ 20）。
#   max_steps = エピソード長の P75（1684 step = 56 秒・最長 2238）。GR00T は完了を出せないので、成功したら人が Ctrl-C で止める。
set -euo pipefail
MODE="${1:-}"; TAG="${2:-}"
REPO="$HOME/projects/openarm-orboh"
export POSE_SET=takeei-v2
source "$REPO/scripts/lib/pose_set.sh"
pose_set_load required || exit 1
MODEL="${MODEL:-$PS_MODEL_DIR}"
LANG_TEXT="Clear the garbage bag, the 2x4 lumber and the white bar off the blue sheet, then remove the blue sheet from the table"
export OPENARM_EMBODIMENT=bimanual
export OPENARM_CAMERAS=chest,wrist_right,wrist_left
POLICY_PORT="${POLICY_PORT:-5555}"   # 2 つのモデルのサーバを並べるときは片方を 5556 などにする（server と dry/run で同じ値）

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
      --host 127.0.0.1 --port "$POLICY_PORT"
    ;;
  server-rtc)
    [ -f "$MODEL/config.json" ] || { echo "モデルが無い: $MODEL" >&2; exit 1; }
    export HF_TOKEN="$(cat "$HOME/.cache/huggingface/token")"
    export HF_HOME="$HOME/.cache/huggingface"
    R="$HOME/projects/Isaac-GR00T-n1.7"
    unset LD_LIBRARY_PATH   # server と同じ理由（libcublasLt の版ずれ）
    cd "$R"
    echo "[serve-rtc] model=$MODEL embodiment=$OPENARM_EMBODIMENT cameras=$OPENARM_CAMERAS"
    exec env GR00T_ROOT="$R" PYTHONUNBUFFERED=1 "$R/.venv/bin/python" "$REPO/scripts/g4/gr00t_rtc_server.py" \
      --model-path "$MODEL" --embodiment-tag NEW_EMBODIMENT \
      --host 127.0.0.1 --port "$POLICY_PORT"
    ;;
  dry|run) ;;
  *) echo "usage: $0 <server|server-rtc|dry|run> [tag]" >&2; exit 1 ;;
esac

[ -n "$TAG" ] || TAG="$MODE"
INFER_MODE="${INFER_MODE:-sync}"
RTC="${RTC:-false}"
ACTION_HORIZON="${ACTION_HORIZON:-16}"
case "$INFER_MODE" in sync|async) ;; *) echo "INFER_MODE は sync か async: $INFER_MODE" >&2; exit 1 ;; esac
case "$RTC" in true|false) ;; *) echo "RTC は true か false: $RTC" >&2; exit 1 ;; esac
INFER_ARGS=(--inference_mode="$INFER_MODE" --rtc="$RTC")
if [ "$RTC" = true ]; then
  INFER_ARGS+=(--rtc_overlap="${RTC_OVERLAP:-8}" --rtc_frozen="${RTC_FROZEN:-8}" --rtc_ramp_rate="${RTC_RAMP:-5.0}")
fi
# 注: set -e の下で TAG="...$([ "$RTC" = true ] && echo -rtc)" と書くと、RTC=false のとき代入ごと
# 終了コード 1 になり、スクリプトが黙って終わる（2026-10-07 に Thor で踏んだ）。if で書く。
if [ "$INFER_MODE" != sync ] || [ "$RTC" = true ]; then
  TAG="$TAG-$INFER_MODE"
  if [ "$RTC" = true ]; then TAG="$TAG-rtc"; fi
fi
DRY=false; [ "$MODE" = dry ] && DRY=true
KP="[240,240,240,240,24,31,25,12]"
CAMS='{"chest": {"type":"intelrealsense","serial_number_or_name":"344422070898","width":640,"height":480,"fps":30}, "wrist_right": {"type":"intelrealsense","serial_number_or_name":"352122273417","width":640,"height":480,"fps":30}, "wrist_left": {"type":"intelrealsense","serial_number_or_name":"409122272149","width":640,"height":480,"fps":30}}'

echo "[rollout-bi-takeei] mode=$MODE tag=$TAG model=$MODEL"
echo "[rollout-bi-takeei] lang='$LANG_TEXT'"
echo "[rollout-bi-takeei] 開始姿勢セット=$POSE_SET（両腕とも経由地つき）"
echo "[rollout-bi-takeei] 推論の方式=$INFER_MODE rtc=$RTC action_horizon=$ACTION_HORIZON（async では不使用）"
echo "[rollout-bi-takeei] 接続すると両腕が自走して開始姿勢へ整列する。両腕と机から手を引くこと"

MODEL="$MODEL" exec bash "$REPO/scripts/g4/run_rollout.sh" "bi-takeei-$TAG" -- \
  --robot.type=bi_openarm_follower \
  --robot.left_arm_config.port=can_fol_l --robot.left_arm_config.side=left \
  --robot.right_arm_config.port=can_fol_r --robot.right_arm_config.side=right \
  --robot.id=follower \
  --robot.cameras="$CAMS" \
  --robot.left_arm_config.position_kp="$KP" \
  --robot.right_arm_config.position_kp="$KP" \
  --robot.right_arm_config.max_relative_target='{joint_1: 3.5, joint_2: 3.3, joint_3: 3.3, joint_4: 4.9, joint_5: 6.4, joint_6: 6.2, joint_7: 5.6, gripper: 20.0}' \
  --robot.left_arm_config.max_relative_target='{joint_1: 3.4, joint_2: 2.2, joint_3: 2.9, joint_4: 4.0, joint_5: 5.4, joint_6: 3.9, joint_7: 6.3, gripper: 20.0}' \
  --lang_instruction="$LANG_TEXT" \
  --policy_host=127.0.0.1 --policy_port="$POLICY_PORT" \
  --dry_run=$DRY --fps=30 --max_steps=1684 --action_horizon="$ACTION_HORIZON" --log_obs_every_step=true \
  "${INFER_ARGS[@]}" \
  --defer_camera_connect=true --camera_wait_s=30 \
  --save_wrist_frames=true
