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
#   bash scripts/record_pick_place_openarm.sh <scene-tag>                         # 第1引数=シーンタグ(英数ハイフン)
#   bash scripts/record_pick_place_openarm.sh bottle-desk50-bright                # 例
#   WRIST=true bash scripts/record_pick_place_openarm.sh <tag>                    # chest + D405 wrist (2-cam, separate dataset)
#   NUM_EPISODES=2 DISPLAY_DATA=false bash scripts/record_pick_place_openarm.sh t # dry run
#
# 1収集セッション = 1 run_dir: ~/collections/<UTC>-<scene-tag>/ に
#   console.log(端末全文・設定ダンプ込み) と start-pose-<side>.jsonl が残る。
#   RUN_DIR=... で置き場所を上書きできる。既存ディレクトリには書かない。
#   manifest.json / command.sh / ledger.csv / notes.md は未実装(バックログ2・4)。
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

# ---- run_dir: 1収集セッション = 1ディレクトリ --------------------------------
# 第1引数がシーンタグ(机の配置や対象物の識別。英数ハイフンのみ)。
# 記録の外付けであり収集ループには触らない。推論側 scripts/g4/run_rollout.sh の
# 同じ設計を移植したもの。
#
# なぜ tee が要るか: lerobot は起動時に設定の全文をダンプするが、それは stdout で
# なく stderr に出る。`2>&1` を付けないと console.log に1行も入らない。さらに
# パイプ越しだと Python が出力をブロックバッファするので PYTHONUNBUFFERED=1 が
# 無いと途中まで書かれず、異常終了時に肝心の直前ログが失われる。
TAG="${1:-untagged}"
if ! printf '%s' "$TAG" | grep -qE '^[A-Za-z0-9-]+$'; then
  echo "[record] scene-tag は英数ハイフンのみ: '$TAG'" >&2
  echo "[record] 例: bash scripts/record_pick_place_openarm.sh bottle-desk50-bright" >&2
  exit 1
fi
TS="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="${RUN_DIR:-$HOME/collections/${TS}-${TAG}}"
if [ -e "$RUN_DIR" ]; then
  echo "[record] run_dir が既に存在する(上書きしない): $RUN_DIR" >&2
  exit 1
fi
mkdir -p "$RUN_DIR"
CONSOLE_LOG="$RUN_DIR/console.log"
export PYTHONUNBUFFERED=1

# ---- エピソードの進め方 / 開始姿勢の扱い -------------------------------------
# EPISODE_ADVANCE=manual (既定): エピソードの区切りを人が握る。
#   1 エピソード収録が終わる
#   2 人が a キー -> 両腕が開始姿勢へ戻る(先に腕を退避させる = 戻り経路で
#     置いたばかりの物を薙がない。戻りは関節空間の直線補間で衝突判定は無い)
#   3 人が環境を整える
#   4 人が右矢印 -> 次のエピソード開始
#   左矢印 = 直前のテイクを破棄して録り直す(破棄後もう一度 2 から)
#   esc    = 収集終了
#   待ちは無制限。テレオペ収集は必ず人が居るので、勝手に進めない方が安全。
#
# EPISODE_ADVANCE=auto: 従来の時間駆動。reset_time_s の窓が過ぎたら次へ自動で
#   進む。開始姿勢は「許容内か判定して待つ」ゲートになる(人が手で戻す前提)。
#   エピソードだけを速く回したい時用。
EPISODE_ADVANCE="${EPISODE_ADVANCE:-manual}"
case "$EPISODE_ADVANCE" in
  manual|auto) ;;
  *) echo "[record] EPISODE_ADVANCE must be 'manual' or 'auto', got '$EPISODE_ADVANCE'" >&2; exit 1 ;;
esac

# 許容角の意味はモードで変わる。
#   manual: 戻した"後"の検算しきい値。ソフトが戻すのでサーボ整定誤差しか残らず、
#           3° あれば「戻し損ね」(CANドロップ/障害物で停止/沈み込み)だけを拾える。
#   auto  : 人が手で戻したのを裁く門のしきい値。2026-07-20 の実測ばらつきは J4 で
#           range 27.3°/std 9.7°、J1/J6/J7 も可動域の 2 割前後だったので 8°。
# START_POSE_TOL_DEG="" で開始姿勢の扱いを完全に無効化。
# ":-" ではなく "-" を使う: 前者は明示的な空文字も既定値で上書きしてしまい、
# 無効化できなくなる。
if [ "$EPISODE_ADVANCE" = "manual" ]; then
  START_POSE_TOL_DEG="${START_POSE_TOL_DEG-3}"
else
  START_POSE_TOL_DEG="${START_POSE_TOL_DEG-8}"
fi
START_POSE_MAX_WAIT_S="${START_POSE_MAX_WAIT_S:-120}"   # auto モードのゲート待ち上限のみに効く
# run_dir 配下に置く: 1セッションの記録が1ディレクトリに揃っていないと、後で
# pull_run.sh がホワイトリストで回収できない。旧既定の data-collection-logs/ は
# .gitignore 配下で、run_dir と二重管理になっていた。
START_POSE_LOG="${START_POSE_LOG:-$RUN_DIR/start-pose-${SIDE}.jsonl}"
GATE_ARGS=(--episode_advance="$EPISODE_ADVANCE")
if [ -n "$START_POSE_TOL_DEG" ]; then
  GATE_ARGS+=(
    --start_pose_tolerance_deg="$START_POSE_TOL_DEG"
    --start_pose_max_wait_s="$START_POSE_MAX_WAIT_S"
    --start_pose_log_path="$START_POSE_LOG"
  )
fi

# 設定要約も console.log に入れる(ここで新規作成し、以降は追記)。
{
  echo "[record] run_dir=$RUN_DIR  scene_tag=$TAG  ts_utc=$TS  host=$(hostname)"
  echo "[record] side=$SIDE  follower=$FOLLOWER_PORT  leader=$LEADER_PORT"
  echo "[record] cameras=$CAMERAS"
  echo "[record] wrist=$WRIST  repo_id=$REPO_ID  episodes=$NUM_EPISODES  ${CAMERA_W}x${CAMERA_H}@${FPS}fps"
  echo "[record] gravity_comp=$GRAVITY_COMP scale=$GRAVITY_SCALE side=$SIDE urdf=$GRAVITY_URDF"
  echo "[record] episode_advance=$EPISODE_ADVANCE"
  if [ "$EPISODE_ADVANCE" = "manual" ]; then
    echo "[record]   エピソード終了後: a = 両腕を開始姿勢へ戻す -> 環境を整える -> 右矢印 = 次を開始"
    echo "[record]   左矢印 = 直前のテイクを破棄して録り直す / esc = 収集終了"
    echo "[record]   a を押す前に、腕と机から手を引くこと(3.5秒ほど両腕が自律動作する)"
    if [ -n "$START_POSE_TOL_DEG" ]; then
      echo "[record]   戻し後の検算: ${START_POSE_TOL_DEG}deg  log=$START_POSE_LOG"
    else
      echo "[record]   戻し後の検算: 無効(戻し操作は行うが成否を確認しない)"
    fi
  else
    echo "[record]   時間駆動: reset_time_s=${RESET_TIME_S}s の窓が過ぎたら自動で次へ"
    if [ -n "$START_POSE_TOL_DEG" ]; then
      echo "[record]   開始姿勢ゲート: tolerance=${START_POSE_TOL_DEG}deg  max_wait=${START_POSE_MAX_WAIT_S}s  log=$START_POSE_LOG"
      echo "[record]   ゲート待機中のキー: a = 腕を戻す / 右矢印 = そのまま録る / esc = 停止"
    else
      echo "[record]   開始姿勢ゲート: 無効(従来の手動リセットのみ)"
    fi
  fi
} 2>&1 | tee "$CONSOLE_LOG"

# set -e を一時的に外す: pipefail 下で lerobot が非0終了すると、ここで即座に
# 抜けて終了メッセージも終了コードの伝播も走らなくなる。PIPESTATUS[0] で
# lerobot 自身の終了コードを取り(tee の 0 で隠さない)、最後に伝播させる。
set +e
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
  "${GATE_ARGS[@]}" 2>&1 | tee -a "$CONSOLE_LOG"
RC=${PIPESTATUS[0]}
set -e

{
  echo "[record] lerobot-record exit=$RC"
  echo "[record] 記録: $RUN_DIR"
  ls -1 "$RUN_DIR" | sed 's/^/[record]   /'
  echo "[record] 次: 軽量メタを Vault へ回収する(pull_run.sh・未実装)。"
  echo "[record]     manifest.json / command.sh / ledger.csv / notes.md も未実装(バックログ2・4)。"
} 2>&1 | tee -a "$CONSOLE_LOG"

exit "$RC"
