#!/usr/bin/env bash
# Record a bimanual (両腕) pick-and-place dataset for OpenArm with lerobot.
#
# Uses bi_openarm_follower / bi_openarm_leader (lerobot fork). CAN は
# scripts/canup.sh が付ける固定名（channel_id/dev_id ベース・USBポート非依存）:
#   can_fol_l / can_fol_r (follower) と can_lead_l / can_lead_r (leader) の4本。
# 較正は左右とも済んでいること: bash scripts/calibrate_openarm.sh left と
# bash scripts/calibrate_openarm.sh right の両方。
#
# Cameras: D435i (chest) 常時、D405 (wrist_right) 既定 true。wrist_left は
# 端子が無くまだ物理接続できないため既定 false（後日別モデルで追加）。
# カメラキーは必ず chest / wrist_right / wrist_left を使う（GR00T 側
# gr00t/openarm_embodiment/openarm_cameras.py の OPENARM_CAMERAS 既定
# chest,wrist_right と一致させるため。wrist_<side> 形式は使わない）。
#
# Prereqs:
#   1. CAN up as fixed names: sudo bash scripts/canup.sh（4本とも上がること）
#   2. 恒久ゼロ較正: bash scripts/calibrate_openarm.sh left / right（左右とも）
#
# Usage:
#   OPERATOR=Kento SCENE=<name> TASK="..." TASK_SLUG=okra \
#     bash scripts/record_bimanual_openarm.sh <scene-tag>
#   WRIST_LEFT=true WRIST_LEFT_SERIAL=<serial> ... bash scripts/record_bimanual_openarm.sh <tag>  # 左 wrist 接続後
#   NUM_EPISODES=1 DISPLAY_DATA=false bash scripts/record_bimanual_openarm.sh t                    # dry run
#   RESUME=true REPO_ID=Orboh/bi_openarm_..._YYYYMMDD_HHMMSS DATASET_ROOT=$HOME/.cache/huggingface/lerobot/Orboh/bi_openarm_..._YYYYMMDD_HHMMSS \
#     NUM_EPISODES=30 ... bash scripts/record_bimanual_openarm.sh <tag>   # 既存データセットへ追記(REPO_ID は stamp 後の実名・NUM_EPISODES は今回分)
#   DISPLAY_DATA=true DISPLAY_IP=<手元PCのIP> DISPLAY_PORT=9876 ... bash scripts/record_bimanual_openarm.sh <tag>   # 手元 PC の rerun へ映像をストリーミング
#   CONSOLE_FILTER=off bash scripts/record_bimanual_openarm.sh <tag>   # 画面の間引き（scripts/record_console_filter.py・2026-09-22）を切って全行を表示。console.log は常に全行
#
# 1収集セッション = 1 run_dir: ~/collections/<UTC>-<scene-tag>/ に
#   console.log / start-pose-bimanual.jsonl / manifest.json / command.sh が残る。
#   RUN_DIR=... で置き場所を上書きできる。既存ディレクトリには書かない。
#
# 必須の環境変数: OPERATOR(録った人) と SCENE(scenes/<name>.yaml の name) と
#   TASK(言語指示) と TASK_SLUG(repo_id に入る英数字_のみのタグ)。
#   条件を残さない dry run は SCENE=none / TODO 入りシーンは ALLOW_TODO_SCENE=true。
set -euo pipefail

# REMOTE=true は非対応: openarm_leader_remote は単腕専用で、bi_openarm_leader
# にリモート版が無い（遠隔両腕テレオペは別途 unit の対象外）。
if [ "${REMOTE:-false}" = "true" ]; then
  echo "[record-bi] REMOTE=true は両腕収集では非対応（openarm_leader_remote は単腕専用で bi_openarm_leader に remote 版が無い）" >&2
  exit 1
fi

REPO="$(cd "$(dirname "$0")/.." && pwd)"

# ---- Arm pair (channel_id fixed CAN names -- canup.sh L17-20 の固定名) --------
FOLLOWER_L=can_fol_l
FOLLOWER_R=can_fol_r
LEADER_L=can_lead_l
LEADER_R=can_lead_r

# ---- Cameras: D435i (chest) 常時; D405 wrist_right 既定 true; wrist_left 既定 false ----
CAM_SERIAL="${CAM_SERIAL:-344422070898}"            # D435i (chest)
WRIST_RIGHT="${WRIST_RIGHT:-true}"
WRIST_RIGHT_SERIAL="${WRIST_RIGHT_SERIAL:-352122273417}"  # D405 (wrist_right) 板橋個体。Kota の 9/5 個体(260322272302)とは別
WRIST_LEFT="${WRIST_LEFT:-false}"
WRIST_LEFT_SERIAL="${WRIST_LEFT_SERIAL:-}"
if [ "$WRIST_LEFT" = "true" ] && [ -z "$WRIST_LEFT_SERIAL" ]; then
  echo "[record-bi] WRIST_LEFT=true だが WRIST_LEFT_SERIAL が未指定。" >&2
  echo "[record-bi] 左 wrist は端子無しで現状未接続。serial を明示せよ。" >&2
  exit 1
fi
CAMERA_W="${CAMERA_W:-640}"
CAMERA_H="${CAMERA_H:-480}"
FPS="${FPS:-30}"

# ---- Chest/wrist_right source override (PICO 併用収集等の口。単腕版と同じ流儀) ----
CHEST_V4L2="${CHEST_V4L2:-}"
WRIST_RIGHT_V4L2="${WRIST_RIGHT_V4L2:-}"

if [ -n "$CHEST_V4L2" ]; then
  CAMERAS="{ chest: {type: opencv, index_or_path: '$CHEST_V4L2', width: $CAMERA_W, height: $CAMERA_H, fps: $FPS}"
else
  CAMERAS="{ chest: {type: intelrealsense, serial_number_or_name: '$CAM_SERIAL', width: $CAMERA_W, height: $CAMERA_H, fps: $FPS}"
fi
if [ "$WRIST_RIGHT" = "true" ]; then
  if [ -n "$WRIST_RIGHT_V4L2" ]; then
    CAMERAS="$CAMERAS, wrist_right: {type: opencv, index_or_path: '$WRIST_RIGHT_V4L2', width: $CAMERA_W, height: $CAMERA_H, fps: $FPS}"
  else
    CAMERAS="$CAMERAS, wrist_right: {type: intelrealsense, serial_number_or_name: '$WRIST_RIGHT_SERIAL', width: $CAMERA_W, height: $CAMERA_H, fps: $FPS}"
  fi
fi
if [ "$WRIST_LEFT" = "true" ]; then
  CAMERAS="$CAMERAS, wrist_left: {type: intelrealsense, serial_number_or_name: '$WRIST_LEFT_SERIAL', width: $CAMERA_W, height: $CAMERA_H, fps: $FPS}"
fi
CAMERAS="$CAMERAS }"

# ---- obs タグ（repo_id / manifest に使う） -----------------------------------
OBS=chest
[ "$WRIST_RIGHT" = "true" ] && OBS="${OBS}_wristr"
[ "$WRIST_LEFT" = "true" ] && OBS="${OBS}_wristl"

# ---- Task / dataset -----------------------------------------------------------
# single_task は学習時の言語条件。収集ごとに明示せよ(既定値でごまかさない)。
TASK="${TASK:-}"
if [ -z "$TASK" ]; then
  echo "[record-bi] TASK が未指定。single_task は学習時の言語条件。収集ごとに明示せよ。" >&2
  echo "[record-bi] 例: TASK='Pick up the okra and place it in the bin'" >&2
  exit 1
fi
TASK_SLUG="${TASK_SLUG:-}"
if [ -z "$TASK_SLUG" ] || ! printf '%s' "$TASK_SLUG" | grep -qE '^[a-z0-9_]+$'; then
  echo "[record-bi] TASK_SLUG は repo_id に入る名前。小文字英数字と _ のみで指定せよ: '$TASK_SLUG'" >&2
  echo "[record-bi] 例: TASK_SLUG=okra" >&2
  exit 1
fi
REPO_ID="${REPO_ID:-Orboh/bi_openarm_${TASK_SLUG}_${OBS}_v1}"

# ---- Gravity compensation (leader) --------------------------------------------
GRAVITY_COMP="${GRAVITY_COMP:-true}"
LSCALE="${LSCALE:-1.0}"
RSCALE="${RSCALE:-1.0}"
GRAVITY_URDF="${GRAVITY_URDF:-$REPO/urdf/openarm_v10_bimanual.urdf}"

# ---- Follower gripper kp (熱対策) --------------------------------------------
# 2026-09-21 板橋: 木材/積み木の握り込みで follower gripper(dm4310) がコイル過温ラッチ
# (0xC・OT 100℃)。両腕 teleop の積み木練習で左右とも gripper 99℃。kp 25 では握り込みの
# 停動トルクが飽和して銅損 ≈55W(2〜4K/s)。GRIPPER_KP で飽和トルクを抑える(10〜12 目安・
# 滑るなら 15)。空なら lerobot 既定(25)のまま。腕 7 関節の kp は ARM_KP(既定値)で固定。
# 値は console.log の設定ダンプ(position_kp)と command.sh に残る。
GRIPPER_KP="${GRIPPER_KP:-}"
ARM_KP="${ARM_KP:-240,240,240,240,24,31,25}"
KP_ARGS=()
if [ -n "$GRIPPER_KP" ]; then
  KP_ARGS+=(--robot.left_arm_config.position_kp="[${ARM_KP},${GRIPPER_KP}]" \
           --robot.right_arm_config.position_kp="[${ARM_KP},${GRIPPER_KP}]")
fi
# manifest の control_variables.gravity_scale は 1 値しか持てない
# (collect_manifest.py init --gravity-scale は float 1 個)。LSCALE != RSCALE
# だと片方しか manifest に残らず「非対称で録っていた」ことが後から追跡できなく
# なるため、明示許可なしでは止める。
if [ "$LSCALE" != "$RSCALE" ] && [ "${ALLOW_ASYM_GRAVITY:-false}" != "true" ]; then
  echo "[record-bi] LSCALE($LSCALE) != RSCALE($RSCALE) だが manifest の gravity_scale は1値しか持てない。" >&2
  echo "[record-bi] 非対称で録ると後から条件を追跡できなくなる。承知の上なら ALLOW_ASYM_GRAVITY=true。" >&2
  exit 1
fi

# ---- Dataset tunables --------------------------------------------------------
NUM_EPISODES="${NUM_EPISODES:-50}"
EPISODE_TIME_S="${EPISODE_TIME_S:-20}"
RESET_TIME_S="${RESET_TIME_S:-10}"
DISPLAY_DATA="${DISPLAY_DATA:-true}"
# 以下 3 点は単腕版の 2026-09-05 実機修正（Kota・Thor 作業ツリー）から移植:
# RESUME=true で既存データセットに追記する(--dataset.resume 相当)。REPO_ID は前回
# スタンプされた実際のローカル名(例: bi_openarm_..._YYYYMMDD_HHMMSS)を指定すること
# ——stamp 前の名前だと別データセットとして新規作成されてしまう。resume() は
# root=None(既定の Hub 用リビジョン安全キャッシュ)への書き込みを拒否するので、
# DATASET_ROOT に実ディレクトリ(通常 $HF_LEROBOT_HOME/<REPO_ID>)を明示すること。
# NUM_EPISODES は「今回追加で録る本数」(累計ではない)。
RESUME="${RESUME:-false}"
DATASET_ROOT="${DATASET_ROOT:-}"
ROOT_ARGS=()
if [ -n "$DATASET_ROOT" ]; then
  ROOT_ARGS+=(--dataset.root="$DATASET_ROOT")
fi
if [ "$RESUME" = "true" ] && [ -z "$DATASET_ROOT" ]; then
  echo "[record-bi] RESUME=true には DATASET_ROOT(既存データセットの実ディレクトリ)が必須。" >&2
  exit 1
fi
# Thor 自身のローカル音声(spd-say)は既知のハング要因(2026-09-05 実機で "Stop recording" の
# blocking=True 呼び出しが固まり、プロセスが応答しなくなった)。既定で OFF。
PLAY_SOUNDS="${PLAY_SOUNDS:-false}"
# 遠隔プレビュー: DISPLAY_IP/DISPLAY_PORT を指定すると Thor 側で rr.spawn() せず、
# 起動済みの手元 PC の rerun ビューア(既定ポート 9876)へ映像を飛ばす。両方揃って
# いなければ指定しない(DISPLAY_DATA=true と併用する)。
DISPLAY_IP="${DISPLAY_IP:-}"
DISPLAY_PORT="${DISPLAY_PORT:-}"
DISPLAY_ARGS=()
if [ -n "$DISPLAY_IP" ] && [ -n "$DISPLAY_PORT" ]; then
  DISPLAY_ARGS+=(--display_ip="$DISPLAY_IP" --display_port="$DISPLAY_PORT")
fi

# ---- Start pose alignment (左右それぞれキャプチャ済みなら使う) -----------------
# 既定は config/initial_pose_<side>.yaml。LEFT_POSE_PATH / RIGHT_POSE_PATH で差し替えられる
# （2026-09-23・v2 で右腕をカメラ役に固定するため右だけ別ファイルにする用途）。**明示したのに
# ファイルが無いときは黙って公式既定(肘36°)へ落ちず、ここで落とす**——開始姿勢が収集と推論で
# 割れるのは発覚が最も遅い種類の事故で、フォールバックさせてよい場面ではない。
LPOSE="${LEFT_POSE_PATH:-$REPO/config/initial_pose_left.yaml}"
RPOSE="${RIGHT_POSE_PATH:-$REPO/config/initial_pose_right.yaml}"
for _v in LEFT_POSE_PATH RIGHT_POSE_PATH; do
  _p="${!_v:-}"
  if [ -n "$_p" ] && [ ! -f "$_p" ]; then
    echo "[record-bi] $_v に指定した開始姿勢ファイルが無い: $_p" >&2
    exit 1
  fi
done
POSE_ARGS=()
if [ -f "$LPOSE" ]; then
  POSE_ARGS+=(--robot.left_arm_config.initial_pose_path="$LPOSE" --teleop.left_arm_config.initial_pose_path="$LPOSE")
fi
if [ -f "$RPOSE" ]; then
  POSE_ARGS+=(--robot.right_arm_config.initial_pose_path="$RPOSE" --teleop.right_arm_config.initial_pose_path="$RPOSE")
fi
# 無い側は公式既定(肘36°)にフォールバックする。

# 片腕をカメラ役で固定する（v2・2026-09-23）。FREEZE_SIDE=right にすると、その側の leader を
# 読まずに開始姿勢 YAML の姿勢を action として送り続ける。follower は record のループ自身に
# 保持される。**別プロセス（scripts/hold_pose.py）で保持してはいけない**——record が既に
# その CAN バスを持っているので、2 つが同じモータに別々の目標を送り合うことになる。
# FROZEN_GRIPPER_DEG は固定中のグリッパ位置（-65=全開・0=閉）。閉じた手が wrist カメラの
# 画角を塞ぐ場合に開く。整列は常に較正ゼロ（閉）へピン留めするので、ここは別に指定する。
case "${FREEZE_SIDE:-}" in
  right)
    [ -f "$RPOSE" ] || { echo "[record-bi] FREEZE_SIDE=right だが右の開始姿勢が無い: $RPOSE" >&2; exit 1; }
    POSE_ARGS+=(--teleop.right_arm_config.frozen_pose_path="$RPOSE"
                --teleop.right_arm_config.frozen_gripper_deg="${FROZEN_GRIPPER_DEG:-0}")
    echo "[record-bi] 右腕を固定する（カメラ役・grip ${FROZEN_GRIPPER_DEG:-0}）: $RPOSE"
    ;;
  left)
    [ -f "$LPOSE" ] || { echo "[record-bi] FREEZE_SIDE=left だが左の開始姿勢が無い: $LPOSE" >&2; exit 1; }
    POSE_ARGS+=(--teleop.left_arm_config.frozen_pose_path="$LPOSE"
                --teleop.left_arm_config.frozen_gripper_deg="${FROZEN_GRIPPER_DEG:-0}")
    echo "[record-bi] 左腕を固定する（カメラ役・grip ${FROZEN_GRIPPER_DEG:-0}）: $LPOSE"
    ;;
  "") ;;
  *) echo "[record-bi] FREEZE_SIDE は right / left / 未設定 のいずれか: '${FREEZE_SIDE}'" >&2; exit 1 ;;
esac

# run_dir の文字列だけ先に決める(ディレクトリはまだ作らない)。シーン/開始姿勢の
# drift チェックが --collections-dir に $RUN_DIR の親を使うため、値が要る。
TAG="${1:-untagged}"
if ! printf '%s' "$TAG" | grep -qE '^[A-Za-z0-9-]+$'; then
  echo "[record-bi] scene-tag は英数ハイフンのみ: '$TAG'" >&2
  echo "[record-bi] 例: bash scripts/record_bimanual_openarm.sh bottle-desk50-bright" >&2
  exit 1
fi
TS="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="${RUN_DIR:-$HOME/collections/${TS}-${TAG}}"

# ---- 人とシーン ----------------------------------------------------------------
OPERATOR="${OPERATOR:-}"
JUDGE="${JUDGE:-}"
if [ -z "$OPERATOR" ]; then
  echo "[record-bi] OPERATOR が未指定。誰が録ったか分からないデータは学習に使えない。" >&2
  echo "[record-bi] 例: OPERATOR=Kento SCENE=bottle-desk-v1 TASK=... TASK_SLUG=okra bash scripts/record_bimanual_openarm.sh <tag>" >&2
  exit 1
fi

SCENE="${SCENE:-}"
SCENE_PATH=""
if [ -z "$SCENE" ]; then
  echo "[record-bi] SCENE が未指定。物理条件が記録されず、あとから条件を復元できない。" >&2
  echo "[record-bi] 利用可能なシーン:" >&2
  if [ -d "$REPO/scenes" ]; then
    ls -1 "$REPO/scenes" | grep -v '^_' | sed 's/\.yaml$//; s/^/[record-bi]   /' >&2
  else
    echo "[record-bi]   (scenes/ が無い)" >&2
  fi
  echo "[record-bi] 新しい環境で録るなら、雛形をコピーして実測値を入れる:" >&2
  echo "[record-bi]   cp scenes/_TEMPLATE.yaml scenes/<name>.yaml && \$EDITOR scenes/<name>.yaml" >&2
  echo "[record-bi]   TODO を全部埋めるまでは起動できない(埋め忘れを防ぐため)" >&2
  echo "[record-bi] 条件を残さない dry run なら SCENE=none" >&2
  exit 1
fi
if [ "$SCENE" != "none" ]; then
  SCENE_PATH="$REPO/scenes/${SCENE}.yaml"
  if [ ! -f "$SCENE_PATH" ]; then
    echo "[record-bi] シーンプロファイルが無い: $SCENE_PATH" >&2
    echo "[record-bi] 新しい環境なら、雛形をコピーして作る:" >&2
    echo "[record-bi]   cp scenes/_TEMPLATE.yaml \"scenes/${SCENE}.yaml\" && \$EDITOR \"scenes/${SCENE}.yaml\"" >&2
    echo "[record-bi] 実機の前で TODO を全部埋めてから、もう一度この起動を打つ。" >&2
    echo "[record-bi] 既存の環境で録るつもりなら、名前の綴りを確認する:" >&2
    ls -1 "$REPO/scenes" 2>/dev/null | grep -v '^_' | sed 's/\.yaml$//; s/^/[record-bi]   /' >&2
    exit 1
  fi
fi

# manifest を書く python。lerobot-record と同じインタプリタを使う(shebang から取る)。
LEROBOT_RECORD="${LEROBOT_RECORD:-}"
if [ -z "$LEROBOT_RECORD" ]; then
  if command -v lerobot-record >/dev/null 2>&1; then
    LEROBOT_RECORD=lerobot-record
  else
    LEROBOT_RECORD="$HOME/projects/lerobot/.venv/bin/lerobot-record"
  fi
fi
if ! command -v "$LEROBOT_RECORD" >/dev/null 2>&1 && [ ! -x "$LEROBOT_RECORD" ]; then
  echo "[record-bi] NG: lerobot-record が見つからない ($LEROBOT_RECORD)" >&2
  exit 1
fi
case "$LEROBOT_RECORD" in
  */bin/lerobot-record) export PATH="$(dirname "$LEROBOT_RECORD"):$PATH" ;;
esac

PYTHON_BIN="${PYTHON_BIN:-}"
if [ -z "$PYTHON_BIN" ]; then
  LR_BIN="$(command -v lerobot-record || true)"
  if [ -n "$LR_BIN" ]; then
    PYTHON_BIN="$(head -1 "$LR_BIN" | sed -e 's/^#!//' -e 's/[[:space:]].*$//')"
  fi
  if [ ! -x "${PYTHON_BIN:-}" ]; then PYTHON_BIN="$HOME/projects/lerobot/.venv/bin/python"; fi
  if [ ! -x "${PYTHON_BIN:-}" ]; then PYTHON_BIN=python3; fi
fi
MANIFEST_TOOL="$REPO/scripts/collect_manifest.py"

# 左右の開始姿勢が同一キャプチャの鏡像かを検算する。左右バラバラのキャプチャ
# 由来(片方だけ撮り直した等)を組で使うと、両腕を揃えて戻しても実際には非対称
# な姿勢に着地し、それが気づかれないまま収集が進む。joint_1/2/3/5/6/7 は左右で
# 符号反転(|L+R|が小さいはず)、joint_4(肘)は左右で同符号(|L-R|が小さいはず)、
# gripper は較正ゼロ固定で対象外。
if [ -f "$LPOSE" ] && [ -f "$RPOSE" ]; then
  POSE_MIRROR_TOL_DEG="${POSE_MIRROR_TOL_DEG:-2.0}"
  MIRROR_CHECK_OUT="$("$PYTHON_BIN" - "$LPOSE" "$RPOSE" "$POSE_MIRROR_TOL_DEG" <<'PYEOF'
import sys
import yaml

lpath, rpath, tol = sys.argv[1], sys.argv[2], float(sys.argv[3])
with open(lpath) as f:
    lpose = yaml.safe_load(f)
with open(rpath) as f:
    rpose = yaml.safe_load(f)

bad = []
for j in ("joint_1", "joint_2", "joint_3", "joint_5", "joint_6", "joint_7"):
    l, r = float(lpose[j]), float(rpose[j])
    diff = abs(l + r)
    if diff > tol:
        bad.append(f"{j}: L={l} R={r} |L+R|={diff:.2f} > {tol}deg (符号反転のはずが崩れている)")
for j in ("joint_4",):
    l, r = float(lpose[j]), float(rpose[j])
    diff = abs(l - r)
    if diff > tol:
        bad.append(f"{j}: L={l} R={r} |L-R|={diff:.2f} > {tol}deg (肘は同符号のはずが崩れている)")
if bad:
    print("\n".join(bad))
PYEOF
)"
  if [ -n "$MIRROR_CHECK_OUT" ]; then
    if [ "${ALLOW_ASYM_POSE:-false}" != "true" ]; then
      echo "[record-bi] 左右の開始姿勢 ($LPOSE / $RPOSE) が同一キャプチャの鏡像になっていない:" >&2
      echo "$MIRROR_CHECK_OUT" | sed 's/^/[record-bi]   /' >&2
      echo "[record-bi] 片方だけ撮り直した等で組が崩れている可能性。撮り直すか、意図的な" >&2
      echo "[record-bi] 非対称なら承知の上で ALLOW_ASYM_POSE=true。" >&2
      exit 1
    fi
    echo "[record-bi] 警告: 左右の開始姿勢が鏡像から外れている(ALLOW_ASYM_POSE=true で続行):" >&2
    echo "$MIRROR_CHECK_OUT" | sed 's/^/[record-bi]   /' >&2
  fi
fi

# ---- 同期負債の警告（録る前に「古いコードで録っていないか」を見える化） --------
if [ "${SKIP_SYNC_CHECK:-false}" != "true" ] && [ -f "$REPO/scripts/check_repo_sync.sh" ]; then
  LEROBOT_ROOT="$("$PYTHON_BIN" -c 'import lerobot, pathlib; print(pathlib.Path(lerobot.__file__).resolve().parents[2])' 2>/dev/null || true)"
  for _r in "$REPO" ${LEROBOT_ROOT:+"$LEROBOT_ROOT"}; do
    bash "$REPO/scripts/check_repo_sync.sh" "$_r" >&2 || true
  done
fi

# TODO が残ったままでは録らせない。
if [ -n "$SCENE_PATH" ]; then
  TODO_N="$("$PYTHON_BIN" "$MANIFEST_TOOL" check-scene --scene-path "$SCENE_PATH" 2>/dev/null || echo "")"
  if printf '%s' "$TODO_N" | grep -qE '^[0-9]+$' && [ "$TODO_N" -gt 0 ]; then
    if [ "${ALLOW_TODO_SCENE:-false}" != "true" ]; then
      echo "[record-bi] シーンプロファイルに未確定(TODO)が $TODO_N 件残っている: $SCENE_PATH" >&2
      echo "[record-bi] 実測値を入れてから録ること。TODO 版で録ると、あとで埋めた時に sha が" >&2
      echo "[record-bi] 変わり、このセッションだけ条件を辿れなくなる。" >&2
      echo "[record-bi] 条件を残さない dry run なら ALLOW_TODO_SCENE=true" >&2
      exit 1
    fi
    echo "[record-bi] 警告: TODO $TODO_N 件のシーンで録る(ALLOW_TODO_SCENE=true)。条件は復元できない。" >&2
  fi

  DRIFT="$("$PYTHON_BIN" "$MANIFEST_TOOL" check-drift --scene-path "$SCENE_PATH" \
            --collections-dir "$(dirname "$RUN_DIR")" 2>/dev/null || echo "")"
  if [ -n "$DRIFT" ]; then
    set -- $DRIFT
    if [ "${ALLOW_SCENE_DRIFT:-false}" != "true" ]; then
      echo "[record-bi] シーン '$SCENE' が前回の収録時と変わっている（sha が割れた）" >&2
      echo "[record-bi]   前回 $2 （run: $4）" >&2
      echo "[record-bi]   今回 $3" >&2
      echo "[record-bi] このまま録ると 50ep が2つの条件に割れ、あとから直せない。" >&2
      echo "[record-bi] 物理配置を変えたなら**新しいシーンファイル**を作って SCENE を変える。" >&2
      echo "[record-bi] 変えたつもりが無いなら git diff で何が動いたか確認する。" >&2
      echo "[record-bi] 承知で続けるなら ALLOW_SCENE_DRIFT=true" >&2
      exit 1
    fi
    echo "[record-bi] 警告: シーン sha が前回と違う（$2 → $3・ALLOW_SCENE_DRIFT=true）" >&2
  fi
fi

# 開始姿勢 YAML の drift も同様に見る。左右それぞれ独立にチェックする
# (check-pose-drift は manifest の initial_pose と initial_pose_left の両キーを
# stem で照合する実装済み)。
for _p in "$LPOSE" "$RPOSE"; do
  [ -f "$_p" ] || continue
  POSE_DRIFT="$("$PYTHON_BIN" "$MANIFEST_TOOL" check-pose-drift --initial-pose-path "$_p" \
            --collections-dir "$(dirname "$RUN_DIR")" 2>/dev/null || echo "")"
  if [ -n "$POSE_DRIFT" ]; then
    set -- $POSE_DRIFT
    if [ "${ALLOW_POSE_DRIFT:-false}" != "true" ]; then
      echo "[record-bi] 開始姿勢 '$_p' が前回の収録時と変わっている（sha が割れた）" >&2
      echo "[record-bi]   前回 $2 （run: $4）" >&2
      echo "[record-bi]   今回 $3" >&2
      echo "[record-bi] このまま録ると 50ep が2つの条件に割れ、あとから直せない。" >&2
      echo "[record-bi] 撮り直したなら承知で ALLOW_POSE_DRIFT=true。誤って上書きしたなら復元する。" >&2
      exit 1
    fi
    echo "[record-bi] 警告: 開始姿勢 sha が前回と違う（$2 → $3・ALLOW_POSE_DRIFT=true）" >&2
  fi
done

# ---- run_dir: 1収集セッション = 1ディレクトリ --------------------------------
if [ -e "$RUN_DIR" ]; then
  echo "[record-bi] run_dir が既に存在する(上書きしない): $RUN_DIR" >&2
  exit 1
fi
mkdir -p "$RUN_DIR"
CONSOLE_LOG="$RUN_DIR/console.log"
export PYTHONUNBUFFERED=1

# ---- エピソードの進め方 / 開始姿勢の扱い -------------------------------------
EPISODE_ADVANCE="${EPISODE_ADVANCE:-manual}"
case "$EPISODE_ADVANCE" in
  manual|auto) ;;
  *) echo "[record-bi] EPISODE_ADVANCE must be 'manual' or 'auto', got '$EPISODE_ADVANCE'" >&2; exit 1 ;;
esac

if [ "$EPISODE_ADVANCE" = "manual" ]; then
  START_POSE_TOL_DEG="${START_POSE_TOL_DEG-3}"
else
  START_POSE_TOL_DEG="${START_POSE_TOL_DEG-8}"
fi
START_POSE_MAX_WAIT_S="${START_POSE_MAX_WAIT_S:-120}"
START_POSE_LOG="${START_POSE_LOG:-$RUN_DIR/start-pose-bimanual.jsonl}"
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
  echo "[record-bi] run_dir=$RUN_DIR  scene_tag=$TAG  ts_utc=$TS  host=$(hostname)"
  echo "[record-bi] follower=L:$FOLLOWER_L R:$FOLLOWER_R  leader=L:$LEADER_L R:$LEADER_R"
  echo "[record-bi] cameras=$CAMERAS"
  echo "[record-bi] d405: $( [ -n "${D405_GAIN:-}" ] && echo "manual exposure=${D405_EXPOSURE:-28000}us gain=$D405_GAIN（camera_setup.py で焼き済み）" || echo "auto exposure（既定。垂らし姿勢で床を見る向きだけ暗く出る）" )"
  echo "[record-bi] d435: auto exposure, brightness=${D435_BRIGHTNESS:-0}"
  echo "[record-bi] obs=$OBS  repo_id=$REPO_ID  resume=$RESUME  root=${DATASET_ROOT:-<default>}  play_sounds=$PLAY_SOUNDS  episodes(this session)=$NUM_EPISODES  ${CAMERA_W}x${CAMERA_H}@${FPS}fps"
  echo "[record-bi] gravity_comp=$GRAVITY_COMP L=$LSCALE R=$RSCALE urdf=$GRAVITY_URDF"
  echo "[record-bi] episode_advance=$EPISODE_ADVANCE"
  if [ "$EPISODE_ADVANCE" = "manual" ]; then
    echo "[record-bi]   起動時: follower L→R, leader L→R の順に各 2.2 秒ずつ自動整列する(約 9 秒)。connect 前に両腕と机から手を引くこと"
    echo "[record-bi]   エピソード終了後: a = 両腕を開始姿勢へ戻す -> 環境を整える -> 右矢印 = 次を開始"
    echo "[record-bi]   左矢印 = 直前のテイクを破棄して録り直す / esc = 収集終了"
    echo "[record-bi]   a を押す前に両腕と机から手を引くこと（leader 左→右が各 2.0 秒、続いて follower 両腕 1.5 秒＝約 5.5 秒両腕が自律動作する）"
    if [ -n "$START_POSE_TOL_DEG" ]; then
      echo "[record-bi]   戻し後の検算: ${START_POSE_TOL_DEG}deg  log=$START_POSE_LOG"
    else
      echo "[record-bi]   戻し後の検算: 無効(戻し操作は行うが成否を確認しない)"
    fi
  else
    echo "[record-bi]   時間駆動: reset_time_s=${RESET_TIME_S}s の窓が過ぎたら自動で次へ"
    if [ -n "$START_POSE_TOL_DEG" ]; then
      echo "[record-bi]   開始姿勢ゲート: tolerance=${START_POSE_TOL_DEG}deg  max_wait=${START_POSE_MAX_WAIT_S}s  log=$START_POSE_LOG"
      echo "[record-bi]   ゲート待機中のキー: a = 腕を戻す / 右矢印 = そのまま録る / esc = 停止"
    else
      echo "[record-bi]   開始姿勢ゲート: 無効(従来の手動リセットのみ)"
    fi
  fi
} 2>&1 | tee "$CONSOLE_LOG"

# ---- manifest.json ------------------------------------------------------------
SESSION_ID="${TS}-${TAG}"
STARTED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
MANIFEST_ARGS=(
  init
  --run-dir "$RUN_DIR"
  --session-id "$SESSION_ID"
  --started-utc "$STARTED_UTC"
  --operator "$OPERATOR"
  --judge "$JUDGE"
  --scene "$SCENE"
  --scene-path "$SCENE_PATH"
  --side bimanual
  --cam-serial "$CAM_SERIAL"
  --initial-pose-path "$RPOSE"
  --initial-pose-path-left "$LPOSE"
  --gravity-scale "$RSCALE"
  --repo-id "$REPO_ID"
  --repo-root "$REPO"
  --fps "$FPS"
)
if [ "$WRIST_RIGHT" = "true" ]; then
  MANIFEST_ARGS+=(--wrist-right-serial "$WRIST_RIGHT_SERIAL")
fi
if [ "$WRIST_LEFT" = "true" ]; then
  MANIFEST_ARGS+=(--wrist-left-serial "$WRIST_LEFT_SERIAL")
fi
MANIFEST_INIT_OUT="$RUN_DIR/.manifest-init.out"
MANIFEST_OK=true
if ! "$PYTHON_BIN" "$MANIFEST_TOOL" "${MANIFEST_ARGS[@]}" > "$MANIFEST_INIT_OUT" 2>&1; then
  MANIFEST_OK=false
fi

# ---- command.sh --------------------------------------------------------------
CAM_NOTE="chest D435i"
[ "$WRIST_RIGHT" = "true" ] && CAM_NOTE="$CAM_NOTE + wrist_right D405"
[ "$WRIST_LEFT" = "true" ] && CAM_NOTE="$CAM_NOTE + wrist_left D405"
CAM_NOTE="$CAM_NOTE (type=intelrealsense)"
COMMAND_SH="$RUN_DIR/command.sh"
{
  echo '#!/usr/bin/env bash'
  echo "# 自動生成: このセッションを同じ条件で起動し直す行。"
  echo "# 解決済み設定の全文 -> console.log / 環境スナップショット -> manifest.json"
  echo "# 設定ダンプが落とす type 判別子: robot.type=bi_openarm_follower / teleop.type=bi_openarm_leader"
  echo "# カメラ: $CAM_NOTE"
  echo "set -euo pipefail"
  printf 'OPERATOR=%q \\\n' "$OPERATOR"
  printf 'JUDGE=%q \\\n' "$JUDGE"
  printf 'SCENE=%q \\\n' "$SCENE"
  printf 'CAM_SERIAL=%q CHEST_V4L2=%q \\\n' "$CAM_SERIAL" "$CHEST_V4L2"
  printf 'WRIST_RIGHT=%q WRIST_RIGHT_SERIAL=%q WRIST_RIGHT_V4L2=%q \\\n' "$WRIST_RIGHT" "$WRIST_RIGHT_SERIAL" "$WRIST_RIGHT_V4L2"
  printf 'WRIST_LEFT=%q WRIST_LEFT_SERIAL=%q \\\n' "$WRIST_LEFT" "$WRIST_LEFT_SERIAL"
  printf 'CAMERA_W=%q CAMERA_H=%q FPS=%q \\\n' "$CAMERA_W" "$CAMERA_H" "$FPS"
  printf 'NUM_EPISODES=%q EPISODE_TIME_S=%q RESET_TIME_S=%q \\\n' "$NUM_EPISODES" "$EPISODE_TIME_S" "$RESET_TIME_S"
  printf 'REPO_ID=%q \\\n' "$REPO_ID"
  printf 'RESUME=%q DATASET_ROOT=%q \\\n' "$RESUME" "$DATASET_ROOT"
  printf 'PLAY_SOUNDS=%q DISPLAY_IP=%q DISPLAY_PORT=%q \\\n' "$PLAY_SOUNDS" "$DISPLAY_IP" "$DISPLAY_PORT"
  printf 'D405_GAIN=%q D405_EXPOSURE=%q D435_BRIGHTNESS=%q \\\n' "${D405_GAIN:-}" "${D405_EXPOSURE:-}" "${D435_BRIGHTNESS:-}"
  printf 'TASK=%q \\\n' "$TASK"
  printf 'TASK_SLUG=%q \\\n' "$TASK_SLUG"
  printf 'GRAVITY_COMP=%q LSCALE=%q RSCALE=%q GRAVITY_URDF=%q \\\n' "$GRAVITY_COMP" "$LSCALE" "$RSCALE" "$GRAVITY_URDF"
  printf 'GRIPPER_KP=%q ARM_KP=%q \\\n' "$GRIPPER_KP" "$ARM_KP"
  printf 'EPISODE_ADVANCE=%q START_POSE_TOL_DEG=%q START_POSE_MAX_WAIT_S=%q \\\n' \
    "$EPISODE_ADVANCE" "$START_POSE_TOL_DEG" "$START_POSE_MAX_WAIT_S"
  printf 'DISPLAY_DATA=%q \\\n' "$DISPLAY_DATA"
  printf 'bash scripts/record_bimanual_openarm.sh %q\n' "$TAG"
} > "$COMMAND_SH"
chmod +x "$COMMAND_SH"

{
  sed 's/^/[record-bi] manifest: /' "$MANIFEST_INIT_OUT"
  if [ "$MANIFEST_OK" != true ]; then
    echo "[record-bi] 警告: manifest.json を書けなかった。環境スナップショットが残らない。"
  fi
  echo "[record-bi] command.sh: $COMMAND_SH"
} 2>&1 | tee -a "$CONSOLE_LOG"
rm -f "$MANIFEST_INIT_OUT"

# set -e を一時的に外す: pipefail 下で lerobot が非0終了すると、ここで即座に
# 抜けて終了メッセージも終了コードの伝播も走らなくなる。PIPESTATUS[0] で
# lerobot 自身の終了コードを取り(tee の 0 で隠さない)、最後に伝播させる。
# --teleop.*.use_velocity_and_torque は渡さない(単腕収集版と同じ流儀。dataset の
# action 特徴は robot.action_features の .pos のみなので生成物は変わらない。
# run_teleop_bimanual.sh は渡すが、teleop 品質確認用と収集の契約は別軸)。
set +e
"$LEROBOT_RECORD" \
  --robot.type=bi_openarm_follower \
  --robot.left_arm_config.port="$FOLLOWER_L" --robot.left_arm_config.side=left \
  --robot.right_arm_config.port="$FOLLOWER_R" --robot.right_arm_config.side=right \
  --robot.id=follower \
  --robot.cameras="$CAMERAS" \
  --teleop.type=bi_openarm_leader \
  --teleop.left_arm_config.port="$LEADER_L" \
  --teleop.right_arm_config.port="$LEADER_R" \
  --teleop.id=leader \
  --teleop.left_arm_config.gravity_compensation="$GRAVITY_COMP" --teleop.left_arm_config.gravity_urdf_path="$GRAVITY_URDF" --teleop.left_arm_config.gravity_side=left --teleop.left_arm_config.gravity_scale="$LSCALE" \
  --teleop.right_arm_config.gravity_compensation="$GRAVITY_COMP" --teleop.right_arm_config.gravity_urdf_path="$GRAVITY_URDF" --teleop.right_arm_config.gravity_side=right --teleop.right_arm_config.gravity_scale="$RSCALE" \
  --display_data="$DISPLAY_DATA" \
  ${DISPLAY_ARGS[@]+"${DISPLAY_ARGS[@]}"} \
  --resume="$RESUME" \
  --play_sounds="$PLAY_SOUNDS" \
  --dataset.repo_id="$REPO_ID" ${ROOT_ARGS[@]+"${ROOT_ARGS[@]}"} --dataset.single_task="$TASK" --dataset.fps="$FPS" \
  --dataset.num_episodes="$NUM_EPISODES" --dataset.episode_time_s="$EPISODE_TIME_S" --dataset.reset_time_s="$RESET_TIME_S" \
  --dataset.push_to_hub=false \
  ${KP_ARGS[@]+"${KP_ARGS[@]}"} \
  ${POSE_ARGS[@]+"${POSE_ARGS[@]}"} \
  "${GATE_ARGS[@]}" 2>&1 | tee -a "$CONSOLE_LOG" | "$PYTHON_BIN" -u "$REPO/scripts/record_console_filter.py"
RC=${PIPESTATUS[0]}
set -e

# stamp 後の実 repo_id は実行後にしか分からない。ファイルシステムから引いて
# manifest に追記する。
MANIFEST_RESOLVE_OUT="$RUN_DIR/.manifest-resolve.out"
if ! "$PYTHON_BIN" "$MANIFEST_TOOL" resolve --run-dir "$RUN_DIR" --exit-code "$RC" \
     > "$MANIFEST_RESOLVE_OUT" 2>&1; then
  echo "[record-bi] 警告: manifest の事後解決に失敗した(stamp 後 repo_id が未記録)" >> "$MANIFEST_RESOLVE_OUT"
fi

{
  echo "[record-bi] lerobot-record exit=$RC"
  sed 's/^/[record-bi] manifest: /' "$MANIFEST_RESOLVE_OUT"
  echo "[record-bi] 記録: $RUN_DIR"
  ls -1 "$RUN_DIR" | sed 's/^/[record-bi]   /'
  echo "[record-bi] 次: 軽量メタを Vault へ回収する(pull_run.sh・未実装=バックログ4)。"
  echo "[record-bi]     manifest.json の human_fields(散らした範囲・異常)は人が埋める。"
} 2>&1 | tee -a "$CONSOLE_LOG"
rm -f "$MANIFEST_RESOLVE_OUT"

exit "$RC"
