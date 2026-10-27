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
#   REMOTE=true bash scripts/record_pick_place_openarm.sh <tag>                    # 遠隔テレオペ収集 (leader は操作側PCの leader_agent)
#   CHEST_V4L2=/dev/video11 ... bash scripts/record_pick_place_openarm.sh <tag>    # chest を仮想カメラから取る (PICO 分配・下記)
#   NUM_EPISODES=2 DISPLAY_DATA=false bash scripts/record_pick_place_openarm.sh t # dry run
#
# 1収集セッション = 1 run_dir: ~/collections/<UTC>-<scene-tag>/ に
#   console.log(端末全文・設定ダンプ込み) / start-pose-<side>.jsonl /
#   manifest.json(環境スナップショット) / command.sh(再実行用) が残る。
#   RUN_DIR=... で置き場所を上書きできる。既存ディレクトリには書かない。
#   ledger.csv / notes.md / setup/ と Vault への回収(pull_run.sh)は未実装(バックログ4)。
#
# 必須の環境変数: OPERATOR(録った人) と SCENE(scenes/<name>.yaml の name)。
#   条件を残さない dry run は SCENE=none / TODO 入りシーンは ALLOW_TODO_SCENE=true。
set -euo pipefail

# repo ルート(このスクリプトの一つ上)。GRAVITY_URDF の既定値・POSE/SCENE の解決・
# check_repo_sync.sh 呼び出しなど、スクリプト全体で使う。BASH_SOURCE を使うので
# source されても解決する。
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ---- Arm pair (channel_id fixed CAN names -- USB/PC independent) -------------
SIDE="${SIDE:-left}"                        # left | right
case "$SIDE" in
  left)  FOLLOWER_PORT=can_fol_l; LEADER_PORT=can_lead_l; DEFAULT_WRIST_SERIAL=409122272149 ;;
  right) FOLLOWER_PORT=can_fol_r; LEADER_PORT=can_lead_r; DEFAULT_WRIST_SERIAL=352122273417 ;;
  *) echo "[record] SIDE must be 'left' or 'right', got '$SIDE'" >&2; exit 1 ;;
esac

# ---- Cameras: D435i (chest) always; optional D405 wrist (WRIST=true) ----------
CAM_SERIAL="${CAM_SERIAL:-344422070898}"      # D435i (chest)
WRIST="${WRIST:-false}"                       # true => add D405 wrist cam (different model! separate dataset)
# D405 wrist cams -- confirmed 2026-10-19 via lerobot-find-cameras (left = has E-stop button
# in frame): left=409122272149 / right=352122273417. Defaults picked per $SIDE above.
WRIST_SERIAL="${WRIST_SERIAL:-$DEFAULT_WRIST_SERIAL}"
CAMERA_W="${CAMERA_W:-640}"
CAMERA_H="${CAMERA_H:-480}"
FPS="${FPS:-30}"                            # = dataset.fps AND teleop loop rate (record forces them equal); camera-capped (~<=60 @640x480 RGB). FPS=60 reduces follower jitter

# ---- Chest source override (PICO 併用収集: v4l2loopback 分配の口) --------------
# CHEST_V4L2=/dev/videoNN を指定すると chest を RealSense SDK 直でなく OpenCV
# (V4L2) で開く。使いどころ: D435i を gst の tee で仮想カメラ2本へ分配し、片方を
# PICO 送信 (OrinVideoSender)、もう片方をここへ渡すと「オペレータが PICO で見て
# いる映像そのもの」がデータセットの chest になる。SDK 直読 (既定) は sender と
# 同一カメラで排他なので、PICO 併用収集では必ずこちらを使うこと。
CHEST_V4L2="${CHEST_V4L2:-}"
# WRIST_V4L2: 同上の wrist 版。PiP 分配 (start-collection-cameras.sh) は実カメラ
# 2台とも V4L2 で占有するので、分配併用時は両方を仮想カメラ経由にする。
WRIST_V4L2="${WRIST_V4L2:-}"

# Build the cameras dict: chest always; append wrist_<SIDE> only when WRIST=true
if [ -n "$CHEST_V4L2" ]; then
  CAMERAS="{ chest: {type: opencv, index_or_path: '$CHEST_V4L2', width: $CAMERA_W, height: $CAMERA_H, fps: $FPS}"
else
  CAMERAS="{ chest: {type: intelrealsense, serial_number_or_name: '$CAM_SERIAL', width: $CAMERA_W, height: $CAMERA_H, fps: $FPS}"
fi
if [ "$WRIST" = "true" ]; then
  if [ -n "$WRIST_V4L2" ]; then
    CAMERAS="$CAMERAS, wrist_${SIDE}: {type: opencv, index_or_path: '$WRIST_V4L2', width: $CAMERA_W, height: $CAMERA_H, fps: $FPS}"
  else
    CAMERAS="$CAMERAS, wrist_${SIDE}: {type: intelrealsense, serial_number_or_name: '$WRIST_SERIAL', width: $CAMERA_W, height: $CAMERA_H, fps: $FPS}"
  fi
fi
CAMERAS="$CAMERAS }"

# ---- Remote teleop (leader = 操作側PCの leader_agent) --------------------------
# REMOTE=true: teleop を openarm_leader_remote (ZMQ 受信) に差し替える。leader
# 実機とその重力補償は操作側PC (run_leader_agent_<side>.sh) の管轄なので、
# --teleop.port / --teleop.gravity_* はこちらでは渡さない。ポートは leader_agent
# 側と一致させる (right=5701/5711, left=5702/5712)。
# 'a' キーはローカル収集と同じに働く: リモート teleop の soft_move_to が echo 経路で
# leader_agent に戻り指令を送り、実 leader が開始姿勢へ整列→follower を歩かせる。
# 指令が届かない/収束しない場合は従来どおり「手で戻す」にフォールバック (ゲートが検算)。
REMOTE="${REMOTE:-false}"
ZMQ_PORT="${ZMQ_PORT:-}"
ECHO_PORT="${ECHO_PORT:-}"
if [ "$REMOTE" = "true" ]; then
  case "$SIDE" in
    right) ZMQ_PORT="${ZMQ_PORT:-5701}"; ECHO_PORT="${ECHO_PORT:-5711}" ;;
    left)  ZMQ_PORT="${ZMQ_PORT:-5702}"; ECHO_PORT="${ECHO_PORT:-5712}" ;;
  esac
fi

# ---- Leader gravity compensation (light leader for comfortable collection) ---
# Inject G(q) feed-forward on the leader (joints 1-7; gripper excluded) so it can
# be moved by hand. Independent of FPS. Requires Thor's lerobot venv on the
# feat/gravity-comp branch with pinocchio installed. scale 1.2 validated 2026-06-17;
# fixed at 1.0 for the 50-episode collection (2026-08-05 Kento decision, tested
# clean across the day's sessions) so all 5 sessions share one value.
GRAVITY_COMP="${GRAVITY_COMP:-true}"
GRAVITY_SCALE="${GRAVITY_SCALE:-1.0}"
# hand-port of eec9666 (cherry-pick conflicted): repo-relative ($HOME/openarm-orboh
# 決め打ちだと Thor/Vector の ~/projects/openarm-orboh 配置で壊れる。REPO は上で解決済み)
GRAVITY_URDF="${GRAVITY_URDF:-$REPO/urdf/openarm_v10_bimanual.urdf}"

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
#     録り始めない(推論側 run_rollout_lerobot.sh は reset_to_initial_position で
#     完全自動復帰。record は teleop 運転中で人の手が leader に載るため、
#     無断で腕を動かさずゲート+キー押しアラインにしてある)。
#   - 収集セッションを跨ぐ場合も同じ ready YAML を使い続ける(較正=恒久ゼロ +
#     同一 YAML で、セッション間の座標系ずれ・開始姿勢ずれの両方を防ぐ)。
# REPO は冒頭で解決済み。
source "$REPO/scripts/lib/pose_set.sh"
pose_set_load current || exit 1
if [ "$SIDE" = left ]; then POSE="$LPOSE"; else POSE="$RPOSE"; fi
POSE_ARGS=()
if [ -f "$POSE" ]; then
  POSE_ARGS+=(--robot.initial_pose_path="$POSE")
  # openarm_leader_remote に initial_pose_path は無い (leader 実機の整列は操作側の
  # leader_agent が同名 YAML で行う。両マシンの YAML 同一が前提 = md5sum で照合)。
  [ "$REMOTE" != "true" ] && POSE_ARGS+=(--teleop.initial_pose_path="$POSE")
fi

# run_dir の文字列だけ先に決める(ディレクトリはまだ作らない)。シーン/開始姿勢の
# drift チェックが --collections-dir に $RUN_DIR の親を使うため、値が要る。
# ここで決めるのは値のみ: 実際の存在チェックと mkdir は後方の run_dir セクション
# (第1引数=シーンタグの検証がまだ済んでいないため、通常の実行順はそちらが正)。
TAG="${1:-untagged}"
if ! printf '%s' "$TAG" | grep -qE '^[A-Za-z0-9-]+$'; then
  echo "[record] scene-tag は英数ハイフンのみ: '$TAG'" >&2
  echo "[record] 例: bash scripts/record_pick_place_openarm.sh bottle-desk50-bright" >&2
  exit 1
fi
TS="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="${RUN_DIR:-$HOME/collections/${TS}-${TAG}}"

# ---- 人とシーン(人が入れる3つのうち2つ) --------------------------------------
# OPERATOR を必須にするのは、操作者が違うとデモの癖が変わり、少数データの模倣
# 学習では交絡要因になるから。実際に bottle 25ep は3人混在(Kento/Satomi/Aoi)で
# 訓練セットにできなくなった。JUDGE(成否の判定者)は収録後に決まることもあるので任意。
OPERATOR="${OPERATOR:-}"
JUDGE="${JUDGE:-}"
if [ -z "$OPERATOR" ]; then
  echo "[record] OPERATOR が未指定。誰が録ったか分からないデータは学習に使えない。" >&2
  echo "[record] 例: OPERATOR=Kento SCENE=bottle-desk-v1 bash scripts/record_pick_place_openarm.sh <tag>" >&2
  exit 1
fi

# SCENE = scenes/<name>.yaml の <name>。机の高さ・照明・散らし方といった
# 「ソフトから見えない物理」の正本で、manifest には名前と sha が焼かれる。
# sha を持つのは、名前が同じまま中身を書き換えられても検出できるようにするため。
SCENE="${SCENE:-}"
SCENE_PATH=""
if [ -z "$SCENE" ]; then
  echo "[record] SCENE が未指定。物理条件が記録されず、あとから条件を復元できない。" >&2
  echo "[record] 利用可能なシーン:" >&2
  # `_` 始まりは雛形・テスト用なので候補に出さない。
  if [ -d "$REPO/scenes" ]; then
    ls -1 "$REPO/scenes" | grep -v '^_' | sed 's/\.yaml$//; s/^/[record]   /' >&2
  else
    echo "[record]   (scenes/ が無い)" >&2
  fi
  echo "[record] 新しい環境で録るなら、雛形をコピーして実測値を入れる:" >&2
  echo "[record]   cp scenes/_TEMPLATE.yaml scenes/<name>.yaml && \$EDITOR scenes/<name>.yaml" >&2
  echo "[record]   TODO を全部埋めるまでは起動できない(埋め忘れを防ぐため)" >&2
  echo "[record] 条件を残さない dry run なら SCENE=none" >&2
  exit 1
fi
if [ "$SCENE" != "none" ]; then
  SCENE_PATH="$REPO/scenes/${SCENE}.yaml"
  if [ ! -f "$SCENE_PATH" ]; then
    echo "[record] シーンプロファイルが無い: $SCENE_PATH" >&2
    echo "[record] 新しい環境なら、雛形をコピーして作る:" >&2
    echo "[record]   cp scenes/_TEMPLATE.yaml \"scenes/${SCENE}.yaml\" && \$EDITOR \"scenes/${SCENE}.yaml\"" >&2
    echo "[record] 実機の前で TODO を全部埋めてから、もう一度この起動を打つ。" >&2
    echo "[record] 既存の環境で録るつもりなら、名前の綴りを確認する:" >&2
    ls -1 "$REPO/scenes" 2>/dev/null | grep -v '^_' | sed 's/\.yaml$//; s/^/[record]   /' >&2
    exit 1
  fi
fi

# manifest を書く python。lerobot の定数(HF_LEROBOT_HOME・較正ルート)は lerobot
# 自身から読ませたいので、lerobot-record と同じインタプリタを使う(shebang から取る)。
# lerobot-record は PATH に無ければ lerobot venv のものを使う (Thor は venv 運用。
# 兄弟スクリプト run_teleop_remote_*.sh と同じ流儀)。
LEROBOT_RECORD="${LEROBOT_RECORD:-}"
if [ -z "$LEROBOT_RECORD" ]; then
  if command -v lerobot-record >/dev/null 2>&1; then
    LEROBOT_RECORD=lerobot-record
  else
    LEROBOT_RECORD="$HOME/projects/lerobot/.venv/bin/lerobot-record"
  fi
fi
if ! command -v "$LEROBOT_RECORD" >/dev/null 2>&1 && [ ! -x "$LEROBOT_RECORD" ]; then
  echo "[record] NG: lerobot-record が見つからない ($LEROBOT_RECORD)" >&2
  exit 1
fi
# venv をフルパスで呼ぶ場合、rerun ビューア等の同梱バイナリを rr.spawn が PATH から
# 探すため、venv の bin を PATH に足す (無いと display_data=true が起動時に落ちる)。
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

# ---- 同期負債の警告（録る前に「古いコードで録っていないか」を見える化） --------
# ブロックしない。現場の Thor はオフラインがあり、古いコードで録ること自体は
# manifest に commit/branch/behind が残るので追跡できる。防ぎたいのは
# **黙って**古いまま録ること（実例: ←破棄の根治が push 済みなのに録画機に
# 届いておらず、誰もそれを知らなかった＝2026-09-01）。SKIP_SYNC_CHECK=true で省略可。
if [ "${SKIP_SYNC_CHECK:-false}" != "true" ] && [ -f "$REPO/scripts/check_repo_sync.sh" ]; then
  LEROBOT_ROOT="$("$PYTHON_BIN" -c 'import lerobot, pathlib; print(pathlib.Path(lerobot.__file__).resolve().parents[2])' 2>/dev/null || true)"
  for _r in "$REPO" ${LEROBOT_ROOT:+"$LEROBOT_ROOT"}; do
    bash "$REPO/scripts/check_repo_sync.sh" "$_r" >&2 || true
  done
fi

# TODO が残ったままでは録らせない。TODO 版の sha でセッションを記録すると、
# あとで実測値を入れた時に sha が変わり、そのセッションだけ「もう存在しない版の
# 条件」を指したまま孤立する(遡って直せない)。
if [ -n "$SCENE_PATH" ]; then
  TODO_N="$("$PYTHON_BIN" "$MANIFEST_TOOL" check-scene --scene-path "$SCENE_PATH" 2>/dev/null || echo "")"
  if printf '%s' "$TODO_N" | grep -qE '^[0-9]+$' && [ "$TODO_N" -gt 0 ]; then
    if [ "${ALLOW_TODO_SCENE:-false}" != "true" ]; then
      echo "[record] シーンプロファイルに未確定(TODO)が $TODO_N 件残っている: $SCENE_PATH" >&2
      echo "[record] 実測値を入れてから録ること。TODO 版で録ると、あとで埋めた時に sha が" >&2
      echo "[record] 変わり、このセッションだけ条件を辿れなくなる。" >&2
      echo "[record] 条件を残さない dry run なら ALLOW_TODO_SCENE=true" >&2
      exit 1
    fi
    echo "[record] 警告: TODO $TODO_N 件のシーンで録る(ALLOW_TODO_SCENE=true)。条件は復元できない。" >&2
  fi

  # 同じシーン名で前に録った run と sha が食い違っていたら止める。
  # 「1本録ったらシーンを書き換えず新ファイルを作る」は規約として書いてあるが、
  # 機構が無いと守られたか誰も確認できない。50ep は複数セッション・複数日に分かれる
  # ので、途中で実測値を1つ直すと前後で sha が割れ、**各 run の記録は正しいまま
  # 「50ep が実は2条件に割れていた」と学習の分析時に判明する**。そこまで行くと
  # 録り直しでしか直せないので、起動時に弾く。
  DRIFT="$("$PYTHON_BIN" "$MANIFEST_TOOL" check-drift --scene-path "$SCENE_PATH" \
            --collections-dir "$(dirname "$RUN_DIR")" 2>/dev/null || echo "")"
  if [ -n "$DRIFT" ]; then
    set -- $DRIFT
    if [ "${ALLOW_SCENE_DRIFT:-false}" != "true" ]; then
      echo "[record] シーン '$SCENE' が前回の収録時と変わっている（sha が割れた）" >&2
      echo "[record]   前回 $2 （run: $4）" >&2
      echo "[record]   今回 $3" >&2
      echo "[record] このまま録ると 50ep が2つの条件に割れ、あとから直せない。" >&2
      echo "[record] 物理配置を変えたなら**新しいシーンファイル**を作って SCENE を変える。" >&2
      echo "[record] 変えたつもりが無いなら git diff で何が動いたか確認する。" >&2
      echo "[record] 承知で続けるなら ALLOW_SCENE_DRIFT=true" >&2
      exit 1
    fi
    echo "[record] 警告: シーン sha が前回と違う（$2 → $3・ALLOW_SCENE_DRIFT=true）" >&2
  fi
fi

# 開始姿勢セットの照合（2026-09-30）。draft のセットなら今の較正・シーン・コードの版を取り込み、
# frozen のセットなら開始姿勢・較正・シーンが凍結時と 1 つでも違えば止める（条件を変えたら新しいセット）。
pose_set_prepare_record "$SCENE" || exit 1

# 開始姿勢 YAML も同じ穴を持つ(シーンと違い名前は変えず内容を撮り直しうる)。
# 撮り直すと50epが2条件に割れても各runの記録自体は正しいまま学習の分析時まで
# 発覚しない。ファイルが無ければ公式既定(肘36°)にフォールバックするだけなので
# チェックしない(較正の有無と同じ扱い)。
if [ -f "$POSE" ]; then
  POSE_DRIFT="$("$PYTHON_BIN" "$MANIFEST_TOOL" check-pose-drift --initial-pose-path "$POSE" \
            --collections-dir "$(dirname "$RUN_DIR")" 2>/dev/null || echo "")"
  if [ -n "$POSE_DRIFT" ]; then
    set -- $POSE_DRIFT
    if [ "${ALLOW_POSE_DRIFT:-false}" != "true" ]; then
      echo "[record] 開始姿勢 '$POSE' が前回の収録時と変わっている（sha が割れた）" >&2
      echo "[record]   前回 $2 （run: $4）" >&2
      echo "[record]   今回 $3" >&2
      echo "[record] このまま録ると 50ep が2つの条件に割れ、あとから直せない。" >&2
      echo "[record] 撮り直したなら承知で ALLOW_POSE_DRIFT=true。誤って上書きしたなら復元する。" >&2
      exit 1
    fi
    echo "[record] 警告: 開始姿勢 sha が前回と違う（$2 → $3・ALLOW_POSE_DRIFT=true）" >&2
  fi
fi

# ---- run_dir: 1収集セッション = 1ディレクトリ --------------------------------
# TAG/TS/RUN_DIR の値はすでに決めてある(上の POSE ブロック直後。drift チェックが
# 必要とするため)。ここでは実際の存在チェックと作成だけを行う。
#
# なぜ tee が要るか: lerobot は起動時に設定の全文をダンプするが、それは stdout で
# なく stderr に出る。`2>&1` を付けないと console.log に1行も入らない。さらに
# パイプ越しだと Python が出力をブロックバッファするので PYTHONUNBUFFERED=1 が
# 無いと途中まで書かれず、異常終了時に肝心の直前ログが失われる。
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
  if [ "$REMOTE" = "true" ]; then
    echo "[record] side=$SIDE  follower=$FOLLOWER_PORT  teleop=REMOTE (zmq:$ZMQ_PORT echo:$ECHO_PORT / leader_agent は操作側PC)"
    echo "[record]   'a' = ローカル同様に leader も戻る (echo 経路で leader_agent に指令。不達時は手戻しフォールバック)"
  else
    echo "[record] side=$SIDE  follower=$FOLLOWER_PORT  leader=$LEADER_PORT"
  fi
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

# ---- manifest.json ------------------------------------------------------------
# manifest は設定の全文を持たない。lerobot が起動時に解決済み設定を丸ごとダンプ
# していて、それは console.log に入っている。ここに残すのはダンプから取れない
# 事実だけ(コードの版・較正の sha・stamp 後の repo_id・python/pinocchio 版)。
# 書けなくても収集は止めない: 目の前に実機と人が居る場面で、メタデータの不調で
# 収録を落とす方が損失が大きい。代わりに警告を console.log に残す。
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
  --initial-pose-path "$POSE"
  --side "$SIDE"
  --cam-serial "$CAM_SERIAL"
  --fps "$FPS"
  --gravity-scale "$GRAVITY_SCALE"
  --repo-id "$REPO_ID"
  --repo-root "$REPO"
)
if [ "$WRIST" = "true" ]; then
  MANIFEST_ARGS+=(--wrist-serial "$WRIST_SERIAL")
fi
MANIFEST_INIT_OUT="$RUN_DIR/.manifest-init.out"
MANIFEST_OK=true
if ! "$PYTHON_BIN" "$MANIFEST_TOOL" "${MANIFEST_ARGS[@]}" > "$MANIFEST_INIT_OUT" 2>&1; then
  MANIFEST_OK=false
fi

# ---- command.sh --------------------------------------------------------------
# 同じ条件で起動し直すための行。フラグの全列挙はしない(設定ダンプと二重になり
# ドリフト源になる)。環境変数の解決値だけを allowlist で残し、詳細は console.log
# と manifest.json を指す。ダンプが落とす type 判別子はコメントで明示する。
if [ "$WRIST" = "true" ]; then
  CAM_NOTE="chest D435i + wrist_${SIDE} D405 (type=intelrealsense)"
else
  CAM_NOTE="chest D435i (type=intelrealsense)"
fi
COMMAND_SH="$RUN_DIR/command.sh"
{
  echo '#!/usr/bin/env bash'
  echo "# 自動生成: このセッションを同じ条件で起動し直す行。"
  echo "# 解決済み設定の全文 -> console.log / 環境スナップショット -> manifest.json"
  if [ "$REMOTE" = "true" ]; then
    echo "# 設定ダンプが落とす type 判別子: robot.type=openarm_follower / teleop.type=openarm_leader_remote"
  else
    echo "# 設定ダンプが落とす type 判別子: robot.type=openarm_follower / teleop.type=openarm_leader"
  fi
  echo "# カメラ: $CAM_NOTE"
  echo "set -euo pipefail"
  printf 'OPERATOR=%q \\\n' "$OPERATOR"
  printf 'JUDGE=%q \\\n' "$JUDGE"
  printf 'SCENE=%q \\\n' "$SCENE"
  printf 'SIDE=%q \\\n' "$SIDE"
  printf 'REMOTE=%q ZMQ_PORT=%q ECHO_PORT=%q \\\n' "$REMOTE" "$ZMQ_PORT" "$ECHO_PORT"
  printf 'CAM_SERIAL=%q CHEST_V4L2=%q \\\n' "$CAM_SERIAL" "$CHEST_V4L2"
  printf 'WRIST=%q WRIST_SERIAL=%q \\\n' "$WRIST" "$WRIST_SERIAL"
  printf 'CAMERA_W=%q CAMERA_H=%q FPS=%q \\\n' "$CAMERA_W" "$CAMERA_H" "$FPS"
  printf 'NUM_EPISODES=%q EPISODE_TIME_S=%q RESET_TIME_S=%q \\\n' "$NUM_EPISODES" "$EPISODE_TIME_S" "$RESET_TIME_S"
  printf 'REPO_ID=%q \\\n' "$REPO_ID"
  printf 'TASK=%q \\\n' "$TASK"
  printf 'GRAVITY_COMP=%q GRAVITY_SCALE=%q GRAVITY_URDF=%q \\\n' "$GRAVITY_COMP" "$GRAVITY_SCALE" "$GRAVITY_URDF"
  printf 'EPISODE_ADVANCE=%q START_POSE_TOL_DEG=%q START_POSE_MAX_WAIT_S=%q \\\n' \
    "$EPISODE_ADVANCE" "$START_POSE_TOL_DEG" "$START_POSE_MAX_WAIT_S"
  printf 'DISPLAY_DATA=%q \\\n' "$DISPLAY_DATA"
  printf 'bash scripts/record_pick_place_openarm.sh %q\n' "$TAG"
} > "$COMMAND_SH"
chmod +x "$COMMAND_SH"

{
  sed 's/^/[record] manifest: /' "$MANIFEST_INIT_OUT"
  if [ "$MANIFEST_OK" != true ]; then
    echo "[record] 警告: manifest.json を書けなかった。環境スナップショットが残らない。"
  fi
  echo "[record] command.sh: $COMMAND_SH"
} 2>&1 | tee -a "$CONSOLE_LOG"
rm -f "$MANIFEST_INIT_OUT"

# set -e を一時的に外す: pipefail 下で lerobot が非0終了すると、ここで即座に
# 抜けて終了メッセージも終了コードの伝播も走らなくなる。PIPESTATUS[0] で
# lerobot 自身の終了コードを取り(tee の 0 で隠さない)、最後に伝播させる。
# teleop 引数: local = leader 実機を Thor の CAN で直接駆動 / remote = ZMQ 受信
if [ "$REMOTE" = "true" ]; then
  TELEOP_ARGS=(
    --teleop.type=openarm_leader_remote
    --teleop.id="leader_${SIDE}_remote"
    --teleop.zmq_port="$ZMQ_PORT"
    --teleop.echo_port="$ECHO_PORT"
  )
else
  TELEOP_ARGS=(
    --teleop.type=openarm_leader
    --teleop.port="$LEADER_PORT"
    --teleop.id="leader_${SIDE}"
    --teleop.gravity_compensation="$GRAVITY_COMP"
    --teleop.gravity_urdf_path="$GRAVITY_URDF"
    --teleop.gravity_side="$SIDE"
    --teleop.gravity_scale="$GRAVITY_SCALE"
  )
fi

set +e
"$LEROBOT_RECORD" \
  --robot.type=openarm_follower \
  --robot.port="$FOLLOWER_PORT" \
  --robot.side="$SIDE" \
  --robot.id="follower_${SIDE}" \
  --robot.cameras="$CAMERAS" \
  "${TELEOP_ARGS[@]}" \
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

# stamp 後の実 repo_id は、設定ダンプが stamp_repo_id() より前に出る構造なので
# 実行後にしか分からない。ここでファイルシステムから引いて manifest に追記する
# (stamp はローカル時刻・run_dir は UTC 名なので、文字列では突き合わせられない)。
MANIFEST_RESOLVE_OUT="$RUN_DIR/.manifest-resolve.out"
if ! "$PYTHON_BIN" "$MANIFEST_TOOL" resolve --run-dir "$RUN_DIR" --exit-code "$RC" \
     > "$MANIFEST_RESOLVE_OUT" 2>&1; then
  echo "[record] 警告: manifest の事後解決に失敗した(stamp 後 repo_id が未記録)" >> "$MANIFEST_RESOLVE_OUT"
fi

# 本番の収集でデータセットが作られたら、セットを凍結する（以後この組は変えない・2026-09-30）。
pose_set_freeze "$RUN_DIR" 2>&1 | tee -a "$CONSOLE_LOG"

{
  echo "[record] lerobot-record exit=$RC"
  sed 's/^/[record] manifest: /' "$MANIFEST_RESOLVE_OUT"
  echo "[record] 記録: $RUN_DIR"
  ls -1 "$RUN_DIR" | sed 's/^/[record]   /'
  echo "[record] 次: 軽量メタを Vault へ回収する(pull_run.sh・未実装=バックログ4)。"
  echo "[record]     ledger.csv / notes.md / setup/ も未実装(同4)。"
  echo "[record]     manifest.json の human_fields(散らした範囲・異常)は人が埋める。"
} 2>&1 | tee -a "$CONSOLE_LOG"
rm -f "$MANIFEST_RESOLVE_OUT"

exit "$RC"
