#!/usr/bin/env bash
# 両腕収集の短い入口。長い env 列を端末に貼ると行割れで事故る（既知）ので、
# 固定値は config/bimanual-collect.env に置き、ここは「モード・タグ・本数」だけ受ける。
# 本体は scripts/record_bimanual_openarm.sh（運用契約・ゲート・manifest はそちら）。
#
# 使い方（Thor・~/projects/openarm-orboh で）:
#   bash scripts/collect_bimanual.sh dry                 # 2 本・捨て名・SCENE=none（配管と 'a' キーの確認）
#   bash scripts/collect_bimanual.sh gate                # 受入ゲート 10 本（タグ <slug>-v1-s1）→ 止めて検査
#   bash scripts/collect_bimanual.sh session s2 [10]     # 続きのセッション（タグ <slug>-v1-s2・本数既定 10）
#   RESUME=true DATASET_ROOT=... bash scripts/collect_bimanual.sh session s2   # 既存データセットへ追記（本体の契約どおり）
#   DISPLAY_IP=<手元PC> DISPLAY_PORT=9876 bash scripts/collect_bimanual.sh gate # 手元の rerun へ映像を飛ばす
#   MONITOR=off bash scripts/collect_bimanual.sh gate      # record 中の映像をモニターに出さない（既定は on＝Thor のビューア :9876 へ 3 眼を流す・2026-09-22 Kento。関節値は 2026-10-05 から既定で送らない）
#   DISPLAY_SCALARS=true bash scripts/collect_bimanual.sh gate   # 関節値の時系列もモニターに流す（9/22 にビューアが数分で固まった挙動。表示のみ）
#   GRIPPER_KP=12 bash scripts/collect_bimanual.sh gate      # follower gripper の kp を下げる(熱対策・2026-09-21。env に書いてもよい)
#   DISPLAY_SCALE=1 DISPLAY_JPEG_QUALITY=95 bash scripts/collect_bimanual.sh gate   # 表示用画像を間引かず原寸・q95 で流す（既定は 1/2・q60。表示のみ、データは無関係）
#   DISPLAY_STRIDE=3 bash scripts/collect_bimanual.sh gate   # 表示用画像を 3 フレームに 1 回だけ流す（関節値は毎フレーム）
#   VIEWER_KEEP=true bash scripts/collect_bimanual.sh gate   # 起動時にビューアを作り直さない（既定は作り直す＝古い recording が前面に残って映像が止まって見える事故の防止・2026-09-22）
#   DMLOG=off bash scripts/collect_bimanual.sh gate          # Damiao CAN フィードバックの自動記録を切る（既定は on＝record が裏で受信専用ロガーを起こし、
#                                                            #   run_dir に dmlog-<tag>.csv・エピソード境界の MARK・要約を残す・2026-10-05。別端末のロガーは不要）
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
ENV_FILE="${ENV_FILE:-$REPO/config/bimanual-collect.env}"
[ -f "$ENV_FILE" ] || { echo "[collect-bi] 設定ファイルが無い: $ENV_FILE" >&2; exit 1; }
# env ファイルは「既定値」。呼び出し側で明示した変数（空文字を含む）はそちらを優先する
# （例: D405_GAIN= で今回だけ焼かない、OPERATOR=Kota で録る人を変える）。
declare -A _caller=()
for _v in $(sed -n 's/^\([A-Z_][A-Z0-9_]*\)=.*/\1/p' "$ENV_FILE"); do
  if [ -n "${!_v+x}" ]; then _caller["$_v"]="${!_v}"; fi
done
# shellcheck disable=SC1090
set -a; source "$ENV_FILE"; set +a
for _v in "${!_caller[@]}"; do printf -v "$_v" '%s' "${_caller[$_v]}"; export "$_v"; done

MODE="${1:-}"
case "$MODE" in
  dry|gate|session) ;;
  *) echo "[collect-bi] usage: bash scripts/collect_bimanual.sh <dry|gate|session> [tag-suffix] [num_episodes]" >&2; exit 1 ;;
esac

# TODO が残る設定では録らない（dry は SCENE=none で物理条件を持たないので TASK/TASK_SLUG だけ見る）
if printf '%s' "${TASK:-}" | grep -q '^TODO' || [ "${TASK_SLUG:-TODO}" = "TODO" ]; then
  echo "[collect-bi] $ENV_FILE の TASK / TASK_SLUG が未記入(TODO)。学習と rollout で同じ文を使うので先に決める。" >&2
  exit 1
fi
if [ "$MODE" != "dry" ] && [ "${SCENE:-TODO}" = "TODO" ]; then
  echo "[collect-bi] $ENV_FILE の SCENE が未記入(TODO)。scenes/<name>.yaml を実測で作ってから。" >&2
  exit 1
fi

case "$MODE" in
  dry)
    SUFFIX="dry"; NUM="${3:-${2:-2}}"
    export SCENE=none NUM_EPISODES="$NUM"
    # 捨て名（データ標準 §4: _test_<unixtime>）。本体の repo_id 自動組立を上書きする
    OBS=chest; [ "${WRIST_RIGHT:-true}" = "true" ] && OBS="${OBS}_wristr"; [ "${WRIST_LEFT:-false}" = "true" ] && OBS="${OBS}_wristl"
    export REPO_ID="Orboh/bi_openarm_${TASK_SLUG}_${OBS}_test_$(date +%s)"
    ;;
  gate)
    SUFFIX="v1-s1"; NUM="${2:-10}"
    export NUM_EPISODES="$NUM"
    ;;
  session)
    [ -n "${2:-}" ] || { echo "[collect-bi] session にはタグ接尾辞（例 s2）が要る" >&2; exit 1; }
    SUFFIX="v1-$2"; NUM="${3:-10}"
    export NUM_EPISODES="$NUM"
    ;;
esac
# scene-tag は英数ハイフンのみ（record_bimanual_openarm.sh の検証）。TASK_SLUG は repo_id に
# 入る名前なので下線を使う（命名標準 §4: dataset は <robot>_<task>_<obs>_vN）。両者は別レイヤー
# なので、タグを作るときだけ下線をハイフンへ寄せる。ここを揃えないと TASK_SLUG=blockstack_leftarm
# のような下線入りの slug で gate の起動が即エラーになる（2026-09-23）。
TAG="bi-${TASK_SLUG//_/-}-${SUFFIX}"

# record 中の映像出力（2026-09-22 Kento 決定: 既定オン）。record 自身が Thor のビューア（rerun :9876）へ
# 3 眼と関節値を流す。プレビュー用プロセスとは RealSense が排他なので、下でプレビューを止めビューアだけ確保する。
#   MONITOR=off          → 流さない（DISPLAY_DATA=false）
#   DISPLAY_IP=<手元PC>  → 手元 PC の rerun へ（従来どおり・MONITOR より優先）
MONITOR="${MONITOR:-on}"
MON_PORT="${DISPLAY_PORT:-9876}"
if [ -n "${DISPLAY_IP:-}" ]; then
  export DISPLAY_DATA=true DISPLAY_PORT="$MON_PORT"; MON_NOTE="remote(${DISPLAY_IP}:${MON_PORT})"
elif [ "$MONITOR" = "off" ] || [ "${DISPLAY_DATA:-}" = "false" ]; then
  export DISPLAY_DATA=false; unset DISPLAY_IP DISPLAY_PORT; MON_NOTE="off"
elif PORT="$MON_PORT" bash "$REPO/scripts/preview_on_monitor.sh" "$( [ "${VIEWER_KEEP:-false}" = "true" ] && echo start-viewer || echo restart-viewer )"; then
  # ビューアは起動ごとに作り直す（restart-viewer）。rerun は最後に届いた recording を前面に出すので、前の run や
  # テストの recording が残っていると record の映像が裏に入る（2026-09-22 gate 中に 3 回「モニターが止まった」）。
  # record 側の recording id も固定して（LEROBOT_RERUN_RECORDING_ID）、同じビューア内で run が増えないようにする。
  export DISPLAY_DATA=true DISPLAY_IP=127.0.0.1 DISPLAY_PORT="$MON_PORT" LEROBOT_RERUN_RECORDING_ID="${DISPLAY_RECORDING_ID:-openarm-record}"; MON_NOTE="on(127.0.0.1:${MON_PORT})"
else
  echo "[collect-bi] ビューアを起動できないので映像出力なしで続ける（MONITOR=off と同じ）" >&2
  export DISPLAY_DATA=false; unset DISPLAY_IP DISPLAY_PORT; MON_NOTE="off(viewer unavailable)"
fi

# 表示用画像の間引きと JPEG 品質（2026-09-22・dry5）。lerobot は DISPLAY_IP 指定時に JPEG を強制する（upstream の
# 遠隔／Web ビューア向け帯域規則）。640x480 を q95 で毎サイクル 3 眼エンコードすると record ループが 30 Hz を割った
# （22% のサイクルで 1〜4 ms 超過）。設計書 thor-camera-monitor-current §3.4 のプレビューと同じ 320x240・q60 を既定に
# する。データセットには影響しない（表示コピーのみ）。
if [ "$DISPLAY_DATA" = true ]; then
  export LEROBOT_DISPLAY_IMAGE_SCALE="${DISPLAY_SCALE:-2}" LEROBOT_DISPLAY_JPEG_QUALITY="${DISPLAY_JPEG_QUALITY:-60}" LEROBOT_DISPLAY_IMAGE_STRIDE="${DISPLAY_STRIDE:-1}"
  # 関節値はモニターに送らず映像だけにする（2026-10-05）。映像は lerobot が「最新 1 枚」（static）で送るので溜まらないが、
  # 関節値は約 40 本の時系列として 30 Hz で溜まり続け、ビューアが全履歴を毎フレーム描き直して数分で CPU 150〜250% に
  # 張り付きモニターが固まった（2026-09-22 gate・精密記録 §2 の 46）。表示のみでデータセットには影響しない。
  # 関節値のグラフも見たいときは DISPLAY_SCALARS=true（9/22 と同じ挙動に戻る）。
  export LEROBOT_DISPLAY_SCALARS="${DISPLAY_SCALARS:-false}"
  MON_NOTE="$MON_NOTE 1/${LEROBOT_DISPLAY_IMAGE_SCALE} q${LEROBOT_DISPLAY_JPEG_QUALITY} stride${LEROBOT_DISPLAY_IMAGE_STRIDE} scalars=${LEROBOT_DISPLAY_SCALARS}"
fi
export OPERATOR TASK TASK_SLUG EPISODE_TIME_S LSCALE RSCALE
export CAM_SERIAL="${CHEST_SERIAL:-344422070898}"
export WRIST_RIGHT="${WRIST_RIGHT:-true}" WRIST_RIGHT_SERIAL WRIST_LEFT WRIST_LEFT_SERIAL

echo "[collect-bi] mode=$MODE tag=$TAG episodes=$NUM_EPISODES scene=$SCENE task='$TASK' cams=chest+$( [ "$WRIST_RIGHT" = true ] && printf 'wrist_right' )$( [ "$WRIST_LEFT" = true ] && printf '+wrist_left' ) repo_id=${REPO_ID:-<auto>} monitor=$MON_NOTE"
# RealSense は SDK 直読が排他: モニター用プレビューが動いていれば止める。次いで D405 の露出/ゲインを焼く
bash "$REPO/scripts/preview_on_monitor.sh" stop-preview >/dev/null 2>&1 || true
# USB 省電力を止める（udev ルールの保険。再列挙で auto に戻ることがある）
for _d in /sys/bus/usb/devices/*; do
  [ -f "$_d/idVendor" ] || continue
  case "$(cat "$_d/idVendor"):$(cat "$_d/idProduct")" in 05e3:0626|8086:0b3a|8086:0b5b) echo on | sudo -n tee "$_d/power/control" >/dev/null 2>&1 ;; esac
done
_cs=()
if [ -n "${D405_GAIN:-}" ]; then _cs+=(--d405-gain "$D405_GAIN" --d405-exposure "${D405_EXPOSURE:-28000}"); else _cs+=(--d405-auto); fi   # 空なら前セッションの手動値を自動露出へ戻す
[ -n "${D435_BRIGHTNESS:-}" ] && _cs+=(--d435-brightness "$D435_BRIGHTNESS")
if [ ${#_cs[@]} -gt 0 ]; then
  sleep 2
  "$HOME/projects/lerobot/.venv/bin/python" "$REPO/scripts/camera_setup.py" "${_cs[@]}" || { echo "[collect-bi] camera_setup（${_cs[*]}）に失敗。カメラを掴んでいるプロセスが無いか確認" >&2; exit 1; }
fi
export D405_GAIN D405_EXPOSURE D435_BRIGHTNESS
# pynput が無言でキーを食う既知問題の予防（ssh -X でも DISPLAY を落として stdin 直読みにする）
exec env -u DISPLAY -u WAYLAND_DISPLAY bash "$REPO/scripts/record_bimanual_openarm.sh" "$TAG"
