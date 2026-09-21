#!/usr/bin/env bash
# 両腕収集の短い入口。長い env 列を端末に貼ると行割れで事故る（既知）ので、
# 固定値は config/bimanual-collect.env に置き、ここは「モード・タグ・本数」だけ受ける。
# 本体は scripts/record_bimanual_openarm.sh（運用契約・ゲート・manifest はそちら）。
#
# 使い方（Thor・~/projects/openarm-orboh で）:
#   bash scripts/collect_bimanual.sh dry                 # 2 本・捨て名・SCENE=none・表示なし（配管と 'a' キーの確認）
#   bash scripts/collect_bimanual.sh gate                # 受入ゲート 10 本（タグ <slug>-v1-s1）→ 止めて検査
#   bash scripts/collect_bimanual.sh session s2 [10]     # 続きのセッション（タグ <slug>-v1-s2・本数既定 10）
#   RESUME=true DATASET_ROOT=... bash scripts/collect_bimanual.sh session s2   # 既存データセットへ追記（本体の契約どおり）
#   DISPLAY_IP=<手元PC> DISPLAY_PORT=9876 bash scripts/collect_bimanual.sh gate # 手元の rerun へ映像を飛ばす
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
    export SCENE=none NUM_EPISODES="$NUM" DISPLAY_DATA="${DISPLAY_DATA:-false}"
    # 捨て名（データ標準 §4: _test_<unixtime>）。本体の repo_id 自動組立を上書きする
    OBS=chest; [ "${WRIST_RIGHT:-true}" = "true" ] && OBS="${OBS}_wristr"; [ "${WRIST_LEFT:-false}" = "true" ] && OBS="${OBS}_wristl"
    export REPO_ID="Orboh/bi_openarm_${TASK_SLUG}_${OBS}_test_$(date +%s)"
    ;;
  gate)
    SUFFIX="v1-s1"; NUM="${2:-10}"
    export NUM_EPISODES="$NUM" DISPLAY_DATA="${DISPLAY_DATA:-false}"
    ;;
  session)
    [ -n "${2:-}" ] || { echo "[collect-bi] session にはタグ接尾辞（例 s2）が要る" >&2; exit 1; }
    SUFFIX="v1-$2"; NUM="${3:-10}"
    export NUM_EPISODES="$NUM" DISPLAY_DATA="${DISPLAY_DATA:-false}"
    ;;
esac
TAG="bi-${TASK_SLUG}-${SUFFIX}"

export OPERATOR TASK TASK_SLUG EPISODE_TIME_S LSCALE RSCALE
export CAM_SERIAL="${CHEST_SERIAL:-344422070898}"
export WRIST_RIGHT="${WRIST_RIGHT:-true}" WRIST_RIGHT_SERIAL WRIST_LEFT WRIST_LEFT_SERIAL

echo "[collect-bi] mode=$MODE tag=$TAG episodes=$NUM_EPISODES scene=$SCENE task='$TASK' cams=chest+$( [ "$WRIST_RIGHT" = true ] && printf 'wrist_right' )$( [ "$WRIST_LEFT" = true ] && printf '+wrist_left' ) repo_id=${REPO_ID:-<auto>}"
# RealSense は SDK 直読が排他: モニター用プレビューが動いていれば止める。次いで D405 の露出/ゲインを焼く
bash "$REPO/scripts/preview_on_monitor.sh" stop-preview >/dev/null 2>&1 || true
_cs=()
[ -n "${D405_GAIN:-}" ] && _cs+=(--d405-gain "$D405_GAIN" --d405-exposure "${D405_EXPOSURE:-28000}")
[ -n "${D435_BRIGHTNESS:-}" ] && _cs+=(--d435-brightness "$D435_BRIGHTNESS")
if [ ${#_cs[@]} -gt 0 ]; then
  sleep 2
  "$HOME/projects/lerobot/.venv/bin/python" "$REPO/scripts/camera_setup.py" "${_cs[@]}" || { echo "[collect-bi] camera_setup（${_cs[*]}）に失敗。カメラを掴んでいるプロセスが無いか確認" >&2; exit 1; }
fi
export D405_GAIN D405_EXPOSURE D435_BRIGHTNESS
# pynput が無言でキーを食う既知問題の予防（ssh -X でも DISPLAY を落として stdin 直読みにする）
exec env -u DISPLAY -u WAYLAND_DISPLAY bash "$REPO/scripts/record_bimanual_openarm.sh" "$TAG"
