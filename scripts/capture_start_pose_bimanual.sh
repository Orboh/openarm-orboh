#!/usr/bin/env bash
# 両腕の開始姿勢と立ち上がり経路を、テレオペ 1 回で撮って新しい開始姿勢セット（draft）にする。
#
#   bash scripts/capture_start_pose_bimanual.sh <新しいセット名> [シーン名] [run_teleop_bimanual.sh の引数...]
#   例: bash scripts/capture_start_pose_bimanual.sh takeei-v2
#
# 流れ（操作者がするのは 3 と 6 だけ）:
#   1. CAN 4 系統の UP とセット名の重複を確かめる（DOWN なら sudo bash scripts/canup.sh を案内して終わる）
#   2. テレオペ（scripts/run_teleop_bimanual.sh）を起動する。整列先は CURRENT ではなく、撮影専用の固定の
#      出発姿勢 capture-base（腕を下ろした構え）に固定する。CURRENT に依存させると、撮り直すたびに前回撮った
#      姿勢へ一度上がることになり、同じ手順でも出発の動きが変わる（再現性が無い）。CAPTURE_BASE で差し替え可
#   3. 整列中は腕に触らない。4 ユニットの "connected." が揃った時点（整列完了）で記録を自動で始める
#      ＝整列中の静止を開始姿勢として拾わない（以前は --since を手で渡していた）
#   4. 操作者: 腕を下ろして 3〜5 秒止める → 通したい経路で開始姿勢まで動かす → 15 秒止める → 少し下ろして Ctrl-C
#      （このテレオペは出発姿勢 capture-base で動くので、Ctrl-C 時の「戻り」は起きない＝経由地が無い）
#   5. 記録を止め、開始姿勢・経路・経由地・警告（可動域の端・手首のねじれ）を表示する
#   6. 操作者: 書き出すか（y/N）、CURRENT にするか（y/N）を答える
#   7. 記録 CSV とテレオペのログをセットの evidence/ に残す（不採用なら ~/collections に -failed で残す）
# 設計の正本: Vault 07Member's-free-space/Kento/01-design/openarm-start-pose-determination-process.md
set -uo pipefail

SET="${1:-}"; SCENE="${2:-}"
[ -n "$SET" ] || { sed -n '2,4p' "$0"; exit 1; }
shift; [ $# -gt 0 ] && shift
REPO="$(cd "$(dirname "$0")/.." && pwd)"
PY="$HOME/projects/lerobot/.venv/bin/python"
CAPTURE_BASE="${CAPTURE_BASE:-capture-base}"
cd "$REPO"
[ -f "config/pose_sets/$CAPTURE_BASE/set.yaml" ] || { echo "[capture] 出発姿勢のセット $CAPTURE_BASE が無い" >&2; exit 1; }

if [ -e "config/pose_sets/$SET" ]; then
  echo "[capture] セット $SET は既にある（別の名前で撮る）" >&2; exit 1
fi
if [ -n "$SCENE" ] && [ ! -f "scenes/$SCENE.yaml" ]; then
  echo "[capture] scenes/$SCENE.yaml が無い（シーンは後から python3 scripts/pose_set.py sync $SET --scene <名前> でもよい）" >&2; exit 1
fi
for i in can_fol_l can_fol_r can_lead_l can_lead_r; do
  ip -br link show "$i" 2>/dev/null | grep -q " UP " || { echo "[capture] $i が DOWN。先に: sudo bash scripts/canup.sh" >&2; exit 1; }
done

TS="$(date +%Y%m%d-%H%M%S)"
mkdir -p "$HOME/collections"
CSV="$HOME/collections/pose-$SET-$TS.csv"
LOG="$HOME/collections/pose-$SET-$TS.teleop.log"
: > "$LOG"

cat <<MSG
[capture] セット $SET を撮る（整列先 = 撮影の出発姿勢 $CAPTURE_BASE。CURRENT は書き出し時まで触らない）
[capture] 1) 整列中は腕に触らない。整列が終わると「記録開始」と出る
[capture] 2) 腕を下ろして 3〜5 秒止める
[capture] 3) 通したい経路で開始姿勢までゆっくり動かす（肘は畳み切らない・持ち手をねじらない）
[capture] 4) 開始姿勢で 15 秒止める（記録の中で一番長い静止にする）
[capture] 5) 少し下ろしてから Ctrl-C（止める前に腕を支える）
MSG

# 整列完了（4 ユニットの "<name> connected." が揃う）を待って受信専用の記録を始める
(
  while [ "$(grep -c ' connected\.' "$LOG" 2>/dev/null)" -lt 4 ]; do sleep 0.2; done
  echo; echo "[capture] ===== 整列完了。記録開始（$CSV）====="
  exec "$PY" -u scripts/openarm_pose_monitor.py --record "$CSV" --quiet > "$CSV.monitor.log" 2>&1
) &
REC=$!

trap 'true' INT   # Ctrl-C はテレオペにだけ効かせ、このスクリプトは後処理へ進む
POSE_SET="$CAPTURE_BASE" bash scripts/run_teleop_bimanual.sh "$@" 2>&1 | tee -i "$LOG"
trap - INT
kill -INT "$REC" 2>/dev/null; sleep 1; kill "$REC" 2>/dev/null; wait "$REC" 2>/dev/null

rows=$(( $(wc -l < "$CSV" 2>/dev/null || echo 1) - 1 ))
if [ "$rows" -le 0 ]; then
  echo "[capture] 記録が空（整列が完了しなかったか、テレオペが途中で落ちた）。ログ: $LOG" >&2; exit 1
fi
echo; echo "[capture] 記録 $rows 行。抽出する:"
"$PY" scripts/pose_from_dwell.py "$CSV" || { echo "[capture] 抽出に失敗。CSV は残した: $CSV" >&2; exit 1; }

read -r -p "[capture] この開始姿勢と経路で $SET を書き出す？ [y/N] " ans
if [ "${ans:-N}" != "y" ]; then
  mv "$CSV" "${CSV%.csv}-failed.csv"; mv "$LOG" "${LOG%.teleop.log}-failed.teleop.log"
  echo "[capture] 書き出さなかった。記録は ${CSV%.csv}-failed.csv に残した（撮り直すなら同じコマンドをもう一度）"; exit 0
fi
"$PY" scripts/pose_from_dwell.py "$CSV" --set "$SET" ${SCENE:+--scene "$SCENE"} --write --quiet || exit 1
mkdir -p "config/pose_sets/$SET/evidence"
cp "$CSV" "config/pose_sets/$SET/evidence/02-raw-$SET.csv"
cp "$LOG" "config/pose_sets/$SET/evidence/02-teleop.log"
read -r -p "[capture] CURRENT を $SET にする（次のテレオペ・収集の整列先になる）？ [y/N] " ans
[ "${ans:-N}" = "y" ] && python3 scripts/pose_set.py current "$SET"
python3 scripts/pose_set.py list
echo "[capture] 完了。次の起動で新しい開始姿勢へ整列する: bash scripts/run_teleop_bimanual.sh"
