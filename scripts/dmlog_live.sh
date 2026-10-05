#!/usr/bin/env bash
# 記録中の Damiao CAN フィードバック（dmlog-*.csv）を、テレオペ・収集をしながら別の端末で見る（2026-10-05）。
# damiao_feedback_log.py --live が CSV を末尾から tail で追い、5 秒ごとに直近 30 秒の要約を出す:
#   モータごとの T_rotor の最大と傾き（C/min）、LOST/BACK（うち動作中のもの）、エラー状態、追従誤差 |e|max
#   （--report と同じ定義: lead-fol の最近傍・最初の 5 秒の中央値を引く）、直近の MARK。
# CAN には一切触らず CSV を読むだけなので、記録中のロガーとも lerobot とも干渉しない。Ctrl-C で終わる。
#
# 使い方:
#   bash scripts/dmlog_live.sh                        # いちばん新しい CSV（テレオペ・収集を起動して [dmlog] の行が出てから流す）
#   bash scripts/dmlog_live.sh ~/collections/<run_dir> # その run_dir の CSV
#   bash scripts/dmlog_live.sh <csv> --every 2 --window 60   # 間隔と窓を変える
#   bash scripts/dmlog_live.sh latest --from-start    # 末尾でなく先頭から読む（終わった記録を流し見るとき）
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
TOOL="$REPO/scripts/damiao_feedback_log.py"
source "$REPO/scripts/lib/dmlog.sh"
PY="$HOME/projects/lerobot/.venv/bin/python"
[ -x "$PY" ] || PY=python3

target="latest"
if [ $# -gt 0 ] && [ "${1#--}" = "$1" ]; then target="$1"; shift; fi
if [ "$target" = latest ]; then
  csv="$(dmlog_latest_csv)" || { echo "[dmlog-live] dmlog の CSV が見つからない（${DMLOG_DIR:-$HOME/collections}）" >&2; exit 1; }
elif [ -d "$target" ]; then
  csv="$(dmlog_latest_csv "$target")" || { echo "[dmlog-live] $target に dmlog-*.csv が無い" >&2; exit 1; }
elif [ -f "$target" ]; then
  csv="$target"
else
  echo "[dmlog-live] $target が無い（latest・CSV・run_dir のどれか）" >&2; exit 1
fi
age=$(( $(date +%s) - $(stat -c %Y "$csv" 2>/dev/null || date +%s) ))
if [ "$age" -gt 10 ]; then
  echo "[dmlog-live] 注意: $csv は ${age} 秒更新されていない（終わった記録かもしれない。テレオペを起動してから流し直す）" >&2
fi
launch=()
command -v nice >/dev/null 2>&1 && launch+=(nice -n 10)
exec ${launch[@]+"${launch[@]}"} "$PY" -u "$TOOL" --live "$csv" "$@"
