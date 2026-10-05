#!/usr/bin/env bash
# Damiao CAN フィードバックの記録（dmlog-*.csv）から、あとでレポート（damiao_feedback_log.py --report）を作る（2026-10-05）。
# 停止時の自動解析は既定で「異常があったときだけ」になった（scripts/lib/dmlog.sh の DMLOG_REPORT=anomaly）。
# --report が読むのは CSV だけなので、「あのときの report がほしい」となったら、いつでもここから同じものが作れる。
# --report は 1 行ずつ読む（RAM は CSV の大きさに依らない）ので、テレオペや収集と並べて回してよい（nice 10 で回す）。
#
# 使い方:
#   bash scripts/dmlog_report.sh                                  # いちばん新しい CSV（テレオペ ~/collections/dmlog-teleop-*.csv と
#                                                                 #   収集の ~/collections/<run_dir>/dmlog-*.csv のうち最新）
#   bash scripts/dmlog_report.sh latest --episode 22              # エピソード 22 の区間だけ（EP_START ep=22 〜 EP_END/EP_DISCARD/EXIT）。
#                                                                 #   録り直しで同じ番号が複数あるときは最後のテイク（保存されたもの）
#   bash scripts/dmlog_report.sh ~/collections/<run_dir> --episode 3 --take 5   # run_dir の中の最新 CSV・テイク番号まで指定
#   bash scripts/dmlog_report.sh <csv> --since 120 --until 180    # t_rel（ロガー起動からの秒）で絞る
#   その他の引数（--pre 5 --window 30）は --report へそのまま渡す。
#
# 出力: CSV の横に .report.txt（切り出しは .report-ep<N>.txt・.report-ep<N>-take<K>.txt・.report-t<since>-<until>.txt）と端末。
#   python は ~/projects/lerobot/.venv/bin/python（無ければ python3）。CAN には触らない（読むだけ）。
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
TOOL="$REPO/scripts/damiao_feedback_log.py"
source "$REPO/scripts/lib/dmlog.sh"
PY="$HOME/projects/lerobot/.venv/bin/python"
[ -x "$PY" ] || PY=python3

target="latest"
if [ $# -gt 0 ] && [ "${1#--}" = "$1" ]; then target="$1"; shift; fi
if [ "$target" = latest ]; then
  csv="$(dmlog_latest_csv)" || { echo "[dmlog-report] dmlog の CSV が見つからない（${DMLOG_DIR:-$HOME/collections}）" >&2; exit 1; }
elif [ -d "$target" ]; then
  csv="$(dmlog_latest_csv "$target")" || { echo "[dmlog-report] $target に dmlog-*.csv が無い" >&2; exit 1; }
elif [ -f "$target" ]; then
  csv="$target"
else
  echo "[dmlog-report] $target が無い（latest・CSV・run_dir のどれか）" >&2; exit 1
fi

# 出力名の接尾辞（切り出しの引数から）
suffix="" ep="" take="" since="" until=""
args=("$@")
for ((i = 0; i < ${#args[@]}; i++)); do
  a="${args[$i]}" v="${args[$((i + 1))]:-}"
  case "$a" in
    --episode) ep="$v" ;;  --episode=*) ep="${a#*=}" ;;
    --take) take="$v" ;;   --take=*) take="${a#*=}" ;;
    --since) since="$v" ;; --since=*) since="${a#*=}" ;;
    --until) until="$v" ;; --until=*) until="${a#*=}" ;;
  esac
done
[ -n "$ep" ] && suffix+="-ep$ep"
[ -n "$take" ] && suffix+="-take$take"
{ [ -n "$since" ] || [ -n "$until" ]; } && suffix+="-t${since:-0}-${until:-end}"
out="${csv%.csv}.report${suffix}.txt"

echo "[dmlog-report] $csv を解析する（$(du -h "$csv" 2>/dev/null | cut -f1)・読むだけ）→ $out" >&2
launch=()
command -v nice >/dev/null 2>&1 && launch+=(nice -n 10)
${launch[@]+"${launch[@]}"} "$PY" "$TOOL" --report "$csv" ${args[@]+"${args[@]}"} </dev/null | tee "$out.tmp"
rc=${PIPESTATUS[0]}
if [ "$rc" = 0 ]; then
  mv -f "$out.tmp" "$out"
  echo "[dmlog-report] 書いた: $out" >&2
else
  rm -f "$out.tmp"
  echo "[dmlog-report] --report が失敗した（exit=$rc）。ファイルは書いていない" >&2
fi
exit "$rc"
