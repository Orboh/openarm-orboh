# Damiao CAN フィードバックロガー（scripts/damiao_feedback_log.py・受信専用）を裏で自動起動・停止する。bash から source して使う。
# 2026-10-05: 「端末 1 でロガー・端末 2 でテレオペ・1 回ごとに起動し直す」手作業をやめるため
#   （設計ノート itabashi-bimanual-calibration-and-design-current.md §2.1 の未了項目）。
#
#   source "$REPO/scripts/lib/dmlog.sh"
#   dmlog_start <stem> <python> <follow_file|""> <iface...>   # 裏で起動する。常に 0 を返す（失敗は警告だけ・本体は続ける）
#   dmlog_mark "<text>"                                       # MARK を 1 行打つ（<stem>.marks へ追記・ロガーが拾う）
#   dmlog_stop ["rc=<n>"]                                     # 止めて要約を出す。冪等。EXIT trap からも呼ばれる
#
# 出力（<stem> に拡張子を足す）: .csv 記録 / .log ロガーの画面出力 / .marks シェルからの印 /
#   .summary.txt 停止時の要約（ロガー自身が書く・一瞬）/ .report.txt --report の全文（裏で書く・待たない）
#
# 守っていること（10/7 の収集を壊さない）:
#   - 端末の stdin を読まない: </dev/null に加えて setsid で制御端末から外す（lerobot は fd 0 を直読みでキーを取る）。
#   - Ctrl-C で道連れにしない: 端末の Ctrl-C は前景のプロセスグループにしか届かない。止めるのは親が
#     lerobot の終了を見届け、脱力の瞬間を拾う DMLOG_TAIL_S 秒を待ってから送る SIGTERM だけ。
#   - 本体の終了コードを変えない: trap の中で exit を呼ばない。全部 || true。
#   - 親が trap を走らせずに死んでも（SIGKILL 等）、ロガーは --watch-pid で親の消滅を見て自分で止まる。
#
# ノブ:
#   DMLOG=off                 何もしない（起動する子プロセスも trap も従来どおり）
#   DMLOG_IFACES="a b"        記録する CAN を差し替える（重いときは "can_fol_l can_fol_r" に絞る）
#   DMLOG_REPORT=auto|on|off  --report を裏で回すか。auto は CSV が DMLOG_REPORT_MAX_MB（既定 300）を超えたら回さない
#   DMLOG_REPORT_TIMEOUT=900  --report の時間制限（秒）
#   DMLOG_TAIL_S=2            本体の終了からロガーを止めるまでの待ち（秒・脱力の瞬間を残す）
#   DMLOG_NICE=10             ロガーの nice 値（record の 30fps より下げる）
#   DMLOG_CONSOLE=<file>      [dmlog] の行をこのファイルにも追記する（record は console.log を渡す）

DMLOG_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DMLOG_TOOL="$DMLOG_REPO/scripts/damiao_feedback_log.py"
DMLOG_PID=""
DMLOG_STEM=""
DMLOG_PY=""

_dmlog_enabled() {
  local v="${DMLOG:-on}"
  case "${v,,}" in off|0|false|no) return 1 ;; esac
  return 0
}

# 端末と DMLOG_CONSOLE の両方に出す（引数があればその 1 行・無ければ stdin の全行）
_dmlog_say() {
  # 引数なしで stdin が端末なら読まない（lerobot のキー入力と取り合わないための保険・2026-10-05 監査）
  [ $# -eq 0 ] && [ -t 0 ] && return 0
  if [ $# -gt 0 ]; then printf '%s\n' "$*"; else cat; fi | {
    if [ -n "${DMLOG_CONSOLE:-}" ]; then tee -a "$DMLOG_CONSOLE" 2>/dev/null || cat; else cat; fi
  } || true
}

# 生きているか（ゾンビは死んだ扱い。自分の子は wait で回収するまでゾンビで残り kill -0 が通るため）
_dmlog_alive() {
  local pid="$1" state
  kill -0 "$pid" 2>/dev/null || return 1
  state="$(sed 's/^.*) //' "/proc/$pid/stat" 2>/dev/null | cut -d' ' -f1)"
  [ "$state" != "Z" ]
}

dmlog_start() {
  _dmlog_enabled || return 0
  [ -z "${DMLOG_PID:-}" ] || return 0
  local stem="${1:-}" py="${2:-}" follow="${3:-}"
  [ $# -ge 3 ] && shift 3 || { _dmlog_say "[dmlog] 警告: dmlog_start の引数が足りない。ロガーなしで続ける"; return 0; }
  local ifaces=("$@")
  if [ -n "${DMLOG_IFACES:-}" ]; then read -r -a ifaces <<< "$DMLOG_IFACES"; fi
  if [ "${#ifaces[@]}" -eq 0 ] || [ -z "$stem" ]; then
    _dmlog_say "[dmlog] 警告: 出力先か iface が空。ロガーなしで続ける"; return 0
  fi
  if [ ! -f "$DMLOG_TOOL" ] || { [ ! -x "$py" ] && ! command -v "$py" >/dev/null 2>&1; }; then
    _dmlog_say "[dmlog] 警告: $DMLOG_TOOL か python（$py）が無い。ロガーなしで続ける"; return 0
  fi
  mkdir -p "$(dirname "$stem")" 2>/dev/null || true
  if [ -e "$stem.csv" ]; then  # 同名があれば -N を付けて逃がす（上書きしない）
    local n=2
    while [ -e "$stem-$n.csv" ]; do n=$((n + 1)); done
    stem="$stem-$n"
  fi
  # 手で起こしたロガーが居ても殺さない（受信専用どうしは干渉しない）。知らせるだけ。前回の --report（解析のみ）は数えない。
  local others
  others="$(pgrep -af '^[^ ]*python[^ ]* .*damiao_feedback_log\.py' 2>/dev/null | grep -v -- '--report' | awk '{printf "%s ", $1}' || true)"
  [ -z "$others" ] || _dmlog_say "[dmlog] 注意: 別の damiao_feedback_log.py が動いている（pid ${others% }）。止めずに並走する"
  : > "$stem.marks" 2>/dev/null || true
  local args=("${ifaces[@]}" --out "$stem.csv" --summary-out "$stem.summary.txt" --pid-file "$stem.pid"
              --watch-pid "$$" --marks "$stem.marks")
  [ -z "$follow" ] || args+=(--follow "$follow")
  local launch=()
  command -v setsid >/dev/null 2>&1 && launch+=(setsid)
  command -v nice >/dev/null 2>&1 && launch+=(nice -n "${DMLOG_NICE:-10}")
  ${launch[@]+"${launch[@]}"} "$py" -u "$DMLOG_TOOL" "${args[@]}" </dev/null >"$stem.log" 2>&1 &
  DMLOG_PID=$!
  DMLOG_STEM="$stem"
  DMLOG_PY="$py"
  sleep 0.5
  if ! _dmlog_alive "$DMLOG_PID"; then
    wait "$DMLOG_PID" 2>/dev/null || true
    _dmlog_say "[dmlog] 警告: ロガーが起動直後に終わった（$stem.log）。ロガーなしで続ける"
    head -5 "$stem.log" 2>/dev/null | sed 's/^/[dmlog]   /' | _dmlog_say || true
    DMLOG_PID=""
    return 0
  fi
  # 通常の終わり方では呼び手が本体の RC を付けて dmlog_stop を先に呼ぶので、trap が働くのは Ctrl-C で
  # bash ごと落ちたときと set -e で抜けたときだけ。そのときの $? は本体の RC と限らないので aborted と書く。
  trap 'dmlog_stop "rc=aborted trap_status=$?"' EXIT
  _dmlog_say "[dmlog] Damiao フィードバックを裏で記録中 pid=$DMLOG_PID ifaces=$(IFS=,; echo "${ifaces[*]}") csv=$stem.csv（止めるのは自動・DMLOG=off で無効）"
  # pipefail + set -e の下でも落ちないよう、パイプは全部 || true（grep の不一致は 1 を返す）
  grep 'open failed' "$stem.log" 2>/dev/null | sed 's/^/[dmlog] 警告: /' | _dmlog_say || true
  return 0
}

dmlog_mark() {
  [ -n "${DMLOG_PID:-}" ] && [ -n "${DMLOG_STEM:-}" ] || return 0
  printf 'MARK %s\n' "$*" >> "$DMLOG_STEM.marks" 2>/dev/null || true
  return 0
}

dmlog_stop() {
  local pid="${DMLOG_PID:-}" stem="${DMLOG_STEM:-}" py="${DMLOG_PY:-python3}" note="${1:-}"
  [ -n "$pid" ] || return 0
  DMLOG_PID=""   # 冪等（EXIT trap からの 2 回目は何もしない）
  trap '' INT    # 後始末の数秒に Ctrl-C を重ねられてもロガーを取りこぼさない
  [ -z "$note" ] || printf 'MARK EXIT %s\n' "$note" >> "$stem.marks" 2>/dev/null || true
  sleep "${DMLOG_TAIL_S:-2}" 2>/dev/null || true
  local real="$pid"
  [ -s "$stem.pid" ] && real="$(tr -dc '0-9' < "$stem.pid" 2>/dev/null || echo "$pid")"
  [ -n "$real" ] || real="$pid"
  kill -TERM "$real" 2>/dev/null || true
  local i
  for i in $(seq 1 50); do _dmlog_alive "$real" || break; sleep 0.1; done
  if _dmlog_alive "$real"; then
    _dmlog_say "[dmlog] 警告: SIGTERM で 5 秒止まらないので SIGKILL（CSV の末尾 1 秒ぶんが欠けうる）"
    kill -KILL "$real" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
  trap - INT
  if [ -s "$stem.summary.txt" ]; then
    sed '/^== MARK (全件)/,$d' "$stem.summary.txt" 2>/dev/null | sed 's/^/[dmlog] /' | _dmlog_say || true
    _dmlog_say "[dmlog] 要約の全文（MARK 全件）: $stem.summary.txt"
  else
    _dmlog_say "[dmlog] 警告: 要約が書かれていない（$stem.log を参照）。CSV: $stem.csv"
  fi
  local manual="$py $DMLOG_TOOL --report $stem.csv > $stem.report.txt"
  local mode="${DMLOG_REPORT:-auto}" mb=0
  mb=$(( $(stat -c %s "$stem.csv" 2>/dev/null || echo 0) / 1048576 )) || true
  if [ "$mode" = off ] || [ ! -s "$stem.csv" ]; then
    _dmlog_say "[dmlog] --report は回していない。手で回すとき: $manual"
  elif [ "$mode" = auto ] && [ "$mb" -gt "${DMLOG_REPORT_MAX_MB:-300}" ]; then
    _dmlog_say "[dmlog] CSV が ${mb}MB と大きいので --report は回していない（DMLOG_REPORT=on で回す）。手で回すとき: $manual"
  else
    local launch=()
    command -v setsid >/dev/null 2>&1 && launch+=(setsid)
    command -v nice >/dev/null 2>&1 && launch+=(nice -n 15)
    command -v timeout >/dev/null 2>&1 && launch+=(timeout "${DMLOG_REPORT_TIMEOUT:-900}")
    ${launch[@]+"${launch[@]}"} "$py" "$DMLOG_TOOL" --report "$stem.csv" </dev/null >"$stem.report.txt" 2>&1 &
    _dmlog_say "[dmlog] --report を裏で書いている（待たない・最大 ${DMLOG_REPORT_TIMEOUT:-900}s）: $stem.report.txt"
  fi
  return 0
}
