# Damiao CAN フィードバックロガー（scripts/damiao_feedback_log.py・受信専用）を裏で自動起動・停止する。bash から source して使う。
# 2026-10-05: 「端末 1 でロガー・端末 2 でテレオペ・1 回ごとに起動し直す」手作業をやめるため
#   （設計ノート itabashi-bimanual-calibration-and-design-current.md §2.1 の未了項目）。
#
#   source "$REPO/scripts/lib/dmlog.sh"
#   dmlog_start <stem> <python> <follow_file|""> <iface...>   # 裏で起動する。常に 0 を返す（失敗は警告だけ・本体は続ける）
#   dmlog_mark "<text>"                                       # MARK を 1 行打つ（<stem>.marks へ追記・ロガーが拾う）
#   dmlog_stop ["rc=<n>"]                                     # 止めて要約を出す。冪等。EXIT trap からも呼ばれる
#
#   dmlog_latest_csv [dir]                                    # いちばん新しい dmlog CSV のパス（dmlog_report.sh / dmlog_live.sh が使う）
#
# 出力（<stem> に拡張子を足す）: .csv 記録 / .log ロガーの画面出力 / .marks シェルからの印 /
#   .summary.txt 停止時の要約（ロガー自身が書く・一瞬・判定材料と自動解析の判断を含む）/
#   .report.txt --report の全文（既定は異常があったときだけ裏で書く・待たない）
#
# 解析の扱い（2026-10-05 作り直し・Kento「既定は記録だけ。必要なときに解析し、あとからも作れるように」）:
#   - 既定では記録（CSV・MARK 行込み）と要約だけを残す。--report が読むのは CSV だけなので、あとで
#     scripts/dmlog_report.sh で同じレポートを作れる（エピソード単位の切り出しは --episode N）。
#   - 停止時、ロガーが記録中に数えた判定材料（要約の「判定材料」行）と終了コードで異常が見えたときだけ、
#     裏で --report を回す（DMLOG_REPORT=anomaly）。異常の条件は _dmlog_anomaly_reasons を参照。
#   - テレオペしながら見るときは別端末で bash scripts/dmlog_live.sh（CSV を読むだけ・CAN には触らない）。
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
#   DMLOG_REPORT=anomaly|on|off  停止時に --report を裏で回すか（既定 anomaly＝異常があったときだけ・2026-10-05）。
#                             on は毎回回す（旧 auto も on と同じ扱い）。off は回さない（手で回すコマンドだけ出す）。
#   DMLOG_ANOMALY_TEMP_C=80   anomaly の温度の閾値（ロータ温度の最大がこれ以上なら回す）
#   DMLOG_REPORT_MAX_MB=3000  CSV がこれより大きければ自動では回さない（on・anomaly 共通）。--report は 2026-10-05 から
#                             1 行ずつ読むので RAM は CSV の大きさに依らない（300MB で約 28MB）。上限は時間の目安で、
#                             Vector で約 35MB/s（Thor は未測定）。旧既定 300 は全行をメモリに載せていた頃の値
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

# DMLOG_REPORT の正規化（anomaly / on / off）。旧 auto は on と同じ（毎回回す・大きさの上限つき）。知らない値は既定の anomaly。
_dmlog_report_mode() {
  local v="${DMLOG_REPORT:-anomaly}"
  case "${v,,}" in
    off|0|false|no) echo off ;;
    on|1|true|yes|always|auto) echo on ;;
    *) echo anomaly ;;
  esac
}

# 要約の「判定材料」行の機械可読部分（[run_lost=.. bad_state=.. max_t_rotor=..]）から 1 つ取り出す。無ければ空。
_dmlog_material() {
  local f="$1" k="$2"
  [ -s "$f" ] || return 0
  grep -m1 '^判定材料' "$f" 2>/dev/null | sed -n "s/.*[[ ]$k=\([0-9][0-9]*\).*/\1/p" || true
}

# dmlog_stop の引数（"rc=<n>" か "rc=aborted trap_status=<n>"）から終了コードを取り出す。分からなければ空。
_dmlog_rc_of() {
  local note="$1" rc
  rc="$(printf '%s' "$note" | sed -n 's/.*rc=\([^ ]*\).*/\1/p')"
  [ "$rc" = aborted ] && rc="$(printf '%s' "$note" | sed -n 's/.*trap_status=\([0-9][0-9]*\).*/\1/p')"
  case "$rc" in ''|*[!0-9-]*) rc="" ;; esac
  printf '%s' "$rc"
}

_dmlog_temp_thr() {
  local thr="${DMLOG_ANOMALY_TEMP_C:-80}"
  case "$thr" in ''|*[!0-9]*) thr=80 ;; esac
  printf '%s' "$thr"
}

# 異常の理由を「・」区切りで出す（無ければ空）。2026-10-05・DMLOG_REPORT=anomaly の判定。
#   (a) 動作中の LOST が 1 回以上: ロガーが数えた run_lost（同じバスの他モータが流れ続けた間の無応答。起動の整列・
#       保存・停止でバス全体が黙る間の束は数えない。基準は damiao_feedback_log.py の LostTracker）
#   (b) 状態コードが disabled(0)/enabled(1) 以外のフレームが 1 つ以上（過温 0xC・0xB、過電流 0xA 等。記録開始時点の
#       ラッチも含む。--report の state分布・transition・「記録開始時点で既にラッチ」と同じ 0/1 以外の扱い）
#   (c) ロータ温度の最大が DMLOG_ANOMALY_TEMP_C（既定 80）以上
#   (d) 終了コードが 0 と 130（Ctrl-C は通常の止め方）以外。rc=aborted のときは trap_status を見る
#   要約（判定材料）が無いときは (a)〜(c) は判定しない（回さない）。(d) だけで決める。
_dmlog_anomaly_reasons() {
  local f="$1" note="$2" out="" v thr rc
  v="$(_dmlog_material "$f" run_lost)"
  [ -n "$v" ] && [ "$v" -gt 0 ] && out="${out:+$out・}動作中の LOST ${v} 回"
  v="$(_dmlog_material "$f" bad_state)"
  [ -n "$v" ] && [ "$v" -gt 0 ] && out="${out:+$out・}エラー状態のフレーム ${v}"
  thr="$(_dmlog_temp_thr)"
  v="$(_dmlog_material "$f" max_t_rotor)"
  [ -n "$v" ] && [ "$v" -ge "$thr" ] && out="${out:+$out・}ロータ温度 ${v}C≥${thr}C"
  rc="$(_dmlog_rc_of "$note")"
  if [ -n "$rc" ] && [ "$rc" != 0 ] && [ "$rc" != 130 ]; then out="${out:+$out・}終了コード $rc"; fi
  printf '%s' "$out"
  return 0
}

# 「異常なし」のときに添える判定材料の値（何を見て回さなかったかを残す）
_dmlog_material_text() {
  local f="$1" note="$2" a b c rc
  a="$(_dmlog_material "$f" run_lost)"; b="$(_dmlog_material "$f" bad_state)"; c="$(_dmlog_material "$f" max_t_rotor)"
  rc="$(_dmlog_rc_of "$note")"
  if [ -z "$a$b$c" ]; then
    if [ -s "$f" ]; then
      printf '要約に判定材料の行が無い・終了コード %s' "${rc:-不明}"
    else
      printf '要約が無い・終了コード %s' "${rc:-不明}"
    fi
  else
    printf '動作中の LOST %s・エラー状態 %s・最大 T_rotor %sC<%sC・終了コード %s' "${a:-?}" "${b:-?}" "${c:-?}" "$(_dmlog_temp_thr)" "${rc:-不明}"
  fi
}

# 要約に 1 行足す（端末に出す部分＝'== MARK (全件)' の前に入れる）。失敗しても何もしない。
_dmlog_summary_add() {
  local f="$1"
  DMLOG_ADD_LINE="$2" awk 'BEGIN { l = ENVIRON["DMLOG_ADD_LINE"]; d = 0 }
    !d && index($0, "== MARK (全件)") == 1 { print l; d = 1 }
    { print }
    END { if (!d) print l }' "$f" > "$f.tmp" 2>/dev/null && mv -f "$f.tmp" "$f" 2>/dev/null || rm -f "$f.tmp" 2>/dev/null
  return 0
}

# いちばん新しい dmlog CSV（テレオペの ~/collections/dmlog-teleop-*.csv と収集の run_dir/dmlog-*.csv）。
# 引数に run_dir を渡すとその中だけを探す。無ければ空を返す（終了コード 1）。
dmlog_latest_csv() {
  local base="${1:-}" f
  if [ -n "$base" ]; then
    f="$(ls -1t "$base"/dmlog-*.csv 2>/dev/null | head -1 || true)"
  else
    local top="${DMLOG_DIR:-$HOME/collections}"
    f="$(ls -1t "$top"/dmlog-teleop-*.csv "$top"/*/dmlog-*.csv "$HOME/collections"/*/dmlog-*.csv 2>/dev/null | head -1 || true)"
  fi
  [ -n "$f" ] || return 1
  printf '%s\n' "$f"
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
  # 手で起こしたロガーが居ても殺さない（受信専用どうしは干渉しない）。知らせるだけ。--report（解析のみ）と --live（読むだけ）は数えない。
  local others
  others="$(pgrep -af '^[^ ]*python[^ ]* .*damiao_feedback_log\.py' 2>/dev/null | grep -v -e '--report' -e '--live' | awk '{printf "%s ", $1}' || true)"
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
  # 解析を回すかを決めてから要約を出す（判断の 1 行を要約にも残す・2026-10-05）
  local manual="bash $DMLOG_REPO/scripts/dmlog_report.sh $stem.csv"
  local mode reasons="" mb=0 run=0 why
  mode="$(_dmlog_report_mode)"
  mb=$(( $(stat -c %s "$stem.csv" 2>/dev/null || echo 0) / 1048576 )) || true
  if [ "$mode" = anomaly ]; then
    reasons="$(_dmlog_anomaly_reasons "$stem.summary.txt" "$note" 2>/dev/null || true)"
  fi
  if [ ! -s "$stem.csv" ]; then
    why="回していない（CSV が空）"
  elif [ "$mode" = off ]; then
    why="回していない（DMLOG_REPORT=off）"
  elif [ "$mode" = anomaly ] && [ -z "$reasons" ]; then
    if [ -n "$(_dmlog_material "$stem.summary.txt" run_lost)" ]; then
      why="回していない（異常なし: $(_dmlog_material_text "$stem.summary.txt" "$note")）"
    else  # 判定材料が無いときは終了コードだけで決めた（(a)〜(c) は判定していない）
      why="回していない（判定材料が無い: $(_dmlog_material_text "$stem.summary.txt" "$note")）"
    fi
  elif [ "$mb" -gt "${DMLOG_REPORT_MAX_MB:-3000}" ]; then
    why="回していない（${reasons:+理由: $reasons だが }CSV が ${mb}MB > DMLOG_REPORT_MAX_MB=${DMLOG_REPORT_MAX_MB:-3000}）"
  else
    run=1
    if [ "$mode" = on ]; then why="回した（DMLOG_REPORT=on）"; else why="回した（理由: $reasons）"; fi
  fi
  [ -s "$stem.summary.txt" ] && _dmlog_summary_add "$stem.summary.txt" "自動解析（--report）: $why"
  if [ -s "$stem.summary.txt" ]; then
    sed '/^== MARK (全件)/,$d' "$stem.summary.txt" 2>/dev/null | sed 's/^/[dmlog] /' | _dmlog_say || true
    _dmlog_say "[dmlog] 要約の全文（MARK 全件）: $stem.summary.txt"
  else
    _dmlog_say "[dmlog] 警告: 要約が書かれていない（$stem.log を参照）。CSV: $stem.csv"
    _dmlog_say "[dmlog] 自動解析（--report）: $why"
  fi
  if [ "$run" = 1 ]; then
    local launch=()
    command -v setsid >/dev/null 2>&1 && launch+=(setsid)
    command -v nice >/dev/null 2>&1 && launch+=(nice -n 15)
    command -v timeout >/dev/null 2>&1 && launch+=(timeout "${DMLOG_REPORT_TIMEOUT:-900}")
    ${launch[@]+"${launch[@]}"} "$py" "$DMLOG_TOOL" --report "$stem.csv" </dev/null >"$stem.report.txt" 2>&1 &
    _dmlog_say "[dmlog] --report を裏で書いている（待たない・最大 ${DMLOG_REPORT_TIMEOUT:-900}s）: $stem.report.txt"
  else
    _dmlog_say "[dmlog] あとでレポートを作るとき: $manual（エピソード単位は --episode N を足す）"
  fi
  return 0
}
