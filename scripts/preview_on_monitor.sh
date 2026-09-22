#!/usr/bin/env bash
# Thor のモニターに rerun ビューアを常駐させ、3 眼プレビュー（camera_preview.py）を流す/止める。
#
#   bash scripts/preview_on_monitor.sh start          # ビューア(無ければ起動) + プレビュー開始
#   bash scripts/preview_on_monitor.sh start-viewer   # ビューアだけ起動（record が映像を流す先。collect_bimanual.sh が自動で呼ぶ・2026-09-22）
#   bash scripts/preview_on_monitor.sh stop-preview   # プレビューだけ止める（record を起動する前に必ず）
#   bash scripts/preview_on_monitor.sh stop           # 両方止める
#   bash scripts/preview_on_monitor.sh status
#
# ビューアは gRPC :9876 で待受。record は DISPLAY_DATA=true DISPLAY_IP=127.0.0.1 DISPLAY_PORT=9876
# で同じビューアに流す（camera_preview.py と RealSense を同時に開けないので、record 前に stop-preview）。
# DISPLAY/XAUTHORITY はログイン中の gnome-shell の環境から拾う（引数 DISPLAY=… で上書き可）。
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
VENV="$HOME/projects/lerobot/.venv/bin"
RUN="$HOME/.cache/openarm-preview"; mkdir -p "$RUN"
PORT="${PORT:-9876}"
CMD="${1:-status}"
# D405 の露出/ゲイン（自動露出だと暗い・2026-09-21 実測）。config/bimanual-collect.env の D405_GAIN を既定にする
ENV_FILE="${ENV_FILE:-$REPO/config/bimanual-collect.env}"
envdef(){ sed -n "s/^$1=\([0-9-]*\).*/\1/p" "$ENV_FILE" 2>/dev/null | head -1; }
if [ -f "$ENV_FILE" ]; then
  [ -n "${D405_GAIN+x}" ] || D405_GAIN="$(envdef D405_GAIN)"
  [ -n "${D405_EXPOSURE+x}" ] || D405_EXPOSURE="$(envdef D405_EXPOSURE)"
  [ -n "${D435_BRIGHTNESS+x}" ] || D435_BRIGHTNESS="$(envdef D435_BRIGHTNESS)"
fi
cam_setup_args(){ local a=(); if [ -n "${D405_GAIN:-}" ]; then a+=(--d405-gain "$D405_GAIN" --d405-exposure "${D405_EXPOSURE:-28000}"); else a+=(--d405-auto); fi; [ -n "${D435_BRIGHTNESS:-}" ] && a+=(--d435-brightness "$D435_BRIGHTNESS"); printf "%s\n" "${a[@]}"; }

viewer_pid(){ [ -f "$RUN/viewer.pid" ] && kill -0 "$(cat "$RUN/viewer.pid")" 2>/dev/null && cat "$RUN/viewer.pid"; }
preview_pid(){ [ -f "$RUN/preview.pid" ] && kill -0 "$(cat "$RUN/preview.pid")" 2>/dev/null && cat "$RUN/preview.pid"; }
port_open(){ ss -ltn 2>/dev/null | grep -q ":${PORT} "; }

detect_display(){
  # ログイン中のデスクトップ（gnome-shell）の DISPLAY/XAUTHORITY を借りる
  local p
  for p in $(pgrep -u "$USER" -x gnome-shell); do
    DISPLAY_D="$(tr '\0' '\n' < /proc/$p/environ | sed -n 's/^DISPLAY=//p' | head -1)"
    XAUTH_D="$(tr '\0' '\n' < /proc/$p/environ | sed -n 's/^XAUTHORITY=//p' | head -1)"
    [ -n "$DISPLAY_D" ] && break
  done
  DISPLAY_D="${DISPLAY:-${DISPLAY_D:-}}"; XAUTH_D="${XAUTHORITY:-${XAUTH_D:-}}"
}

ensure_viewer(){
  # rerun ビューアが :$PORT で待っていなければデスクトップに起動する（プレビューは起動しない）。
  # record（lerobot-record --display_ip 127.0.0.1 --display_port $PORT）はこのビューアへ 3 眼と関節値を流す。
  if port_open; then
    echo "[monitor] viewer: already listening on :$PORT (pid $(viewer_pid 2>/dev/null || echo '?'))"
  else
    detect_display
    [ -n "$DISPLAY_D" ] || { echo "[monitor] デスクトップにログインしていない（gnome-shell 無し）。モニターでログインしてから。" >&2; return 1; }
    DISPLAY="$DISPLAY_D" XAUTHORITY="$XAUTH_D" setsid nohup "$VENV/rerun" --port "$PORT" --memory-limit 2GB \
      > "$RUN/viewer.log" 2>&1 < /dev/null &
    echo $! > "$RUN/viewer.pid"
    for _ in 1 2 3 4 5 6 7 8 9 10; do sleep 1; port_open && break; done
    port_open && echo "[monitor] viewer: started on DISPLAY=$DISPLAY_D, listening :$PORT (pid $(cat "$RUN/viewer.pid"))" \
              || { echo "[monitor] viewer が :$PORT で待受しない。log:"; tail -5 "$RUN/viewer.log"; return 1; }
  fi
  # モニターの暗転防止（X の DPMS とスクリーンセーバ。X セッション毎にリセットされるので毎回）
  detect_display
  if [ -n "${DISPLAY_D:-}" ]; then DISPLAY="$DISPLAY_D" XAUTHORITY="$XAUTH_D" xset s off -dpms 2>/dev/null && echo "[monitor] dpms/screensaver off on $DISPLAY_D"; fi
}

case "$CMD" in
  start)
    ensure_viewer || exit 1
    if [ -n "$(preview_pid)" ]; then
      echo "[monitor] preview: already running (pid $(preview_pid))"
    else
      # USB 省電力を止める（udev 81-openarm-usb-nosuspend.rules の保険。再列挙で auto に戻ることがある）
      for _d in /sys/bus/usb/devices/*; do
        [ -f "$_d/idVendor" ] || continue
        case "$(cat "$_d/idVendor"):$(cat "$_d/idProduct")" in 05e3:0626|8086:0b3a|8086:0b5b) echo on | sudo -n tee "$_d/power/control" >/dev/null 2>&1 ;; esac
      done
      # カメラを誰も掴んでいない今のうちに D405 の露出/ゲインを焼く（設定はデバイスに残る）
      mapfile -t _cs < <(cam_setup_args)
      if [ ${#_cs[@]} -gt 0 ]; then "$VENV/python" "$REPO/scripts/camera_setup.py" "${_cs[@]}" || echo "[monitor] camera_setup 失敗（続行）"; fi
      # RERUN_FLUSH_TICK_SECS: SDK の送信バッファ（既定 0.2 秒）を短くしてラグを削る
      RERUN_FLUSH_TICK_SECS="${RERUN_FLUSH_TICK_SECS:-0.01}" setsid nohup "$VENV/python" -u "$REPO/scripts/camera_preview.py" --ip 127.0.0.1 --port "$PORT" \
        > "$RUN/preview.log" 2>&1 < /dev/null &
      echo $! > "$RUN/preview.pid"
      sleep 6
      if [ -n "$(preview_pid)" ]; then echo "[monitor] preview: started (pid $(cat "$RUN/preview.pid"))"; else echo "[monitor] preview が落ちた。log:"; tail -8 "$RUN/preview.log"; exit 1; fi
    fi
    tail -3 "$RUN/preview.log"
    ;;
  start-viewer)
    ensure_viewer
    ;;
  stop-preview)
    p="$(preview_pid)"; if [ -n "$p" ]; then kill -INT "$p"; sleep 2; kill -0 "$p" 2>/dev/null && kill "$p"; echo "[monitor] preview: stopped"; else echo "[monitor] preview: not running"; fi
    rm -f "$RUN/preview.pid"
    ;;
  stop)
    "$0" stop-preview
    v="$(viewer_pid)"; if [ -n "$v" ]; then kill "$v"; echo "[monitor] viewer: stopped"; else echo "[monitor] viewer: not running (pidfile)"; fi
    rm -f "$RUN/viewer.pid"
    ;;
  status)
    echo "[monitor] viewer: $( port_open && echo "listening :$PORT" || echo "not listening" ) pid=$(viewer_pid || echo -)"
    echo "[monitor] preview: pid=$(preview_pid || echo -)"
    [ -f "$RUN/preview.log" ] && tail -2 "$RUN/preview.log"
    ;;
  *) echo "usage: $0 start|start-viewer|stop-preview|stop|status" >&2; exit 1 ;;
esac
