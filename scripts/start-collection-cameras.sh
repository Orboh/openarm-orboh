#!/usr/bin/env bash
# 収集カメラ一式 (Thor): 実カメラ2台を仮想カメラ3本へ分配して
#   - PICO には chest 全画面 + wrist 小窓の PiP 合成 (/dev/video20)
#   - record には素の chest (/dev/video21) と素の wrist (/dev/video22)
# を同時供給する。実カメラは本スクリプトが V4L2 で占有するので、record 側は
# 必ず CHEST_V4L2=/dev/video21 WRIST_V4L2=/dev/video22 (OpenCV 経由) を使うこと。
#
# 仮想カメラ番号が 20 番台なのは、実カメラのノード (D435i=video0-5,
# D405=video6-11) と衝突させないため。ノード解決は sysfs の USB product 文字列
# + YUYV 対応で行う (video 番号は接続順で変わる。by-id は D435i の RGB が別 USB
# インターフェースにあり同一シリアルで命名衝突してリンクが生えない=使えない)。
#
# 使い方:
#   bash scripts/start-collection-cameras.sh          # 起動 (Ctrl-C で停止)
# 次に別端末で:
#   sender: ~/projects/XRoboToolkit-Orin-Video-Sender/OrinVideoSender --listen 0.0.0.0:13579 --device /dev/video20
#   record: REMOTE=true WRIST=true CHEST_V4L2=/dev/video21 WRIST_V4L2=/dev/video22 ... bash scripts/record_pick_place_openarm.sh <tag>
set -euo pipefail

W=640; H=480; FPS=30
# PiP 小窓: 幅1/3 (偶数必須=I420)。右下に配置。
PIP_W=214; PIP_H=160
PIP_X=$((W - PIP_W)); PIP_Y=$((H - PIP_H))

# product 文字列でカメラ個体を特定し、その個体の YUYV(=RGB) ノードを返す。
# 仮想カメラ (v4l2loopback) は product が無いので自然に除外される。
find_rgb_node() {
  local sys dev prod
  for sys in /sys/class/video4linux/video*; do
    dev="/dev/$(basename "$sys")"
    prod="$(cat "$sys/device/../product" 2>/dev/null || true)"
    case "$prod" in *"$1"*) ;; *) continue ;; esac
    if v4l2-ctl -d "$dev" --list-formats 2>/dev/null | grep -q "'YUYV'"; then
      echo "$dev"
      return 0
    fi
  done
  return 1
}
CHEST_DEV="$(find_rgb_node '435i' || true)"
WRIST_DEV="$(find_rgb_node ' 405' || true)"
if [ -z "$CHEST_DEV" ] || [ -z "$WRIST_DEV" ]; then
  echo "[cams] 実カメラを解決できない (chest='$CHEST_DEV' wrist='$WRIST_DEV')。USB 接続を確認。" >&2
  exit 1
fi
echo "[cams] chest(D435i RGB)=$CHEST_DEV  wrist(D405 RGB)=$WRIST_DEV"

# 残骸掃除 (実カメラの busy チェックより先に行う): 前回の分配プロセス (ffmpeg/gst)
# は仮想カメラを掴んでおり、旧 feeder は実カメラも掴んでいる。仮想カメラの保持
# プロセスを fuser で始末すれば旧リグは丸ごと片付き、再実行がシームレスになる。
# 仮想カメラに触らないプロセス (lerobot の SDK 読みや record のエンコード用
# ffmpeg) は巻き添えにならず、下の busy チェックで正しく検知される。
# ※ v4l2 書き込みで固まった ffmpeg は TERM を受け取れないため -9 (2026-08-27 実測)。
sudo fuser -k -9 /dev/video20 /dev/video21 /dev/video22 2>/dev/null || true
sleep 1

if fuser "$CHEST_DEV" "$WRIST_DEV" >/dev/null 2>&1; then
  echo "[cams] 実カメラを別プロセスが使用中 (sender 直読や SDK 読みを先に止める):" >&2
  fuser -v "$CHEST_DEV" "$WRIST_DEV" >&2 || true
  exit 1
fi

# 仮想カメラ 20/21/22 は毎回作り直す。一度でも書き手が付いて離れたデバイスは
# exclusive_caps の Output ビットが落ちたままになり (ラッチ)、次の gst 書き手が
# "not a output device" で開けなくなる (2026-08-27 実測) ため、既存流用はしない。
# 判定は /proc/modules 直読: `lsmod | grep -q` は pipefail 下で lsmod が SIGPIPE
# を食らいマッチしても偽になることがある (再ロードが時々スキップされた真因)。
sudo modprobe -r v4l2loopback 2>/dev/null || true
if grep -q "^v4l2loopback " /proc/modules; then
  echo "[cams] NG: v4l2loopback を外せない。仮想カメラを使うプロセスが残っている。" >&2
  exit 1
fi
sudo modprobe v4l2loopback video_nr=20,21,22 \
     card_label=PicoPiP,RecChest,RecWrist exclusive_caps=1,1,1
sleep 1

# video20 の初期化 (リトライ付き): ffmpeg は書き込み前に VIDIOC_G_FMT を叩くため、
# フォーマット未設定の loopback には書けない (Invalid argument)。gst で 1 フレーム
# 書いて形式を焼く。ロード直後は udev/PipeWire が新デバイスを一瞬開くらしく
# "not a output device" で失敗することがある (2026-08-27 実測) のでリトライする。
seed_ok=false
SEED_ERR=/tmp/cams-seed-err.log
for _ in 1 2 3 4 5; do
  : > "$SEED_ERR"
  if gst-launch-1.0 -q videotestsrc num-buffers=1 \
       ! video/x-raw,format=I420,width=$W,height=$H,framerate=$FPS/1 \
       ! identity drop-allocation=true ! v4l2sink device=/dev/video20 sync=false >> "$SEED_ERR" 2>&1; then
    seed_ok=true; break
  fi
  sleep 1
done
if [ "$seed_ok" != true ]; then
  echo "[cams] NG: /dev/video20 の初期化に失敗 (5回リトライ)。最終エラー:" >&2
  cat "$SEED_ERR" >&2
  v4l2-ctl -d /dev/video20 --info 2>&1 | grep -A5 -i caps >&2
  exit 1
fi
echo "[cams] 仮想カメラ OK: /dev/video20(PICO PiP) /dev/video21(record chest) /dev/video22(record wrist)"

PIDS=()
cleanup() { echo; echo "[cams] 停止"; kill "${PIDS[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# gst は 3 プロセスに分割する。全部を 1 パイプラインに載せると Thor の
# GStreamer 1.24 が起動時に SIGSEGV する (2026-08-27 切り分け: 各要素は単体で
# 全部通る。複数ライブソース+tee+compositor の同居だけが落ちる)。
# v4l2loopback は 1 writer / 多 reader なので、素流しの 21/22 を record (OpenCV)
# と合成プロセスの両方が読む。
# identity drop-allocation=true は v4l2sink への DMA バッファ問題の回避 (G1 実績)。

# P1/P2: 実カメラ → 素の仮想カメラ (YUY2 のまま・無変換)
gst-launch-1.0 -q v4l2src device="$CHEST_DEV" \
  ! video/x-raw,format=YUY2,width=$W,height=$H,framerate=$FPS/1 \
  ! identity drop-allocation=true ! v4l2sink device=/dev/video21 sync=false &
PIDS+=($!)
gst-launch-1.0 -q v4l2src device="$WRIST_DEV" \
  ! video/x-raw,format=YUY2,width=$W,height=$H,framerate=$FPS/1 \
  ! identity drop-allocation=true ! v4l2sink device=/dev/video22 sync=false &
PIDS+=($!)
sleep 2

# P3: 仮想 21/22 を読み PiP 合成 → 仮想 20。合成だけは ffmpeg で行う:
# Thor の gst 1.24 は「1 プロセス内の複数 v4l2src + compositor」で SIGSEGV する
# (2026-08-27 切り分け: 素流し・変換・合成の各 gst は単体では全部通る)。
# 出力は yuv420p (=I420)。sender は入力を内部で I420 化するのでそのまま渡せる。
ffmpeg -hide_banner -loglevel error \
  -f v4l2 -input_format yuyv422 -video_size ${W}x${H} -framerate $FPS -i /dev/video21 \
  -f v4l2 -input_format yuyv422 -video_size ${W}x${H} -framerate $FPS -i /dev/video22 \
  -filter_complex "[1:v]scale=${PIP_W}:${PIP_H}[pip];[0:v][pip]overlay=${PIP_X}:${PIP_Y},format=yuv420p" \
  -f v4l2 /dev/video20 &
PIDS+=($!)

sleep 3
for pid in "${PIDS[@]}"; do
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "[cams] NG: 分配プロセス (pid $pid) が起動しない" >&2
    exit 1
  fi
done
echo "[cams] 分配稼働中 (pids ${PIDS[*]})。停止は Ctrl-C"
echo "[cams] 次: sender は --device /dev/video20 / record は CHEST_V4L2=/dev/video21 WRIST_V4L2=/dev/video22"
wait
