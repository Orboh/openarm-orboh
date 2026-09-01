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
# PICO 向け合成キャンバスは 16:9 (1280x720)。4:3 のままだと PICO 側キャンバスに
# 余白が出て、その領域が市松柄として符号化されてしまう (2026-08-28 実測・sender
# 側 compositor の既定背景)。レイアウト: chest を左に 4:3 のまま (960x720)、
# 右の余白カラム (320px) を wrist 専用パネル (320x240・上下中央) に転用する。
# wrist が chest に重ならず、PICO への解像度も 720p ネイティブになる。
OUT_W=1280; OUT_H=720
# PICO のステレオ表示モードは 1280x720 のうち中央の約 4:3 (左右 ±約160px を除く)
# しか表示しない (2026-08-31 スクショ実測: 平面=二画面モードでは全幅見えるが、
# ステレオ化すると左右対称に刈られる)。よって全コンテンツを「中央の安全域」に
# 収める: chest=中央 960x720 (4:3・ステレオ表示域と一致=欠けゼロ)、wrist=安全域の
# 右下に小窓 (外側の帯はステレオで見えないため使わない)。左右の黒帯は平面モード
# でのみ見える余白。刈り込み幅の個体差は MARGIN_LR で調整:
#   MARGIN_LR=120 bash start-collection-cameras.sh   (狭める=コンテンツ拡大)
MARGIN_LR="${MARGIN_LR:-160}"
CHEST_W=$(( OUT_W - 2 * MARGIN_LR ))
CHEST_H=$(( CHEST_W * 3 / 4 )); [ "$CHEST_H" -gt "$OUT_H" ] && CHEST_H=$OUT_H
CHEST_H=$(( CHEST_H / 2 * 2 )); CHEST_W=$(( CHEST_H * 4 / 3 / 2 * 2 ))
CHEST_X=$(( (OUT_W - CHEST_W) / 2 ))
CHEST_Y=$(( (OUT_H - CHEST_H) / 2 ))
# wrist 小窓 (4:3・chest 右下に内側 16px で重ねる)。chest への重なりを減らすため
# 既定は 216x162 (chest の約5%。288x216 だと約9%)。見づらければ現場で上書き:
#   WRIST_W=288 WRIST_H=216 bash start-collection-cameras.sh
WRIST_W="${WRIST_W:-216}"; WRIST_H="${WRIST_H:-162}"
WRIST_X=$(( CHEST_X + CHEST_W - WRIST_W - 16 ))
WRIST_Y=$(( CHEST_Y + CHEST_H - WRIST_H - 16 ))

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
       ! video/x-raw,format=I420,width=$OUT_W,height=$OUT_H,framerate=$FPS/1 \
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
# 低遅延フラグ必須: ffmpeg は2本のライブ入力をタイムスタンプ同期 (framesync) で
# 待ち合わせるため、既定バッファのままだと2カメラの時計ズレぶん行列が育ち
# ラグが蓄積する。wallclock 刻印 + nobuffer で待ちを殺す。
ffmpeg -hide_banner -loglevel error \
  -fflags nobuffer -flags low_delay -use_wallclock_as_timestamps 1 \
  -f v4l2 -input_format yuyv422 -video_size ${W}x${H} -framerate $FPS -i /dev/video21 \
  -fflags nobuffer -flags low_delay -use_wallclock_as_timestamps 1 \
  -f v4l2 -input_format yuyv422 -video_size ${W}x${H} -framerate $FPS -i /dev/video22 \
  -filter_complex "[0:v]scale=${CHEST_W}:${CHEST_H}[chest];[1:v]scale=${WRIST_W}:${WRIST_H}[wrist];[chest]pad=${OUT_W}:${OUT_H}:${CHEST_X}:${CHEST_Y}:black[bg];[bg][wrist]overlay=${WRIST_X}:${WRIST_Y}:eof_action=pass,format=yuv420p" \
  -fps_mode passthrough \
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
# どれか1本でも死んだら全体を止めて知らせる。従来の `wait`(全員待ち)だと、
# 素流しがUSB瞬断等で静かに落ちても合成だけ生き残り、video20が空のまま
# 「稼働中に見える半死」になる (2026-08-28 実発生・36分気づけなかった)。
wait -n || true
echo "[cams] NG: 分配プロセスのどれかが終了した。全体を停止する (再実行すれば復旧)" >&2
exit 1
