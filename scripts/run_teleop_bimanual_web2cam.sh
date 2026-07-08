#!/usr/bin/env bash
# 両腕 bimanual lerobot teleop（重力補償・カメラ無し）と、chest / wrist(D405) の2ブラウザ配信を同時に立ち上げる。
#
# なぜ「カメラ無し teleop」なのか:
#   lerobot に --robot.cameras を渡すと、まず RealSense は 1デバイス1プロセス占有のため
#   stream_chest.py と競合する。さらに表示が rerun になり、ヘッドレスの Thor では viewer が
#   出ずに rr.spawn が失敗する。そこで teleop 側はカメラを開かず（run_teleop_bimanual.sh 相当）、
#   カメラは stream_chest.py を2本立てて chest(D435i) と wrist(D405) をそれぞれ占有し、
#   MJPEG をブラウザへ配信する。この分担なら rerun 不要で、2タブで両カメラを見ながら
#   teleop できる。chest と wrist は別デバイス（別シリアル）なので2本の配信同士も競合しない。
#
# ⚠ 起動時は両腕を真下に垂らす（NVRAM 零位が焼かれる）。右系統 CAN は bus-off 既往。
#
# 前提:
#   - 先に  sudo bash scripts/canup.sh  で CAN を立ち上げておく（4ch 全て UP・can_check 通過）。
#   - lerobot venv は ~/projects/lerobot/.venv（run_teleop_bimanual.sh がそのバイナリを直接叩く）。
#   - stream_chest.py は system python3（pyrealsense2 + cv2）で動く。venv 不要。
#
# 使い方:
#   bash scripts/run_teleop_bimanual_web2cam.sh [left_scale] [right_scale]
#     scale = 重力補償スケール（既定 両腕1.2。右だけ下げる例: ... 1.2 1.0）。
#   環境変数で上書き可: CHEST_SERIAL / CHEST_PORT / WRIST_SERIAL / WRIST_PORT
#
# ブラウザ（同一LAN もしくは tailnet の PC）:
#   chest = http://<ThorのIP>:8080   wrist = http://<ThorのIP>:8081
#   （起動時に候補 IP を表示する）
#
# 停止: Ctrl-C。teleop の停止と同時に2つの配信も落とす（trap で後片付け）。

set -uo pipefail

LSCALE="${1:-1.2}"
RSCALE="${2:-$LSCALE}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
STREAM="$REPO/scripts/stream_chest.py"
CHEST_SERIAL="${CHEST_SERIAL:-344422070898}"; CHEST_PORT="${CHEST_PORT:-8080}"
WRIST_SERIAL="${WRIST_SERIAL:-260322272302}"; WRIST_PORT="${WRIST_PORT:-8081}"

if [[ ! -f "$STREAM" ]]; then
  echo "[web2cam] stream_chest.py が見つからない: $STREAM （git pull で取得済みか確認）" >&2
  exit 1
fi

# 閲覧用 IP 候補。LAN(192.168.x) を先頭に、無ければ tailnet(100.x)、それも無ければ先頭アドレス。
ALL_IPS="$(hostname -I 2>/dev/null)"
LAN_IP="$(echo "$ALL_IPS" | tr ' ' '\n' | grep -E '^192\.168\.' | head -n1)"
TS_IP="$(echo "$ALL_IPS" | tr ' ' '\n' | grep -E '^100\.' | head -n1)"
IP="${LAN_IP:-${TS_IP:-$(echo "$ALL_IPS" | awk '{print $1}')}}"
IP="${IP:-<ThorのIP>}"

_pids=()
cleanup() {
  echo ""
  echo "[web2cam] 配信を停止中…"
  for p in "${_pids[@]:-}"; do
    kill "$p" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

echo "[web2cam] chest 配信 :$CHEST_PORT (serial $CHEST_SERIAL)"
python3 "$STREAM" --serial "$CHEST_SERIAL" --port "$CHEST_PORT" >/tmp/web2cam-chest.log 2>&1 &
_pids+=($!)

echo "[web2cam] wrist 配信 :$WRIST_PORT (serial $WRIST_SERIAL)"
python3 "$STREAM" --serial "$WRIST_SERIAL" --port "$WRIST_PORT" >/tmp/web2cam-wrist.log 2>&1 &
_pids+=($!)

# 配信が実際に起動できたか確認する（カメラを他プロセスが掴んでいると即死する）。
sleep 3
for p in "${_pids[@]}"; do
  if ! kill -0 "$p" 2>/dev/null; then
    echo "[web2cam] 配信の起動に失敗した。/tmp/web2cam-chest.log と /tmp/web2cam-wrist.log を確認する。" >&2
    echo "[web2cam] 典型原因: 既存の stream_chest.py が同じカメラを掴んでいる（pkill -f stream_chest.py）／serial 不一致／未接続。" >&2
    exit 1
  fi
done

echo "[web2cam] ブラウザで開く:  chest=http://$IP:$CHEST_PORT   wrist=http://$IP:$WRIST_PORT"
if [[ -n "${ALL_IPS// /}" ]]; then
  echo "[web2cam] IP 候補: $ALL_IPS"
fi
echo "[web2cam] 起動時整列: 左leader→右leader→左follower→右follower の順に初期姿勢（肘36°）へ各約2.2秒かけて自動で動く。両腕まわりの空間を確保して待つ。"
echo "[web2cam] teleop 開始（両腕・カメラ無し・重力補償 L=$LSCALE R=$RSCALE・fps200）。両腕を真下に垂らして開始。Ctrl-C で teleop も配信も全部停止。"
bash "$REPO/scripts/run_teleop_bimanual.sh" "$LSCALE" "$RSCALE"
