#!/usr/bin/env bash
# OpenArm 恒久ゼロ較正(片side分: follower と leader を順に較正する)。
#
# 何をするか: lerobot-calibrate を呼び、腕を真下に垂らした基準姿勢での生関節角を
# software homing offset として較正ファイルに保存する(モータ側の零位 0xFE は焼かない)。
# 以後、connect() は毎セッションの再ゼロ焼きをやめ(自動判定)、開始基準が
# セッション・電源断をまたいで固定される。teleop/record/rollout 全てに効く。
#
# 手順の注意:
#   1. このセッション中にモータ零位(0xFE)を書いたことがあるなら、先に腕を電源再投入する
#   2. 較正プロンプトで「腕を真下に垂らし・グリッパを閉じ」て ENTER
#   3. follower → leader の順に 2 回プロンプトが出る(それぞれの腕で同じ基準姿勢をとる)
#   4. 較正済みIDがある場合は 'c' を入力して再較正を選ぶ
#
# 使い方: bash scripts/calibrate_openarm.sh <left|right>
set -euo pipefail

SIDE="${1:?usage: bash scripts/calibrate_openarm.sh <left|right>}"
case "$SIDE" in
  left)  FOLLOWER_PORT=can_fol_l; LEADER_PORT=can_lead_l ;;
  right) FOLLOWER_PORT=can_fol_r; LEADER_PORT=can_lead_r ;;
  *) echo "[calibrate] SIDE must be 'left' or 'right', got '$SIDE'" >&2; exit 1 ;;
esac

# インタプリタ解決: teleop venv → conda lerobot env の順で最初に在るもの。
BIN=""
for CAND in \
  "$HOME/projects/lerobot/.venv/bin/lerobot-calibrate" \
  "$HOME/miniconda3/envs/lerobot/bin/lerobot-calibrate"; do
  if [ -x "$CAND" ]; then BIN="$CAND"; break; fi
done
[ -n "$BIN" ] || { echo "[calibrate] lerobot-calibrate not found" >&2; exit 1; }

# rezero_on_connect=false: 接続時に旧フローの零位焼き(0xFE)をさせない
#   (焼いた直後の RAM 零位を基準に offset を取ると、電源断で基準が戻る恐れがあるため)。
# align_on_connect=false: 較正前に整列動作をさせない(フレームが古くても較正に入れる)。
echo "[calibrate] == follower_${SIDE} (${FOLLOWER_PORT}) =="
"$BIN" \
  --robot.type=openarm_follower --robot.port="$FOLLOWER_PORT" \
  --robot.side="$SIDE" --robot.id="follower_${SIDE}" \
  --robot.rezero_on_connect=false --robot.align_on_connect=false

echo "[calibrate] == leader_${SIDE} (${LEADER_PORT}) =="
"$BIN" \
  --teleop.type=openarm_leader --teleop.port="$LEADER_PORT" \
  --teleop.id="leader_${SIDE}" \
  --teleop.rezero_on_connect=false --teleop.align_on_connect=false

echo "[calibrate] done. connect() は以後 homing offset を使い再ゼロ焼きをスキップする(自動判定)。"
echo "[calibrate] 開始姿勢YAMLを更新するなら: bash scripts/capture_initial_pose.sh ${SIDE}"
