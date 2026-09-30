#!/usr/bin/env bash
# OpenArm 恒久ゼロ較正(片side分: follower と leader を順に較正する)。
#
# 何をするか: lerobot-calibrate を呼び、基準構成での生関節角から software homing offset を
# 較正ファイルに保存する(モータ側の零位 0xFE は焼かない)。
# 以後、connect() は毎セッションの再ゼロ焼きをやめ(自動判定)、開始基準が
# セッション・電源断をまたいで固定される。teleop/record/rollout 全てに効く。
#
# アンカー(基準の取り方)は 2 種類:
#   hang_down(既定):   腕を真下に垂らした姿勢の生関節角をそのまま offset にする。
#                       人手で姿勢を再現するため精度は目視依存。従来動作・完全互換。
#   bump_to_stop(opt-in): 腕が自走して各アーム関節を機械ストッパに当て、既知のストッパ角から
#                       offset を解析的に決める(最高確度・治具不要)。
#                       ⚠ 腕が自動で全関節をストッパまで掃引する。周囲を完全に空けること。
#                       ⚠ グリッパは絶対にバンプしない(公式スクリプトのスキップが実機破損の原因)。
#                         事前に openarm_gripper_zero.py で「完全閉鎖で flash zero」を焼いておくこと。
#                       ⚠ しきい値・ストッパ角は個体調整が必要(docs/thor-bump-calibration-runbook.md)。
#
# 手順の注意:
#   1. このセッション中にモータ零位(0xFE)を書いたことがあるなら、先に腕を電源再投入する
#   2. hang_down: 較正プロンプトで「腕を真下に垂らし・グリッパを閉じ」て ENTER
#      bump_to_stop: ワークスペースを空け、警告プロンプトを読んで ENTER(腕が自走する)
#   3. follower → leader の順に 2 回プロンプトが出る
#   4. 較正済みIDがある場合は 'c' を入力して再較正を選ぶ
#
# 使い方:
#   bash scripts/calibrate_openarm.sh <left|right>                  # hang_down(既定)
#   bash scripts/calibrate_openarm.sh <left|right> bump_to_stop     # bump-to-stop アンカー
#   ANCHOR=bump_to_stop bash scripts/calibrate_openarm.sh <left|right>   # env でも可
set -euo pipefail

SIDE="${1:?usage: bash scripts/calibrate_openarm.sh <left|right> [hang_down|bump_to_stop]}"
ANCHOR="${2:-${ANCHOR:-hang_down}}"
# accept bump-to-stop spelling too
ANCHOR="${ANCHOR//-/_}"
case "$SIDE" in
  left)  FOLLOWER_PORT=can_fol_l; LEADER_PORT=can_lead_l ;;
  right) FOLLOWER_PORT=can_fol_r; LEADER_PORT=can_lead_r ;;
  *) echo "[calibrate] SIDE must be 'left' or 'right', got '$SIDE'" >&2; exit 1 ;;
esac
case "$ANCHOR" in
  hang_down|bump_to_stop) ;;
  *) echo "[calibrate] ANCHOR must be 'hang_down' or 'bump_to_stop', got '$ANCHOR'" >&2; exit 1 ;;
esac

# インタプリタ解決: teleop venv → conda lerobot env の順で最初に在るもの。
BIN=""
for CAND in \
  "$HOME/projects/lerobot/.venv/bin/lerobot-calibrate" \
  "$HOME/miniconda3/envs/lerobot/bin/lerobot-calibrate"; do
  if [ -x "$CAND" ]; then BIN="$CAND"; break; fi
done
[ -n "$BIN" ] || { echo "[calibrate] lerobot-calibrate not found" >&2; exit 1; }

if [ "$ANCHOR" = "bump_to_stop" ]; then
  echo "[calibrate] anchor=bump_to_stop: 腕が自走して全アーム関節をストッパまで掃引する。"
  echo "[calibrate]   - ワークスペースを空け、電源スイッチ/Ctrl-C に手が届く体勢で行うこと。"
  echo "[calibrate]   - グリッパの flash zero(完全閉鎖)を焼いていないなら先に:"
  echo "[calibrate]       python scripts/openarm_gripper_zero.py <canX>  (follower/leader 両方)"
fi

# rezero_on_connect=false: 接続時に旧フローの零位焼き(0xFE)をさせない
#   (焼いた直後の RAM 零位を基準に offset を取ると、電源断で基準が戻る恐れがあるため)。
# align_on_connect=false: 較正前に整列動作をさせない(フレームが古くても較正に入れる)。
# calibration_anchor: hang_down(既定・従来動作) / bump_to_stop(opt-in・自走掃引)。
# leader は side フィールドを持たないため gravity_side でシーケンス側を明示する
#   (bump_to_stop で side を誤ると誤ったストッパへ向かう)。
# 較正ファイルは同じ名前へ上書きされる。上書き前に必ず退避する（2026-09-21 に右腕の 08-04 版を
# 退避なしで上書きし、その較正で録ったモデルの組が壊れた）。学習済みモデルと組の較正は開始姿勢セット
# （config/pose_sets/<set>/calib/）にも写しがある。
CALIB_ROOT="${CALIB_ROOT:-$HOME/.cache/huggingface/lerobot/calibration}"
STAMP="$(date +%Y%m%d-%H%M%S)"
for _f in "$CALIB_ROOT/robots/openarm_follower/follower_${SIDE}.json" "$CALIB_ROOT/teleoperators/openarm_leader/leader_${SIDE}.json"; do
  if [ -f "$_f" ]; then
    _b="$_f.bak-$STAMP"; _k=1
    while [ -e "$_b" ]; do _b="$_f.bak-$STAMP-$_k"; _k=$((_k+1)); done
    cp -p "$_f" "$_b"
    echo "[calibrate] 退避: $_b"
  fi
done

echo "[calibrate] == follower_${SIDE} (${FOLLOWER_PORT}) anchor=${ANCHOR} =="
"$BIN" \
  --robot.type=openarm_follower --robot.port="$FOLLOWER_PORT" \
  --robot.side="$SIDE" --robot.id="follower_${SIDE}" \
  --robot.calibration_anchor="$ANCHOR" \
  --robot.rezero_on_connect=false --robot.align_on_connect=false

echo "[calibrate] == leader_${SIDE} (${LEADER_PORT}) anchor=${ANCHOR} =="
"$BIN" \
  --teleop.type=openarm_leader --teleop.port="$LEADER_PORT" \
  --teleop.id="leader_${SIDE}" \
  --teleop.calibration_anchor="$ANCHOR" \
  --teleop.gravity_side="$SIDE" \
  --teleop.rezero_on_connect=false --teleop.align_on_connect=false

echo "[calibrate] done. connect() は以後 homing offset を使い再ゼロ焼きをスキップする(自動判定)。"
# 較正が変わると、凍結済みの開始姿勢セット（前の較正で録ったデータ・モデルの組）とは座標が割れる。
# そのセットは残したまま、新しい較正の組は新しいセットで始める。
_cur="$(tr -d '[:space:]' < "$(cd "$(dirname "$0")/.." && pwd)/config/pose_sets/CURRENT" 2>/dev/null || true)"
echo "[calibrate] 較正が変わった。凍結済みの開始姿勢セットは前の較正と組のまま残る（収集・推論は較正不一致で止まる）。"
echo "[calibrate]   新しい較正で録るなら新しいセット: python3 scripts/pose_set.py new <名前> --from ${_cur:-<元のセット>} && python3 scripts/pose_set.py current <名前>"
echo "[calibrate]   前のセットのモデルを回すときは: python3 scripts/pose_set.py restore-calib <セット>"
if [ "$ANCHOR" = "bump_to_stop" ]; then
  echo "[calibrate] 較正後の必須確認(±0.05 rad 基準検証・電源断永続チェック):"
  echo "[calibrate]   docs/thor-bump-calibration-runbook.md 参照"
fi
