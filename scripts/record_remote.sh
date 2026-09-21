#!/usr/bin/env bash
# 遠隔テレオペ収集の起動ラッパー (Thor 端末③)。長い環境変数列の手打ちをやめる1枚
# (改行の泣き別れで変数が渡らず OPERATOR 未指定エラー等になる事故が 2026-08-27/28 に実発生)。
# 焼き込み: REMOTE(リーダー=操作側PCの leader_agent) / WRIST(D405) /
#           分配リグの仮想カメラ2本 (chest=/dev/video21, wrist=/dev/video22) / SIDE=right。
# 前提: start-collection-cameras.sh 稼働中・操作側PCで run_leader_agent_<side>.sh 稼働中。
#
# 使い方:
#   OPERATOR=<名前> SCENE=<scenes/のname> REPO_ID=Orboh/<dataset> TASK="<言語指示>" \
#     bash scripts/record_remote.sh <scene-tag>                                    # 本番収集
#   ※ REPO_ID/TASK を省略すると record_pick_place_openarm.sh の既定(オクラ用)で録られる
#   CHECK=true bash scripts/record_remote.sh                                          # 起動チェック(2本15秒・tmpデータセット)
# NUM_EPISODES / EPISODE_TIME_S / SIDE / REPO_ID 等は env でそのまま上書きできる
# (中身は record_pick_place_openarm.sh なので変数の意味・ガードはそちらが正)。
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

if [ "${CHECK:-false}" = "true" ]; then
  # 起動チェック: 条件記録のガード (OPERATOR/SCENE 必須) を dry-run 値で満たし、
  # 本番データセットを汚さない tmp 名に書く。
  export OPERATOR="${OPERATOR:-Kento}" SCENE="${SCENE:-none}"
  export NUM_EPISODES="${NUM_EPISODES:-2}" EPISODE_TIME_S="${EPISODE_TIME_S:-15}"
  export REPO_ID="${REPO_ID:-Orboh/openarm_startup_check_tmp}"
  TAG="${1:-startup-check}"
else
  TAG="${1:?usage: OPERATOR=<名前> SCENE=<name> bash scripts/record_remote.sh <scene-tag>}"
fi

REMOTE=true WRIST=true \
SIDE="${SIDE:-right}" \
CHEST_V4L2="${CHEST_V4L2:-/dev/video21}" WRIST_V4L2="${WRIST_V4L2:-/dev/video22}" \
bash "$HERE/record_pick_place_openarm.sh" "$TAG"
