#!/usr/bin/env bash
# openarm_embodiment（GR00T の NEW_EMBODIMENT 設定）を Isaac-GR00T の clone 配下に配置する。
# 本リポジトリの gr00t/openarm_embodiment を正本とし、GR00T 側へは symlink を張る。
#
# usage:
#   bash scripts/install_gr00t_embodiment.sh
#   GR00T_ROOT=~/projects/Isaac-GR00T bash scripts/install_gr00t_embodiment.sh
#   FORCE=1 ... （GR00T 側に実体ディレクトリが既にある場合に退避して置き換える）
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO_ROOT/gr00t/openarm_embodiment"
GR00T_ROOT="${GR00T_ROOT:-$HOME/projects/Isaac-GR00T-n1.7}"
DST="$GR00T_ROOT/openarm_embodiment"

[ -d "$SRC" ] || { echo "[FAIL] 正本が見つからない: $SRC"; exit 1; }
[ -d "$GR00T_ROOT" ] || { echo "[FAIL] GR00T clone が見つからない: $GR00T_ROOT（GR00T_ROOT= で指定する）"; exit 1; }

if [ -L "$DST" ]; then
  ln -sfn "$SRC" "$DST"
  echo "[OK] symlink を張り直した: $DST -> $SRC"
elif [ -e "$DST" ]; then
  # 実体がある = このスクリプト以前に手で置いたもの。黙って消さない。
  if [ "${FORCE:-0}" = "1" ]; then
    BAK="$DST.bak-$(date -u +%Y%m%dT%H%M%SZ)"
    mv "$DST" "$BAK"
    ln -s "$SRC" "$DST"
    echo "[OK] 既存の実体を $BAK へ退避し、symlink を張った"
  else
    echo "[SKIP] $DST に実体のディレクトリがある。中身を突き合わせてから FORCE=1 で再実行する:"
    echo "       diff -r \"$SRC\" \"$DST\""
    exit 2
  fi
else
  ln -s "$SRC" "$DST"
  echo "[OK] symlink を張った: $DST -> $SRC"
fi

echo "[CHECK] $(ls "$DST"/ | tr '\n' ' ')"
