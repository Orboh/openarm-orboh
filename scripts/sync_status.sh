#!/usr/bin/env bash
# 3リポ(lerobot / openarm-orboh / XRoboToolkit-Orin-Video-Sender)×両マシンの
# 同期負債を1コマンドで見る。
#
#   bash scripts/sync_status.sh              # このマシンのみ
#   REMOTE=thor bash scripts/sync_status.sh  # リモートも(読み取りのみ・ssh)
#   NOFETCH=true ...                         # ネットに出ない
#   PROJECTS_DIR=... bash scripts/sync_status.sh  # リポの基底を上書き(既定 $HOME/projects)
#
# リモート側に check_repo_sync.sh が未展開でも動く: ローカルの実体を
# `ssh bash -s` で流し込んで実行する(リモートに何も書かない)。
# 3本目(sender)は PICO 映像経路を使うマシン(主に Thor)にしかない想定なので、
# ディレクトリが無いリポは負債扱いにせず「このマシンには無い」と報告してスキップする。
# 終了コード: 0=全て清潔 / 1=どこかに負債
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
CHECK="$HERE/check_repo_sync.sh"
PROJECTS_DIR="${PROJECTS_DIR:-$HOME/projects}"
REPO_NAMES=(lerobot openarm-orboh XRoboToolkit-Orin-Video-Sender)
DEBT=0

echo "== $(hostname)（ローカル） =="
for name in "${REPO_NAMES[@]}"; do
  r="$PROJECTS_DIR/$name"
  if [ ! -d "$r" ]; then
    echo "  [sync] $name: (このマシンには無い)"
    continue
  fi
  NOFETCH="${NOFETCH:-false}" bash "$CHECK" "$r"; rc=$?
  [ $rc -eq 1 ] && DEBT=1
done

if [ -n "${REMOTE:-}" ]; then
  echo "== $REMOTE（ssh・読み取りのみ） =="
  for name in "${REPO_NAMES[@]}"; do
    if ! timeout 15 ssh -o ConnectTimeout=10 "$REMOTE" \
        "test -d \"\$HOME/projects/$name\""; then
      echo "  [sync] $name: (このマシンには無い)"
      continue
    fi
    if timeout 45 ssh -o ConnectTimeout=10 "$REMOTE" \
        "NOFETCH='${NOFETCH:-false}' bash -s -- \"\$HOME/projects/$name\"" < "$CHECK"; then :; else
      rc=$?
      [ $rc -eq 1 ] && DEBT=1
      [ $rc -ge 2 ] && [ $rc -ne 2 ] && echo "  [sync] $REMOTE:projects/$name へ到達できない(ssh $rc)"
    fi
  done
fi

exit $DEBT
