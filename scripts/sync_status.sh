#!/usr/bin/env bash
# 両リポ(lerobot / openarm-orboh)×両マシンの同期負債を1コマンドで見る。
#
#   bash scripts/sync_status.sh              # このマシンのみ
#   REMOTE=thor bash scripts/sync_status.sh  # リモートも(読み取りのみ・ssh)
#   NOFETCH=true ...                         # ネットに出ない
#
# リモート側に check_repo_sync.sh が未展開でも動く: ローカルの実体を
# `ssh bash -s` で流し込んで実行する(リモートに何も書かない)。
# 終了コード: 0=全て清潔 / 1=どこかに負債
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
CHECK="$HERE/check_repo_sync.sh"
REPOS=("$HOME/projects/lerobot" "$HOME/projects/openarm-orboh")
DEBT=0

echo "== $(hostname)（ローカル） =="
for r in "${REPOS[@]}"; do
  NOFETCH="${NOFETCH:-false}" bash "$CHECK" "$r"; rc=$?
  [ $rc -eq 1 ] && DEBT=1
done

if [ -n "${REMOTE:-}" ]; then
  echo "== $REMOTE（ssh・読み取りのみ） =="
  for rel in "projects/lerobot" "projects/openarm-orboh"; do
    if timeout 45 ssh -o ConnectTimeout=10 "$REMOTE" \
        "NOFETCH='${NOFETCH:-false}' bash -s -- \"\$HOME/$rel\"" < "$CHECK"; then :; else
      rc=$?
      [ $rc -eq 1 ] && DEBT=1
      [ $rc -ge 2 ] && [ $rc -ne 2 ] && echo "  [sync] $REMOTE:$rel へ到達できない(ssh $rc)"
    fi
  done
fi

exit $DEBT
