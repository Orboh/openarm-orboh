#!/usr/bin/env bash
# 1つの git リポジトリの「同期負債」を判定する。
#
# なぜ要るか(2026-09-01): セッションが Vector と Thor を跨いで作業する運用で、
# 実害が3種類たまった——①push されない完了コミットが数日残る ②未追跡ファイルが
# 上流で追跡化され pull を塞ぐ ③ブランチ誤認のまま commit して「push した」と誤報。
# どれも「使う時点で状態が見えない」ことが原因なので、判定を1コマンドにする。
#
# 使い方:
#   bash scripts/check_repo_sync.sh <repo-dir>
#   NOFETCH=true ...   # ネットに出ない(前回 fetch 時点の上流と比較)
# 終了コード: 0=清潔で同期済み / 1=負債あり / 2=判定不能(上流なし等)
set -u

REPO_DIR="${1:?usage: check_repo_sync.sh <repo-dir>}"
cd "$REPO_DIR" || { echo "  [sync] リポジトリに入れない: $REPO_DIR"; exit 2; }
NAME="$(basename "$(pwd)")"

BRANCH="$(git branch --show-current 2>/dev/null)"
if [ -z "$BRANCH" ]; then
  echo "  [sync] $NAME: detached HEAD ($(git rev-parse --short HEAD 2>/dev/null))"
  exit 2
fi

# fetch は best-effort。現場の Thor はオフラインがあるので、失敗しても
# 「前回 fetch 時点との比較」として続行する(その旨を出す)。
FETCHED="fetch済"
if [ "${NOFETCH:-false}" != "true" ]; then
  timeout 10 git fetch -q 2>/dev/null || FETCHED="fetch不可(前回時点と比較)"
else
  FETCHED="fetch省略"
fi

if ! git rev-parse --abbrev-ref '@{u}' >/dev/null 2>&1; then
  echo "  [sync] $NAME: $BRANCH に上流(upstream)が無い ★push 先を設定すること"
  exit 2
fi

read -r BEHIND AHEAD <<<"$(git rev-list --left-right --count '@{u}...HEAD' 2>/dev/null)"
DIRTY=$(git status --porcelain 2>/dev/null | grep -cv '^?? ')
UNTRACKED=$(git ls-files --others --exclude-standard | grep -cv '^\.claude/')

# 未追跡ファイルが上流 tip では追跡されている = pull を塞ぐ型(2026-09-01 に実際に発生)
COLLIDE="$(comm -12 \
  <(git ls-files --others --exclude-standard | grep -v '^\.claude/' | sort) \
  <(git ls-tree -r --name-only '@{u}' | sort) )"

DEBT=0
LINE="  [sync] $NAME: $BRANCH  ($FETCHED)"
[ "${AHEAD:-0}" -gt 0 ]  && { LINE="$LINE  ★未push ${AHEAD}件"; DEBT=1; }
[ "${BEHIND:-0}" -gt 0 ] && { LINE="$LINE  ★上流より ${BEHIND}件古い"; DEBT=1; }
[ "$DIRTY" -gt 0 ]       && { LINE="$LINE  ★未コミット ${DIRTY}件"; DEBT=1; }
[ "$UNTRACKED" -gt 0 ]   && LINE="$LINE  未追跡 ${UNTRACKED}件"
echo "$LINE"

if [ "${AHEAD:-0}" -gt 0 ]; then
  git log --oneline '@{u}..HEAD' | head -3 | sed 's/^/         未push → /'
fi
if [ "${BEHIND:-0}" -gt 0 ]; then
  git log --oneline 'HEAD..@{u}' | head -3 | sed 's/^/         未取込 → /'
fi
if [ -n "$COLLIDE" ]; then
  DEBT=1
  echo "$COLLIDE" | sed 's/^/         ★未追跡が上流と衝突(pull を塞ぐ) → /'
  echo "         → 退避してから pull: mv <file> <file>.local"
fi
exit $DEBT
