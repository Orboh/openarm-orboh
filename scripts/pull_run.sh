#!/usr/bin/env bash
# 収集セッションの軽量メタを Thor から Vault へ回収する。
#
# 設計正本: orboh-company-vault
#   07Member's-free-space/Kento/02-Tech-Plan/02VLA/collection-recording-manual.md
#   (§収集構成 / 実装バックログ4)
#
# なぜ要るか: 収集の記録はすべて Thor に生まれるが、Thor は現場に持ち出すと
# ネットから切れて他マシンから見えず、故障・再セットアップで中身は消える中継地。
# **Thor にだけある情報はまだ守られていない**。人が読む軽い6点を Vault へ、
# 重いデータ本体は HF へ、と分けるのがこの体制の骨格。
#
# 回収するもの(ホワイトリスト):
#   command.sh / manifest.json / console.log / start-pose-*.jsonl
#   ledger.csv / notes.md / setup/**
# データセット本体(parquet+動画)は回収しない。HF へ push する経路が別にある。
#
# 使い方(Vault のあるマシン = Vector か Mac で走らせる):
#   bash scripts/pull_run.sh                      # Thor 側のセッション一覧を出す
#   bash scripts/pull_run.sh 20260803T101500Z-bottle-desk-v1
#   DRY_RUN=true bash scripts/pull_run.sh <run>   # 何をするかだけ表示
#   FORCE=true   bash scripts/pull_run.sh <run>   # Vault 側の既存ファイルも上書き(危険)
set -euo pipefail

REMOTE="${REMOTE:-thor}"
REMOTE_DIR="${REMOTE_DIR:-collections}"   # リモートの $HOME 相対
# Vault のパスに含まれる 07Member's-free-space のアポストロフィに注意。
# "${VAR:-...}" の既定値の中では ' が literal にならずクォート開始として解釈され、
# スクリプト全体が「閉じていない引用符」で構文エラーになる。既定値は外に出す。
VAULT_DEFAULT="$HOME/Documents/orboh-company-vault/07Member's-free-space/Kento/02-Tech-Plan/02VLA/gr00t-openarm/collections"
VAULT_DEST="${VAULT_DEST:-$VAULT_DEFAULT}"
FINALIZE="${FINALIZE:-true}"              # 回収前に台帳と notes を Thor 側で用意する
DRY_RUN="${DRY_RUN:-false}"
FORCE="${FORCE:-false}"

RUN="${1:-}"
if [ -z "$RUN" ]; then
  echo "[pull] $REMOTE 上の収集セッション:"
  ssh "$REMOTE" "ls -1 ~/$REMOTE_DIR 2>/dev/null" | sed 's/^/[pull]   /' || echo "[pull]   (取得できない)"
  echo "[pull] 使い方: bash scripts/pull_run.sh <セッション名>"
  exit 0
fi

SRC="\$HOME/$REMOTE_DIR/$RUN"
DEST="$VAULT_DEST/$RUN"

echo "[pull] from: $REMOTE:~/$REMOTE_DIR/$RUN"
echo "[pull] to  : $DEST"

if ! ssh "$REMOTE" "test -d $SRC"; then
  echo "[pull] リモートにセッションが無い: $REMOTE:~/$REMOTE_DIR/$RUN" >&2
  exit 1
fi

# 台帳(ledger.csv)と notes.md はデータセットを読まないと作れないので、
# データセットのある Thor 側で用意してから引く。
if [ "$FINALIZE" = "true" ]; then
  echo "[pull] Thor 側で台帳と notes を用意する(既存ファイルは上書きしない)"
  if [ "$DRY_RUN" = "true" ]; then
    echo "[pull] (dry-run) ssh $REMOTE finalize_run.py --run-dir ~/$REMOTE_DIR/$RUN"
  else
    ssh "$REMOTE" "cd ~/projects/openarm-orboh && python3 scripts/finalize_run.py --run-dir $SRC" \
      2>&1 | sed 's/^/[pull] finalize: /' || echo "[pull] 警告: finalize が失敗した(回収は続ける)"
  fi
fi

mkdir -p "$DEST"

# Vault 側に既にあるファイルは既定で上書きしない。回収後に人が埋めた
# ledger.csv の成否・対象物位置・所見、manifest.json の human_fields は
# **復元できない**(再録回数は復元すらできない)。Thor 側の空の骨組みで
# 潰す事故を、既定の挙動として防いでおく。
RSYNC_ARGS=(-av --prune-empty-dirs)
if [ "$FORCE" = "true" ]; then
  echo "[pull] 警告: FORCE=true。Vault 側の既存ファイルを Thor の内容で上書きする。"
  echo "[pull]        人が埋めた台帳・所見・human_fields は失われる。"
else
  RSYNC_ARGS+=(--ignore-existing)
fi
[ "$DRY_RUN" = "true" ] && RSYNC_ARGS+=(--dry-run)

rsync "${RSYNC_ARGS[@]}" \
  --include='*/' \
  --include='command.sh' \
  --include='manifest.json' \
  --include='console.log' \
  --include='start-pose-*.jsonl' \
  --include='ledger.csv' \
  --include='notes.md' \
  --include='setup/**' \
  --exclude='*' \
  "$REMOTE:$SRC/" "$DEST/" | sed 's/^/[pull] /'

echo "[pull] 回収したもの:"
ls -1 "$DEST" 2>/dev/null | sed 's/^/[pull]   /' || true
echo "[pull] 次にやること:"
echo "[pull]   1. ledger.csv の人手3列(success / object_pos / note)をその日のうちに埋める"
echo "[pull]   2. manifest.json の human_fields(散らした範囲・異常)を埋める"
echo "[pull]   3. setup/ に部屋の広角写真を1枚置く"
echo "[pull]   4. 埋め終えたら notes.md の frontmatter を ledger_complete: true にする"
echo "[pull]   Vault は obsidian-git が自動で commit/push する(手動コミット不要)"
