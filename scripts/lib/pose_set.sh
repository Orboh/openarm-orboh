# 開始姿勢セット（config/pose_sets/<name>/）の解決と照合。bash から source して使う。
# セットの中身・状態（draft / frozen）・運用は config/pose_sets/README.md と scripts/pose_set.py の冒頭。
#
#   source "$REPO/scripts/lib/pose_set.sh"
#   pose_set_load required|current      # 推論系は required（POSE_SET 必須）。テレオペ・収集系は current（未指定なら CURRENT）
#   pose_set_check rollout|teleop       # 開始姿勢・較正を照合（rollout は不一致で止める・teleop は警告のみ）
#   pose_set_prepare_record "$SCENE"    # 収集前: draft なら今の較正・シーン・版を取り込み、frozen なら全部を照合
#   pose_set_freeze "$RUN_DIR"          # 収集後: データセットが作られていれば凍結（record が呼ぶ）
# 結果: POSE_SET PS_DIR LPOSE RPOSE（無い側は空）PS_MODEL_DIR（~ 展開済み）

PS_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PS_ROOT="${PS_ROOT:-$PS_REPO/config/pose_sets}"
PS_TOOL="$PS_REPO/scripts/pose_set.py"

pose_set_get() {  # pose_set_get <key> → set.yaml の値（無ければ空）
  [ -f "$PS_DIR/set.yaml" ] || return 0
  sed -n "s/^$1:[[:space:]]*//p" "$PS_DIR/set.yaml" | head -1
}

pose_set_list() { (cd "$PS_ROOT" && ls -d */ 2>/dev/null | tr -d / | tr '\n' ' '); }

pose_set_load() {
  local mode="${1:-current}"
  if [ -z "${POSE_SET:-}" ]; then
    if [ "$mode" = required ]; then
      echo "[pose-set] POSE_SET を指定すること（モデルと開始姿勢の組を明示する）。候補: $(pose_set_list)" >&2
      return 1
    fi
    POSE_SET="$(tr -d '[:space:]' < "$PS_ROOT/CURRENT" 2>/dev/null || true)"
    [ -n "$POSE_SET" ] || { echo "[pose-set] $PS_ROOT/CURRENT が無い" >&2; return 1; }
  fi
  PS_DIR="$PS_ROOT/$POSE_SET"
  [ -f "$PS_DIR/set.yaml" ] || { echo "[pose-set] セットが無い: $PS_DIR（候補: $(pose_set_list)）" >&2; return 1; }
  LPOSE=""; RPOSE=""
  [ -f "$PS_DIR/left.yaml" ] && LPOSE="$PS_DIR/left.yaml"
  [ -f "$PS_DIR/right.yaml" ] && RPOSE="$PS_DIR/right.yaml"
  PS_MODEL_DIR="$(pose_set_get model_dir)"; PS_MODEL_DIR="${PS_MODEL_DIR/#\~/$HOME}"
  echo "[pose-set] $POSE_SET（$(pose_set_get status)） left=${LPOSE:-なし} right=${RPOSE:-なし}"
  export POSE_SET PS_DIR LPOSE RPOSE PS_MODEL_DIR
}

pose_set_check() { python3 "$PS_TOOL" check "$POSE_SET" --mode "${1:-rollout}"; }

pose_set_prepare_record() {
  local scene="${1:-}"
  python3 "$PS_TOOL" check "$POSE_SET" --mode record ${scene:+--scene "$scene"}
}

pose_set_freeze() { python3 "$PS_TOOL" freeze "$POSE_SET" --from-run "$1" || true; }
