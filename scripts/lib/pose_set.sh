# 開始姿勢セット（config/pose_sets/<name>/）の解決。bash から source して使う。
#
# 1 モデル = 1 セット。セットは「そのモデルの学習データを録った時の開始姿勢 YAML（left.yaml /
# right.yaml）」と「その時の較正・コード・モデル・データセット」（set.yaml）の組。
# 以前は全スクリプトが config/initial_pose_{left,right}.yaml という 1 組の固定名を読み、その中身を
# 撮り直すたびに上書きしていた。その結果、過去モデルを回すと黙って新しい姿勢で起動し、収集時と
# 推論時の開始姿勢が割れる（発覚が最も遅い種類の事故）。2026-09-30 にセットへ移した。
#
# 使い方:
#   source "$REPO/scripts/lib/pose_set.sh"
#   pose_set_load required   # 推論系: POSE_SET の明示が必須（既定に落とさない）
#   pose_set_load current    # テレオペ・収集系: POSE_SET 未指定なら config/pose_sets/CURRENT
#   pose_set_check_calib strict|warn
# 結果: POSE_SET PS_DIR LPOSE RPOSE（無い側は空）PS_MODEL_DIR（set.yaml の model_dir・~ 展開済み）

PS_ROOT="${PS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/config/pose_sets}"

pose_set_get() {  # pose_set_get <key> → set.yaml の値（無ければ空）
  local f="$PS_DIR/set.yaml"
  [ -f "$f" ] || return 0
  sed -n "s/^$1:[[:space:]]*//p" "$f" | head -1
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
  echo "[pose-set] $POSE_SET  left=${LPOSE:-なし} right=${RPOSE:-なし}"
  export POSE_SET PS_DIR LPOSE RPOSE PS_MODEL_DIR
}

# 今の較正ファイルの sha256 先頭 12 桁が set.yaml の calib_* と一致するか。
# 較正が違えば同じ関節角の数値が違う物理姿勢を指す＝開始姿勢を揃えても座標が割れる。
pose_set_check_calib() {
  local mode="${1:-strict}" root="${CALIB_ROOT:-$HOME/.cache/huggingface/lerobot/calibration}"
  local bad=0 key want role side f got
  if [ "$(pose_set_get calib_status)" = lost ]; then
    echo "[pose-set] $POSE_SET の収集時の較正ファイルは現存しない: $(pose_set_get note)" >&2
    bad=1
  fi
  for key in calib_follower_left calib_follower_right calib_leader_left calib_leader_right; do
    want="$(pose_set_get $key)"; [ -n "$want" ] || continue
    role="${key#calib_}"; side="${role#*_}"; role="${role%_*}"
    if [ "$role" = follower ]; then f="$root/robots/openarm_follower/follower_$side.json"
    else f="$root/teleoperators/openarm_leader/leader_$side.json"; fi
    got="$(sha256sum "$f" 2>/dev/null | cut -c1-12)"
    if [ "$got" != "$want" ]; then
      echo "[pose-set] 較正が収集時と違う: $role/$side 今=${got:-ファイル無し} 収集時=$want" >&2
      bad=1
    fi
  done
  [ "$bad" = 0 ] && { echo "[pose-set] 較正は収集時と一致"; return 0; }
  if [ "$mode" = warn ] || [ "${ALLOW_CALIB_MISMATCH:-false}" = true ]; then
    echo "[pose-set] 警告のみで続行（mode=$mode ALLOW_CALIB_MISMATCH=${ALLOW_CALIB_MISMATCH:-false}）" >&2
    return 0
  fi
  echo "[pose-set] 止める。承知の上で回すなら ALLOW_CALIB_MISMATCH=true" >&2
  return 1
}
