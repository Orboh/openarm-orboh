#!/usr/bin/env bash
# 片腕の teleop 開始姿勢(肘曲げ・腕上げ等)を手で動かしてキャプチャする。
# 重力補償で腕を無重力ホールドしながら望む姿勢へ動かし、ENTERで関節角を保存する。
# 保存先は開始姿勢セット config/pose_sets/<POSE_SET>/<side>.yaml（POSE_SET 必須）。
# run_teleop_*.sh / record_*.sh / rollout がセット経由で align_on_connect に渡す。
# 既にあるファイルは上書きしない（学習済みモデルと組になっている姿勢を壊さないため・2026-09-30）。
# 新しい姿勢は新しいセット名で撮る。
#
# 開始姿勢の要件は較正フローで変わる(スクリプトが自動判別して案内する):
#   恒久ゼロ較正済み(scripts/calibrate_openarm.sh、homing offset 保存) → どの姿勢から始めてもよい
#   旧較正(offset 全ゼロ) → ⚠ 接続前に必ず腕を真下に垂らす(接続時に零位が焼かれるため)
# 使い方: POSE_SET=<新しいセット名> bash scripts/capture_initial_pose.sh <left|right> [gravity_scale]
set -euo pipefail

SIDE="${1:-right}"
GSCALE="${2:-1.0}"
case "$SIDE" in
  left)  PORT=can_lead_l ;;
  right) PORT=can_lead_r ;;
  *) echo "[capture] SIDE must be 'left' or 'right', got '$SIDE'" >&2; exit 1 ;;
esac

REPO="$(cd "$(dirname "$0")/.." && pwd)"
URDF="$REPO/urdf/openarm_v10_bimanual.urdf"
[ -n "${POSE_SET:-}" ] || { echo "[capture] POSE_SET=<新しいセット名> を指定すること（既存: $(ls "$REPO/config/pose_sets" | grep -v CURRENT | tr '\n' ' ')）" >&2; exit 1; }
OUT="$REPO/config/pose_sets/$POSE_SET/${SIDE}.yaml"
if [ -e "$OUT" ] && [ "${OVERWRITE:-false}" != true ]; then
  echo "[capture] 既にある: $OUT（学習済みモデルと組の姿勢なら壊さない。別のセット名で撮るか、未使用と分かっていれば OVERWRITE=true）" >&2
  exit 1
fi
mkdir -p "$(dirname "$OUT")"

# インタプリタ解決: teleop venv → conda lerobot env → python3 の順で最初に在るもの。
PY=""
for CAND in \
  "$HOME/projects/lerobot/.venv/bin/python" \
  "$HOME/miniconda3/envs/lerobot/bin/python" \
  "$(command -v python3 || true)"; do
  if [ -n "$CAND" ] && [ -x "$CAND" ]; then PY="$CAND"; break; fi
done
[ -n "$PY" ] || { echo "[capture] no python interpreter found" >&2; exit 1; }

echo "[capture] python=$PY  side=$SIDE  port=$PORT  out=$OUT  gravity_scale=$GSCALE"
"$PY" "$REPO/scripts/capture_initial_pose.py" \
  --side "$SIDE" --port "$PORT" --id "leader_${SIDE}" \
  --gravity-urdf "$URDF" --gravity-side "$SIDE" --gravity-scale "$GSCALE" \
  --out "$OUT"
