#!/usr/bin/env bash
# openarm-orboh ワンストップ・セットアップ（git clone 後にこれ1本）。実機ポートは問わない（channel_id 方式）。
#   1. submodule 取得 → 2. 依存インストール[sudo] → 3. teleop ビルド → 4. URDF（無ければ生成）→ 5. udev ルール導入[sudo]
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
cd "$REPO"
log(){ printf '\n\033[1;36m=== %s ===\033[0m\n' "$*"; }

log "1/5 submodule 取得（openarm_teleop / openarm_description）"
git submodule update --init --recursive

log "2/5 依存インストール（sudo・apt）"
bash "$REPO/scripts/install_deps.sh"

log "3/5 teleop ビルド"
bash "$REPO/scripts/build_teleop.sh"

log "4/5 URDF 準備"
if [ -f "$REPO/urdf/openarm_v10_bimanual.urdf" ]; then echo "  同梱 URDF を使用"; else bash "$REPO/scripts/gen_urdf.sh"; fi

log "5/5 CAN 固定名 udev ルール導入（sudo）"
bash "$REPO/scripts/install_can_rules.sh"

cat <<EOF

=== セットアップ完了 ===
次の手順:
  1. PEAK アダプタ2台を直挿し（どのUSBポートでもOK・ハブNG）。
     役割未設定の新品アダプタだけ役割タグを焼く（既存機は不要）:
       sudo bash scripts/burn_channel_id.sh <iface> leader      # or follower
  2. CAN 固定名を付与/確認:
       sudo bash scripts/canup.sh         # -> can_lead_r/l, can_fol_r/l が UP
  3. テレオペ:
       右腕: bash scripts/launch_unilateral.sh right_arm can_lead_r can_fol_r
       両腕: bash scripts/launch_bimanual.sh    （停止: Ctrl-C で全bus disable）
EOF
