#!/usr/bin/env bash
# openarm_description の xacro から teleop 用 URDF を生成（colcon ws 無しで解決）。リポ相対・sudo不要。
# 既に urdf/openarm_v10_bimanual.urdf がコミット同梱されていれば通常は不要（再生成したい時に使う）。
set -uo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
DESC="$REPO/openarm_description"
XACRO="$DESC/assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro"
OUTDIR="$REPO/urdf"
PREFIX=/tmp/oa_ament

[ -f "$XACRO" ] || { echo "[ERROR] xacro 無し: $XACRO（git submodule update --init --recursive）"; exit 1; }

# 一時 ament prefix（openarm_description をパッケージとして見せる）
rm -rf "$PREFIX"
mkdir -p "$PREFIX/share/ament_index/resource_index/packages"
ln -s "$DESC" "$PREFIX/share/openarm_description"
: > "$PREFIX/share/ament_index/resource_index/packages/openarm_description"

set +u; for _d in jazzy humble; do [ -f "/opt/ros/$_d/setup.bash" ] && { source "/opt/ros/$_d/setup.bash"; break; }; done; set -u
export AMENT_PREFIX_PATH="$PREFIX:${AMENT_PREFIX_PATH:-}"

mkdir -p "$OUTDIR"
echo "=== xacro 生成 ==="
xacro "$XACRO" bimanual:=true -o "$OUTDIR/openarm_v10_bimanual.urdf"

# teleop バイナリが要求する leaf link（openarm_{left,right}_hand）を link7 に fixed で追加
python3 - "$OUTDIR/openarm_v10_bimanual.urdf" <<'PY'
import sys
p=sys.argv[1]; s=open(p).read(); add=""
for side in ("left","right"):
    hand=f"openarm_{side}_hand"; link7=f"openarm_{side}_link7"
    if f'"{hand}"' not in s and f'"{link7}"' in s:
        add += (f'  <link name="{hand}"/>\n'
                f'  <joint name="{hand}_joint" type="fixed">\n'
                f'    <origin rpy="0 0 0" xyz="0 0 0"/>\n'
                f'    <parent link="{link7}"/>\n    <child link="{hand}"/>\n  </joint>\n')
if add:
    s=s.replace("</robot>", add+"</robot>"); open(p,"w").write(s); print("added leaf links")
else:
    print("no change (links already present)")
PY

echo "=== 完了: $OUTDIR/openarm_v10_bimanual.urdf ==="
grep -c "<link "  "$OUTDIR/openarm_v10_bimanual.urdf" | sed 's/^/  links:  /'
grep -c "<joint " "$OUTDIR/openarm_v10_bimanual.urdf" | sed 's/^/  joints: /'
