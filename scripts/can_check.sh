#!/usr/bin/env bash
# OpenArm CAN 健全性チェック（admin up/down ではなく CAN protocol state を見る）
#
# なぜ要るか: `ip link set up` は CAN コントローラが BUS-OFF / ERROR-PASSIVE でも
# admin UP を返す。canup.sh の旧 up カウンタや `ip -br link` の "UP" 表示は
# 「通信できる」ことを意味しない（右系統が ERROR-PASSIVE で teleop が落ちた既往）。
# ここでは各 IF の `ip -details link show` の CAN state を読み、
#   ERROR-ACTIVE / ERROR-WARNING = 健全
#   ERROR-PASSIVE / BUS-OFF / STOPPED / SLEEPING / IF不在 = 異常（teleop 不可）
# 要求した全 IF が健全なら exit 0、ひとつでも異常なら exit 1。
#
# 使い方:
#   bash scripts/can_check.sh                          # 既定4本（bimanual）
#   bash scripts/can_check.sh can_lead_r can_fol_r     # 指定のみ（unilateral 等）
#
# 注意: Jetson Thor は onboard mttcan(can4-7) を持つが OpenArm とは無関係（常時 STOPPED）。
#       本チェックは固定名のみを対象にし、can0-7 は列挙しない。
# 単独デバッグにも使える: teleop 後に実行すれば、TX 負荷で BUS-OFF / ERROR-PASSIVE に
#       落ちた系統を「UP 表示の嘘」なしで暴ける。
set -uo pipefail

if [[ $# -ge 1 ]]; then
  REQ=("$@")
else
  REQ=(can_lead_r can_lead_l can_fol_r can_fol_l)
fi

# `ip -details link show` から CAN state を取り出す。
# 出力例: "    can <FD> state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 100"
#         "    can state STOPPED ..."（onboard mttcan は <FD> 無し）
# link 行の "state UP/DOWN" は CAN-state 語彙に無いのでマッチしない。
can_state() {
  ip -details link show "$1" 2>/dev/null \
    | grep -oE 'state (ERROR-ACTIVE|ERROR-WARNING|ERROR-PASSIVE|BUS-OFF|STOPPED|SLEEPING)' \
    | tail -n1 | awk '{print $2}'
}
berr() {
  ip -details link show "$1" 2>/dev/null \
    | grep -oE 'berr-counter tx [0-9]+ rx [0-9]+' | head -n1
}

bad=0
echo "=== CAN 健全性チェック（can state / 要求 ${#REQ[@]}ch）==="
for ifn in "${REQ[@]}"; do
  if [[ ! -e "/sys/class/net/$ifn" ]]; then
    echo "[NG] $ifn : IF 不在（canup.sh で固定名 UP したか確認）"
    bad=$((bad+1)); continue
  fi
  st=$(can_state "$ifn")
  bc=$(berr "$ifn")
  case "$st" in
    ERROR-ACTIVE|ERROR-WARNING)
      echo "[OK] $ifn : $st  ($bc)" ;;
    "")
      echo "[NG] $ifn : CAN state 取得不可（CAN type でない/未設定）"
      bad=$((bad+1)) ;;
    *)
      echo "[NG] $ifn : $st  ($bc) ＝通信不可（配線/コネクタ嵌合・canup.sh 再実行を確認）"
      bad=$((bad+1)) ;;
  esac
done

echo
if [[ $bad -eq 0 ]]; then
  echo "[PASS] 要求 ${#REQ[@]}ch すべて健全（admin UP ではなく CAN state で確認）"
  exit 0
else
  echo "[FAIL] $bad ch が異常 → teleop を開始しない"
  exit 1
fi
