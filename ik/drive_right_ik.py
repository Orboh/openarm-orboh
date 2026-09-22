#!/usr/bin/env python3
"""OpenArm 右腕 実機 IK 駆動 ＋ 零点/符号検証（lerobot 経由・実機=Thor で実行）.

左腕版 drive_left_ik.py の右腕コピー（把持開発を右腕に寄せたため・2026-06-23）。
port=can_fol_r / side=right / id=follower_right に差し替え。**右ペアは未較正**なので
connect 時に calibrate に入りうる（先に lerobot calibrate で follower_right.json を作る）。
右系統 CAN は暫定復旧（再嵌合のみ・再 bus-off しうる）→ 起動前に scripts/can_check.sh で右ch健全性を確認。

⚠️ 実機を動かす。e-stop を手元に。**ハード未検証のコード**。必ず順番を守る:
  1) `verify` で「lerobot 較正零点と URDF 零点・各関節の符号」が一致するか確認
     （read-only。FK 位置と実物の手先位置を目視照合。--jog で1関節だけ微小に動かし向き確認）
  2) それが通って初めて `reach`（位置のみ IK で粗接近）。最初は小さい --max-rel と dry-run（--go 無し）で。

前提（Thor・lerobot venv）:
  sudo bash ~/openarm-orboh/scripts/canup.sh        # can_fol_r を up
  bash ~/openarm-orboh/scripts/can_check.sh         # 右ch PASS を確認（暫定復旧のため）
  # 較正済み follower_right.json があること（無いと connect 時に calibrate に入る）

実行例:
  python ik/drive_right_ik.py verify
  python ik/drive_right_ik.py verify --jog joint_1 --deg 5 --go     # 1関節を+5°（向き確認）
  python ik/drive_right_ik.py reach --xyz 0.30 -0.10 0.20           # dry-run（送らない）
  python ik/drive_right_ik.py reach --xyz 0.30 -0.10 0.20 --go --max-rel 3
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from openarm_right_ik import (  # noqa: E402
    LEROBOT_JOINTS,
    OpenArmRightIK,
    lerobot_obs_to_q_rad,
    q_rad_to_lerobot_action,
)


def make_robot(port: str, side: str, rid: str, max_rel: float):
    from lerobot.robots.openarm_follower.config_openarm_follower import OpenArmFollowerConfig
    from lerobot.robots.openarm_follower.openarm_follower import OpenArmFollower

    cfg = OpenArmFollowerConfig(
        port=port, side=side, id=rid, max_relative_target=max_rel, cameras={}
    )
    return OpenArmFollower(cfg)


def read_q_rad(robot):
    obs = robot.get_observation()
    return lerobot_obs_to_q_rad(obs), obs


def cmd_verify(robot, ik: OpenArmRightIK, args) -> None:
    robot.connect()
    try:
        q, obs = read_q_rad(robot)
        M = ik.fk(q)
        print("== 現在 q (deg) ==")
        for j in LEROBOT_JOINTS:
            print(f"   {j}: {obs[f'{j}.pos']:+8.2f}")
        print(f"== FK EE ({ik.ee_name}) 位置[m] == {np.round(M.translation, 4).tolist()}")
        print("== FK EE 回転(行列) ==")
        print(np.round(M.rotation, 3))
        print(">> 腕の実際の手先位置と FK 位置が一致するか目視確認。"
              "不一致なら lerobot 較正零点と URDF 零点（or 関節符号）がズレている。")
        if args.jog:
            jdeg = {f"{j}.pos": float(np.rad2deg(q[i])) for i, j in enumerate(LEROBOT_JOINTS)}
            jdeg[f"{args.jog}.pos"] += args.deg
            if not args.go:
                print(f"[dry-run] {args.jog} を {args.deg:+.1f}° 動かす予定（--go で実行）")
                return
            print(f">> {args.jog} を {args.deg:+.1f}° 動かす。動く向きを確認（Ctrl-C で停止）")
            for _ in range(40):  # max_relative_target でクランプされながら ~2s かけて到達
                robot.send_action(jdeg)
                time.sleep(0.05)
    finally:
        robot.disconnect()
        print("（disconnect: torque off。腕が脱力するので支えること）")


def cmd_reach(robot, ik: OpenArmRightIK, args) -> None:
    target = np.asarray(args.xyz, dtype=float)
    robot.connect()
    try:
        q, _ = read_q_rad(robot)
        print(f"目標位置[m]={target.tolist()}  開始 EE={np.round(ik.fk(q).translation, 4).tolist()}")
        for step in range(args.steps):
            q, _ = read_q_rad(robot)                         # 現在角を seed（閉ループ）
            q_goal, ok, _, _ = ik.solve_position(target, q)
            if not ik.within_limits(q_goal):
                print("[warn] IK 解が可動域外 → クランプ")
                q_goal = ik.clamp(q_goal)
            ee_pred = ik.fk(q_goal).translation
            pred_err_mm = float(np.linalg.norm(ee_pred - target) * 1000)
            action = q_rad_to_lerobot_action(q_goal)
            if not args.go:
                print(f"[dry-run] 解 converged={ok} 予測到達誤差={pred_err_mm:.1f}mm")
                print(f"[dry-run] action(deg)={ {k: round(v, 1) for k, v in action.items()} }")
                break
            robot.send_action(action)                        # max_relative_target で各ステップclamp
            cur = ik.fk(read_q_rad(robot)[0]).translation
            d_mm = float(np.linalg.norm(cur - target) * 1000)
            print(f"step{step}: EE={np.round(cur, 4).tolist()}  目標まで={d_mm:.1f}mm")
            if d_mm < args.tol_mm:
                print("到達"); break
            time.sleep(1.0 / args.hz)
    finally:
        robot.disconnect()
        print("（disconnect: torque off。腕が脱力するので支えること）")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="mode", required=True)

    pv = sub.add_parser("verify", help="read-only FK 照合（+任意で1関節の微小ジョグ）")
    pv.add_argument("--jog", choices=LEROBOT_JOINTS, help="向き確認で動かす関節")
    pv.add_argument("--deg", type=float, default=5.0, help="ジョグ角度[deg]")

    pr = sub.add_parser("reach", help="位置のみ IK でカルテシアン目標へ粗接近")
    pr.add_argument("--xyz", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"),
                    help="目標位置[m]（腕ベース openarm_body_link0 系）")
    pr.add_argument("--steps", type=int, default=200)
    pr.add_argument("--tol-mm", type=float, default=10.0)
    pr.add_argument("--hz", type=float, default=10.0)

    for p in (pv, pr):
        p.add_argument("--port", default="can_fol_r")
        p.add_argument("--side", default="right")
        p.add_argument("--id", default="follower_right")
        p.add_argument("--max-rel", type=float, default=3.0, help="1ステップの最大関節変化[deg]（安全弁）")
        p.add_argument("--go", action="store_true", help="実際に send_action する（既定は dry-run）")

    args = ap.parse_args()
    ik = OpenArmRightIK()
    robot = make_robot(args.port, args.side, args.id, args.max_rel)
    if args.mode == "verify":
        cmd_verify(robot, ik, args)
    else:
        cmd_reach(robot, ik, args)


if __name__ == "__main__":
    main()
