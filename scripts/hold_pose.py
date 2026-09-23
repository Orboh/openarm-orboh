#!/usr/bin/env python
"""片腕を指定の姿勢で保持し続ける（teleop なし・leader なし）。

なぜ要るか（2026-09-23 板橋）: v2 の収集では右腕を動かさず、右 wrist D405 で
置き場所を映し続けるカメラ役にする。その保持を人が leader を持って行うか制御で
行うかのうち、**制御で固定する**ことになった（Kento 決定）。teleop を Ctrl-C で
止めると disable_torque_on_disconnect で脱力して腕が落ちるため、保持し続ける
プロセスが別に要る。

やること:
  1. follower に接続する。align_on_connect が開始姿勢 YAML の waypoints を順に
     たどって目的の姿勢まで持っていく（経路は YAML 側の責任）。
  2. その姿勢を目標に位置指令を送り続ける。
  3. Ctrl-C で **経由地を逆順にたどって退避**してから切断する。いきなり脱力
     させると保持していた高さから机へ落ちるため。

グリッパは既定で位置 0（較正ゼロ＝閉）を kp=kd=0 で送る。電流を流さないので、
半開き保持で立つコイル過温ラッチ(0xC)を避けられる。起動時に非ゼロの把持指令を
出して follower gripper を壊した事故への対処とも一致する。
--gripper で開いた位置を保持できる（-65 が全開・0 が閉）。**開いたまま保持すると
過温ラッチの条件そのもの**なので、剛性は収集で使っている 12 を既定にし、ラッチが
立ったら scripts/damiao_probe_state.py で読んで damiao_errcheck.py --clear-motor で
解除する。

使い方（Thor・~/projects/openarm-orboh）:
  python scripts/hold_pose.py --port can_fol_r --side right --id follower_right \
      --pose config/initial_pose_right_rightcam.yaml
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
logger = logging.getLogger("hold-pose")

ARM_JOINTS = [f"joint_{i}" for i in range(1, 8)]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="CAN インターフェース（例 can_fol_r）")
    ap.add_argument("--side", required=True, choices=["left", "right"])
    ap.add_argument("--id", required=True, help="較正ファイルの id（例 follower_right）")
    ap.add_argument("--pose", required=True, help="開始姿勢 YAML（waypoints を含んでよい）")
    ap.add_argument("--fps", type=float, default=50.0, help="保持指令の送信レート")
    ap.add_argument("--report-s", type=float, default=5.0, help="保持誤差を出す間隔（0=出さない）")
    ap.add_argument("--gripper", type=float, default=0.0, help="グリッパの保持位置（-65=全開・0=閉）")
    ap.add_argument(
        "--gripper-kp",
        type=float,
        default=None,
        help="グリッパの剛性。既定は閉なら 0（無通電）、開くなら 12（収集で使う値）",
    )
    ap.add_argument("--retreat-duration-s", type=float, default=2.2, help="退避 1 区間あたりの秒数")
    args = ap.parse_args()

    from lerobot.motors.damiao.damiao_alignment import (
        resolve_initial_pose_sequence,
        soft_move_to_position,
    )
    from lerobot.robots.openarm_follower import OpenArmFollower, OpenArmFollowerConfig

    cfg = OpenArmFollowerConfig(
        port=args.port, side=args.side, id=args.id, initial_pose_path=args.pose
    )
    robot = OpenArmFollower(cfg)

    # connect() が waypoints を順にたどって目的の姿勢まで動かす。
    robot.connect()

    sequence = resolve_initial_pose_sequence(
        initial_pose_path=args.pose, joint_limits=cfg.joint_limits
    )
    target = sequence[-1]
    logger.info(
        "保持開始: " + "  ".join(f"{j}={target[j]:.1f}" for j in ARM_JOINTS) + f"  ({args.fps:.0f} Hz)"
    )
    logger.info("Ctrl-C で経由地を逆順に退避してから切断する")

    action = {f"{j}.pos": float(target[j]) for j in ARM_JOINTS}
    action["gripper.pos"] = float(args.gripper)
    # 閉（0）のままなら剛性ゼロ＝電流を流さない（過温ラッチ回避）。開くなら保持に
    # 剛性が要るので、収集で使っている 12 を既定にする。
    gkp = args.gripper_kp if args.gripper_kp is not None else (0.0 if args.gripper == 0.0 else 12.0)
    custom_kp = {"gripper": gkp}
    custom_kd = {"gripper": 0.0 if gkp == 0.0 else 0.2}
    logger.info("グリッパ: 位置 %.1f  kp %.1f%s" % (
        args.gripper, gkp, "（無通電）" if gkp == 0.0 else "（開いたまま保持＝過温ラッチに注意）"))

    stopping = False

    def on_sigint(signum, frame):  # noqa: ARG001
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, on_sigint)

    period = 1.0 / args.fps
    next_report = time.perf_counter() + args.report_s
    try:
        while not stopping:
            t0 = time.perf_counter()
            robot.send_action(action, custom_kp=custom_kp, custom_kd=custom_kd)
            if args.report_s > 0 and t0 >= next_report:
                obs = robot.get_observation()
                err = {
                    j: obs.get(f"{j}.pos", float("nan")) - target[j]
                    for j in ARM_JOINTS
                }
                worst = max(err, key=lambda j: abs(err[j]))
                logger.info(
                    "保持中 最大ずれ %s %+.2f deg  (" % (worst, err[worst])
                    + " ".join(f"{j[-1]}:{err[j]:+.1f}" for j in ARM_JOINTS)
                    + ")"
                )
                next_report = t0 + args.report_s
            sleep = period - (time.perf_counter() - t0)
            if sleep > 0:
                time.sleep(sleep)
    finally:
        # 逆順に退避してから切る。最後の要素は現在地なので飛ばす。
        logger.info("退避する（経由地を逆順）")
        for goal in list(reversed(sequence))[1:]:
            soft_move_to_position(robot.bus, goal, args.retreat_duration_s)
        robot.disconnect()
        logger.info("切断した")
    return 0


if __name__ == "__main__":
    sys.exit(main())
