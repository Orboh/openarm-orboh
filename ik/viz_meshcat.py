#!/usr/bin/env python3
"""OpenArm IK 解の3D確認（Meshcat・pinocchio・実物メッシュ）.

実機に送る前に IK 解の姿勢をブラウザの3Dビューアで確認する。運動学のみ（物理なし）。
メッシュは別クローン `~/projects/OpenArm/src/openarm_description`（.dae visual を含む）を
package_dirs で解決する。Vector（ディスプレイ/ブラウザがある開発機）で実行する。

実行:
  ~/miniconda3/envs/lerobot/bin/python ik/viz_meshcat.py                 # 到達可能解 [0.14,0.15,0.31] を表示
  ~/miniconda3/envs/lerobot/bin/python ik/viz_meshcat.py --xyz 0.2 0.15 0.3
  ~/miniconda3/envs/lerobot/bin/python ik/viz_meshcat.py --diverged      # 今日の発散解を再現表示
表示後、出力された Meshcat URL をブラウザで開く。Ctrl-C で終了。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from openarm_left_ik import DEFAULT_URDF, OpenArmLeftIK  # noqa: E402

MESH_ROOT = os.environ.get("OPENARM_MESH_ROOT", "/home/orboh/projects/OpenArm/src")


def build_full():
    full = pin.buildModelFromUrdf(DEFAULT_URDF)
    vis = pin.buildGeomFromUrdf(full, DEFAULT_URDF, pin.GeometryType.VISUAL, package_dirs=[MESH_ROOT])
    col = pin.buildGeomFromUrdf(full, DEFAULT_URDF, pin.GeometryType.COLLISION, package_dirs=[MESH_ROOT])
    return full, col, vis


def left_full_q(full, q_left):
    """左腕7値を full モデルの q に埋める（他関節は neutral）。"""
    q = pin.neutral(full)
    for k in range(1, 8):
        jid = full.getJointId(f"openarm_left_joint{k}")
        q[full.joints[jid].idx_q] = float(q_left[k - 1])
    return q


def reach_traj(ik, target, q0, max_rel_deg=3.0, max_steps=400, tol=0.004):
    """coarse 接近エピソードの軌道（垂下→目標）。各ステップ max_rel 度でクランプし IK を解き直す
    ＝実機 reach ループと同じ進み方。到達可能なら目標へ、不可能なら関節限界へ走る様子がそのまま軌道になる。"""
    q = np.asarray(q0, dtype=float).copy()
    cap = np.deg2rad(max_rel_deg)
    tgt = np.asarray(target, dtype=float)
    traj = [q.copy()]
    for _ in range(max_steps):
        q_goal, _, _, _ = ik.solve_position(tgt, q)
        q = q + np.clip(q_goal - q, -cap, cap)
        traj.append(q.copy())
        if np.linalg.norm(ik.fk(q).translation - tgt) < tol:
            break
    return traj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xyz", type=float, nargs=3, default=[0.14, 0.15, 0.31])
    ap.add_argument("--diverged", action="store_true", help="今日の発散解(全関節限界)を表示")
    ap.add_argument("--check", action="store_true", help="load+display して即終了（読み込み検証用）")
    args = ap.parse_args()

    full, col, vis = build_full()
    viz = MeshcatVisualizer(full, col, vis)
    viz.initViewer(open=False)
    viz.loadViewerModel(rootNodeName="openarm")
    print("Meshcat URL:", viz.viewer.url())

    ik = OpenArmLeftIK()
    q0 = np.zeros(7)
    target = [0.10, 0.15, 0.20] if args.diverged else list(args.xyz)
    traj_left = reach_traj(ik, target, q0)
    end_err = float(np.linalg.norm(ik.fk(traj_left[-1]).translation - np.array(target)))
    ok = end_err < 0.02
    print(f"target={target}  steps={len(traj_left)}  end_err={end_err * 1000:.0f}mm  reached={ok}")

    # 目標点を球で表示（到達=緑 / 未到達=赤）
    import meshcat.geometry as mg
    import meshcat.transformations as mtf
    viz.viewer["target"].set_object(mg.Sphere(0.02),
                                    mg.MeshLambertMaterial(color=(0x33aa33 if ok else 0xff3333)))
    viz.viewer["target"].set_transform(mtf.translation_matrix(target))

    traj_full = [left_full_q(full, q) for q in traj_left]
    if args.check:
        viz.display(traj_full[-1])
        print("check OK: model+meshes loaded, episode computed & posed.")
        return

    print(f"episode を再生（ループ・{len(traj_full)}フレーム・はじめ→終わり）。ブラウザで上記 URL を開く。Ctrl-C で終了。")
    try:
        while True:
            viz.display(traj_full[0])
            time.sleep(0.5)
            viz.play(traj_full, 0.04)
            time.sleep(0.8)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
