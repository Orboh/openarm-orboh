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
    if args.diverged:
        q_left = np.deg2rad([-200, 10, 90, 140, 90, -45, 90])
        tgt, ok = [0.10, 0.15, 0.20], False
    else:
        q_left, ok, _, _ = ik.solve_position(args.xyz, np.zeros(7))
        tgt = args.xyz
    print(f"target={tgt}  converged={ok}")

    # 目標点を赤い球で表示
    import meshcat.geometry as mg
    import meshcat.transformations as mtf
    color = 0xff3333 if (np.linalg.norm(np.array(tgt) - ik.fk(np.zeros(7)).translation) > 0.6) else 0x33aa33
    viz.viewer["target"].set_object(mg.Sphere(0.02), mg.MeshLambertMaterial(color=color))
    viz.viewer["target"].set_transform(mtf.translation_matrix(tgt))

    if args.check:
        viz.display(left_full_q(full, q_left))
        print("check OK: model+meshes loaded, posed.")
        return

    # 垂下 -> 解 へアニメーション
    for a in np.linspace(0.0, 1.0, 40):
        viz.display(left_full_q(full, a * q_left))
        time.sleep(0.03)
    print("posed. ブラウザで上記 URL を開く。Ctrl-C で終了（ビューアは生存）。")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
