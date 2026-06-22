#!/usr/bin/env python3
"""OpenArm 左腕の運動学スケルトン可視化（メッシュ不要・pinocchio FK + matplotlib 3D）.

実機に送る前に IK 解の妥当性を目で確認する。__main__ は今日の「発散」を再現表示:
左=発散解（目標が腕の届く範囲外で関節が限界に張り付く） / 右=到達可能解。
出力は PNG（headless・Agg）。Vector の lerobot env(pinocchio+matplotlib) で実行。
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pinocchio as pin  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from openarm_left_ik import OpenArmLeftIK  # noqa: E402


def skeleton(ik, q):
    """base 原点 + joint1..7 + TCP の3D点列（スケルトン）。"""
    q = np.asarray(q, dtype=float)
    pin.forwardKinematics(ik.model, ik.data, q)
    pin.updateFramePlacements(ik.model, ik.data)
    pts = [np.zeros(3)]
    for k in range(1, 8):
        pts.append(ik.data.oMi[ik.model.getJointId(f"openarm_left_joint{k}")].translation.copy())
    pts.append(ik.data.oMf[ik.ee_fid].translation.copy())
    return np.array(pts)


def _draw(ax, ik, q, color, label, tgt, tgt_label, title, sh, R, q0):
    # reach sphere (radius = arm length at full extension, centered at shoulder)
    u, v = np.mgrid[0:2 * np.pi:16j, 0:np.pi:9j]
    ax.plot_wireframe(sh[0] + R * np.cos(u) * np.sin(v),
                      sh[1] + R * np.sin(u) * np.sin(v),
                      sh[2] + R * np.cos(v), color="lightgray", linewidth=0.3, alpha=0.5)
    hang = skeleton(ik, q0)
    ax.plot(hang[:, 0], hang[:, 1], hang[:, 2], "-", color="gray", lw=1, alpha=0.4, label="hang (q0)")
    sk = skeleton(ik, q)
    ax.plot(sk[:, 0], sk[:, 1], sk[:, 2], "-o", color=color, lw=2.2, ms=4, label=label)
    ax.scatter(*sh, color="black", s=30)
    ax.text(sh[0], sh[1], sh[2], "  shoulder", fontsize=8)
    d = np.linalg.norm(np.asarray(tgt) - sh)
    ax.scatter(*tgt, color="crimson" if d > R else "green", s=90, marker="X")
    ax.text(tgt[0], tgt[1], tgt[2], f"  target {tgt_label}\n  {d * 100:.0f}cm from shoulder", fontsize=8)
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("x (fwd+)"); ax.set_ylabel("y (left+)"); ax.set_zlabel("z (up+)")
    ax.set_xlim(-0.35, 0.55); ax.set_ylim(-0.15, 0.55); ax.set_zlim(0.1, 0.85)
    ax.view_init(elev=15, azim=-65)
    ax.legend(fontsize=7, loc="upper left")


def main():
    ik = OpenArmLeftIK()
    q0 = np.zeros(7)
    sh = skeleton(ik, q0)[1]                                    # shoulder (joint_1)
    R = float(np.linalg.norm(skeleton(ik, q0)[-1] - sh))        # arm reach at q0 (~0.44m)

    q_div = np.deg2rad([-200, 10, 90, 140, 90, -45, 90])        # today's diverged solution (joints at limits)
    bad = [0.10, 0.15, 0.20]
    q_good, ok, _, _ = ik.solve_position([0.14, 0.15, 0.31], q0)
    good = [0.14, 0.15, 0.31]

    fig = plt.figure(figsize=(14, 6.2))
    _draw(fig.add_subplot(121, projection="3d"), ik, q_div, "red", "diverged (joints at limits)",
          bad, "[0.10,0.15,0.20]",
          f"DIVERGED: target {np.linalg.norm(np.array(bad)-sh)*100:.0f}cm > arm reach {R*100:.0f}cm\n"
          "-> unreachable -> joints run to limits", sh, R, q0)
    _draw(fig.add_subplot(122, projection="3d"), ik, q_good, "green", "IK solution (elbow bends)",
          good, "[0.14,0.15,0.31]",
          f"REACHABLE: target {np.linalg.norm(np.array(good)-sh)*100:.0f}cm < reach {R*100:.0f}cm\n"
          "-> elbow bends, hand reaches target", sh, R, q0)

    out = "/home/orboh/.claude/jobs/2f5a8ca7/tmp/ik_divergence.png"
    plt.tight_layout()
    plt.savefig(out, dpi=115)
    print("saved", out)
    print(f"arm reach (shoulder->hand at q0) = {R:.3f} m")
    print(f"shoulder->bad target  = {np.linalg.norm(np.array(bad)-sh):.3f} m  (OUT of reach)")
    print(f"shoulder->good target = {np.linalg.norm(np.array(good)-sh):.3f} m  (in reach)")
    print(f"good solution reach error = {np.linalg.norm(skeleton(ik,q_good)[-1]-np.array(good))*1000:.1f} mm (converged={ok})")


if __name__ == "__main__":
    main()
