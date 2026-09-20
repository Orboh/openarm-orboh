#!/usr/bin/env python3
"""OpenArm 左腕 IK（lerobot 駆動用の再利用モジュール）.

bimanual URDF を pinocchio で読み、右腕+指をロックした左腕7-DOF reduced model で、
TCP フレーム（グリッパ先端）をカルテシアン目標へ持っていく damped-LS CLIK。
- position_only=True で「位置だけ合わせる（姿勢は自由）」モード。粗接近はこれで十分。
- lerobot は度(joint_1..7)・pinocchio はラジアン(openarm_left_joint1..7)。橋渡しヘルパを同梱。

注意（実機で要検証）: lerobot の較正零点（腕を垂らしグリッパ閉）と URDF 零点が一致し、
各関節の符号も一致している前提でヘルパが index 対応で deg<->rad する。実機で
「既知姿勢の q を読み FK 照合」＋「微小ジョグで向き確認」をしてから駆動に使うこと。
"""
from __future__ import annotations

import os

import numpy as np
from numpy.linalg import norm, solve
import pinocchio as pin

# repo 相対。標準配置は全マシン ~/projects/<repo>(scripts/bootstrap.sh の管轄)。
# 旧 Thor 配置(~/openarm-orboh・projects 無し)は ~/projects への移設待ちの例外で、
# 相対解決なのでどちらでも動くが、正は前者。
DEFAULT_URDF = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "urdf", "openarm_v10_bimanual.urdf")
)
LEFT_ARM_JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)]  # pinocchio (rad)
LEROBOT_JOINTS = [f"joint_{i}" for i in range(1, 8)]               # lerobot (deg)
TCP_FRAME = "openarm_left_hand_tcp"


class OpenArmLeftIK:
    def __init__(self, urdf_path: str = DEFAULT_URDF, ee_frame: str = TCP_FRAME):
        full = pin.buildModelFromUrdf(urdf_path)
        lock = [j for j in range(1, full.njoints) if full.names[j] not in LEFT_ARM_JOINTS]
        self.model = pin.buildReducedModel(full, lock, pin.neutral(full))
        self.data = self.model.createData()
        # EE: TCP フレームがあれば使う。無ければ joint7 にフォールバック。
        if self.model.existFrame(ee_frame):
            self.ee_fid = self.model.getFrameId(ee_frame)
            self.use_frame = True
            self.ee_name = ee_frame
        else:
            self.ee_jid = self.model.getJointId("openarm_left_joint7")
            self.use_frame = False
            self.ee_name = "openarm_left_joint7(fallback)"
        self.lo = np.where(np.isfinite(self.model.lowerPositionLimit),
                           self.model.lowerPositionLimit, -np.pi)
        self.hi = np.where(np.isfinite(self.model.upperPositionLimit),
                           self.model.upperPositionLimit, np.pi)

    def fk(self, q) -> pin.SE3:
        q = np.asarray(q, dtype=float)
        pin.forwardKinematics(self.model, self.data, q)
        if self.use_frame:
            pin.updateFramePlacements(self.model, self.data)
            return self.data.oMf[self.ee_fid].copy()
        return self.data.oMi[self.ee_jid].copy()

    def _jac(self, q):
        if self.use_frame:
            return pin.computeFrameJacobian(self.model, self.data, q, self.ee_fid)
        return pin.computeJointJacobian(self.model, self.data, q, self.ee_jid)

    def _jac_world_pos(self, q):
        """EE フレームの位置ヤコビアン(3xnv, world-aligned)。位置サーボ用。"""
        if self.use_frame:
            J = pin.computeFrameJacobian(self.model, self.data, q, self.ee_fid,
                                         pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        else:
            J = pin.computeJointJacobian(self.model, self.data, q, self.ee_jid)
        return J[:3, :]

    def solve(self, target: pin.SE3, q_seed, position_only: bool = True,
              max_iter: int = 300, eps: float = 1e-4, damp: float = 1e-4, dt: float = 1.0):
        """target(SE3) へ EE を持っていく q を返す。(q, converged, iters, final_err6)。
        position_only=True は「位置だけ合わせ姿勢は自由」（粗接近向け・到達しやすい・
        world 位置ヤコビアンで位置サーボ）。False は 6-DOF フルポーズ。"""
        q = np.asarray(q_seed, dtype=float).copy()
        p_tgt = np.asarray(target.translation, dtype=float)
        err6 = np.zeros(6)
        for i in range(max_iter):
            M = self.fk(q)
            if position_only:
                e = M.translation - p_tgt              # world 位置誤差(3,)
                err6[:3] = e
                if norm(e) < eps:
                    return q, True, i, err6
                Jp = self._jac_world_pos(q)            # 3xnv
                v = -Jp.T @ solve(Jp @ Jp.T + damp * np.eye(3), e)
            else:
                iMd = M.actInv(target)
                err6 = pin.log(iMd).vector
                if norm(err6) < eps:
                    return q, True, i, err6
                J = -pin.Jlog6(iMd.inverse()) @ self._jac(q)
                v = -J.T @ solve(J @ J.T + damp * np.eye(6), err6)
            q = pin.integrate(self.model, q, v * dt)
        return q, False, max_iter, err6

    def solve_position(self, xyz, q_seed, **kw):
        """3D 位置(腕ベース系)だけを目標に解く（姿勢自由）。"""
        target = pin.SE3(np.eye(3), np.asarray(xyz, dtype=float))
        return self.solve(target, q_seed, position_only=True, **kw)

    def clamp(self, q):
        return np.clip(np.asarray(q, float), self.lo, self.hi)

    def within_limits(self, q) -> bool:
        q = np.asarray(q, float)
        return bool(np.all(q >= self.lo - 1e-6) and np.all(q <= self.hi + 1e-6))


# ---- lerobot 橋渡し（度 <-> ラジアン・joint_k <-> 配列 index）-------------------
def lerobot_obs_to_q_rad(obs: dict) -> np.ndarray:
    return np.deg2rad([obs[f"{j}.pos"] for j in LEROBOT_JOINTS])


def q_rad_to_lerobot_action(q_rad) -> dict:
    deg = np.rad2deg(np.asarray(q_rad, float))
    return {f"{LEROBOT_JOINTS[i]}.pos": float(deg[i]) for i in range(7)}


def max_joint_delta_deg(q1_rad, q2_rad) -> float:
    return float(np.max(np.abs(np.rad2deg(np.asarray(q1_rad, float) - np.asarray(q2_rad, float)))))


if __name__ == "__main__":
    ik = OpenArmLeftIK()
    print("pinocchio", pin.__version__, "| reduced nq=", ik.model.nq, "| EE=", ik.ee_name)
    rng = np.random.default_rng(0)
    N = 30
    pe6, fails6 = [], 0
    pep, failsp = [], 0
    for _ in range(N):
        q_true = ik.lo + (ik.hi - ik.lo) * rng.random(ik.model.nq)
        tgt = ik.fk(q_true)
        # 6-DOF
        q, ok, _, _ = ik.solve(tgt, pin.neutral(ik.model), position_only=False)
        pe6.append(norm(ik.fk(q).translation - tgt.translation) * 1000); fails6 += (not ok)
        # 位置のみ
        q, ok, _, _ = ik.solve_position(tgt.translation, pin.neutral(ik.model))
        pep.append(norm(ik.fk(q).translation - tgt.translation) * 1000); failsp += (not ok)
    print(f"[6-DOF]    fails={fails6}/{N}  pos err mm median={np.median(pe6):.4f} max={np.max(pe6):.4f}")
    print(f"[pos-only] fails={failsp}/{N}  pos err mm median={np.median(pep):.4f} max={np.max(pep):.4f}")
