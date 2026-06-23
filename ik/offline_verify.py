#!/usr/bin/env python3
"""OpenArm 左腕 IK オフライン精度検証.
bimanual URDF を pinocchio で読み、右腕+指をロックして左腕7-DOFの reduced model を作る。
FK で作った到達可能な目標に対し damped-LS (CLIK) IK が解を復元できるか・精度を測る。
実機不要。conda env `lerobot` の pinocchio 4.0 で実行する。
"""
import numpy as np
from numpy.linalg import norm, solve
import pinocchio as pin

URDF = "/home/orboh/projects/openarm-orboh/urdf/openarm_v10_bimanual.urdf"
print("pinocchio", pin.__version__)

# 1. full model
model = pin.buildModelFromUrdf(URDF)
print("full model: njoints=%d nq=%d nv=%d" % (model.njoints, model.nq, model.nv))

# 2. reduced model = 左腕 joint1..7 のみ残す（右腕・指・その他を q=neutral でロック）
keep = ["openarm_left_joint%d" % i for i in range(1, 8)]
lock_ids = [j for j in range(1, model.njoints) if model.names[j] not in keep]
q_ref = pin.neutral(model)
rmodel = pin.buildReducedModel(model, lock_ids, q_ref)
rdata = rmodel.createData()
print("reduced: nq=%d  joints=%s" % (rmodel.nq, [rmodel.names[j] for j in range(1, rmodel.njoints)]))

EE = "openarm_left_joint7"
ee_id = rmodel.getJointId(EE)
print("EE joint id=%d (%s)" % (ee_id, EE))

lo = np.where(np.isfinite(rmodel.lowerPositionLimit), rmodel.lowerPositionLimit, -np.pi)
hi = np.where(np.isfinite(rmodel.upperPositionLimit), rmodel.upperPositionLimit, np.pi)
print("joint limits lo=%s" % np.round(np.rad2deg(lo), 1))
print("joint limits hi=%s" % np.round(np.rad2deg(hi), 1))


def fk(q):
    pin.forwardKinematics(rmodel, rdata, q)
    return rdata.oMi[ee_id].copy()


def clik(target, q0, max_iter=1000, eps=1e-4, DT=1.0, damp=1e-6):
    q = q0.copy()
    err = np.zeros(6)
    for i in range(max_iter):
        pin.forwardKinematics(rmodel, rdata, q)
        iMd = rdata.oMi[ee_id].actInv(target)
        err = pin.log(iMd).vector
        if norm(err) < eps:
            return q, True, i, err
        J = pin.computeJointJacobian(rmodel, rdata, q, ee_id)
        J = -np.dot(pin.Jlog6(iMd.inverse()), J)
        v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
        q = pin.integrate(rmodel, q, v * DT)
    return q, False, max_iter, err


# 3. 6-DOF 往復: ランダムな到達可能 pose を FK で作り、q=0 から IK 復元
rng = np.random.default_rng(0)
N = 30
pe, re, its, fails = [], [], [], 0
for _ in range(N):
    q_true = lo + (hi - lo) * rng.random(rmodel.nq)
    target = fk(q_true)
    q_sol, ok, it, _ = clik(target, pin.neutral(rmodel))
    e = pin.log(target.actInv(fk(q_sol))).vector
    pe.append(norm(e[:3]) * 1000)        # mm
    re.append(np.rad2deg(norm(e[3:])))   # deg
    its.append(it)
    fails += (not ok)
print("\n[6-DOF 往復 N=%d] fails=%d" % (N, fails))
print("  pos err mm : median=%.4f  max=%.4f" % (np.median(pe), np.max(pe)))
print("  rot err deg: median=%.4f  max=%.4f" % (np.median(re), np.max(re)))
print("  iters      : median=%d  max=%d" % (int(np.median(its)), int(np.max(its))))

# 4. 位置のみ（回転は seed の EE 姿勢に固定）= 6-DOFソルバで位置だけ狙う運用の検証
pe2, fails2 = [], 0
for _ in range(N):
    q_true = lo + (hi - lo) * rng.random(rmodel.nq)
    p_target = fk(q_true).translation
    q_seed = pin.neutral(rmodel)
    target = pin.SE3(fk(q_seed).rotation, p_target)
    q_sol, ok, it, _ = clik(target, q_seed)
    pe2.append(norm(target.translation - fk(q_sol).translation) * 1000)
    fails2 += (not ok)
print("\n[位置のみ(回転=seed固定) N=%d] fails=%d (姿勢拘束が無理な位置は収束しない=想定内)" % (N, fails2))
print("  pos err mm : median=%.4f  max=%.4f" % (np.median(pe2), np.max(pe2)))
