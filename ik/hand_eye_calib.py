#!/usr/bin/env python3
"""hand-eye 較正（eye-to-hand・D435i chest 固定カメラ → 左腕ベース系）.

D435i(chest) は固定設置のオーバービューカメラ＝**eye-to-hand**。カメラ座標系で得た
3D点（detect_okra.py の出力）を腕ベース系 openarm_body_link0 (+x前/+y左/+z上) へ
移す固定剛体変換 T_base_cam (R,t) を、対応点群から Umeyama(剛体・スケール無し)で求める。
求めた変換は detect→hand_eye→IK reach の鎖で実オクラへ粗接近するために使う。

原理:
  グリッパ近傍に緑マーカを付け、腕を複数姿勢へ動かす。各姿勢で
    p_base = TCP位置 + R_ee·marker_offset   ← openarm_left_ik の FK（TCPの位置と回転）
    p_cam  = カメラで検出した緑マーカの3D（最大ブロブ）  ← detect_okra
  の対応ペアを集め、p_base ≈ R·p_cam + t を解く（≥4 非共面点が必要・多いほど良い）。

  **生の TCP 姿勢（位置 tcp_pos と回転 tcp_rot）と p_cam を保存する**ので、マーカオフセットを
  変えても再収集は不要：`solve --marker-offset` でオフラインで base を計算し直して再フィットできる。
  マーカが TCP から何cmずれているか不明な本ケースでは `solve --sweep-z` で進入軸(+Z)方向の
  オフセットを自動探索し、残差を最小化するオフセットを選べる。

⚠️ collect は実機を動かす（reach と同じ安全弁: dry-run 既定・--max-rel クランプ・e-stop 手元）。
   現在姿勢からの相対摂動なので動きは小さい。マーカが chest FOV に入る姿勢で開始すること。
   カメラを動かしたら較正やり直し（固定前提）。

実行（Thor・lerobot venv）:
  sudo bash ~/openarm-orboh/scripts/canup.sh        # can_fol_l を up
  # 1) 緑マーカをグリッパ近傍に付け、teleop 等で「マーカが chest に映る」姿勢へ腕を置いてから本スクリプト
  python ik/hand_eye_calib.py collect               # dry-run（動かす姿勢と予測 base 位置を一覧）
  python ik/hand_eye_calib.py collect --go          # 摂動駆動して生ペア収集→解→保存
  # 2) オフセットの当て直し（再収集不要・収集済みの生ペアから再フィット）
  python ik/hand_eye_calib.py solve --marker-offset 0 0 0.05      # 指定オフセットで再フィット
  python ik/hand_eye_calib.py solve --sweep-z 0.0 0.12 0.005      # +Z を自動探索し残差最小を採用
  python ik/hand_eye_calib.py verify                # 保存済み変換の残差[mm]
  python ik/hand_eye_calib.py apply --cam 0.05 0.02 0.40   # カメラ3D→腕ベース3D（reach の --xyz 用）

依存: numpy（apply/solve/verify）。collect は pyrealsense2, opencv, pinocchio, lerobot も。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SAVE_TF = os.path.join(HERE, "hand_eye.npz")           # R(3x3), t(3,), offset(3,)
SAVE_PAIRS = os.path.join(HERE, "hand_eye_pairs.npz")  # cam(N,3), tcp_pos(N,3), tcp_rot(N,3,3)
JK = [f"joint_{i}" for i in range(1, 8)]               # lerobot 関節（度）


# ---- 数学: Umeyama 剛体（スケール無し）-------------------------------------
def umeyama_rigid(src: np.ndarray, dst: np.ndarray):
    """src(N,3)→dst(N,3) を最小二乗で合わせる剛体 R,t を返す（dst ≈ R·src + t）。

    Kabsch/Umeyama: 重心を引いて共分散 H=SᵀD を SVD、反射を det で補正。
    src=カメラ座標点、dst=腕ベース座標点 を渡すと T_base_cam が出る。
    """
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    H = S.T @ D
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = mu_d - R @ mu_s
    return R, t


def residuals_mm(R, t, cam, base):
    """各ペアの ‖R·cam+t − base‖ をミリメートルで返す(N,)。"""
    pred = (np.asarray(cam, float) @ R.T) + t
    return np.linalg.norm(pred - np.asarray(base, float), axis=1) * 1000.0


def base_from_raw(tcp_pos, tcp_rot, offset):
    """生の TCP 姿勢（位置・回転）とマーカオフセット(EE系)から、マーカの腕ベース位置(N,3)を計算。

    p_base_i = tcp_pos_i + tcp_rot_i · offset 。offset を変えればここだけ再計算でき、
    p_cam とのペアを Umeyama し直すだけで再収集なしにオフセットを当て直せる。
    """
    tcp_pos = np.asarray(tcp_pos, float)
    tcp_rot = np.asarray(tcp_rot, float)
    offset = np.asarray(offset, float)
    return tcp_pos + np.einsum("nij,j->ni", tcp_rot, offset)


def transform_cam_to_base(R, t, cam_xyz):
    return (R @ np.asarray(cam_xyz, float)) + t


def _fit_and_report(cam, tcp_pos, tcp_rot, offset):
    """指定オフセットで base を計算→Umeyama→(R,t,base,res)。"""
    base = base_from_raw(tcp_pos, tcp_rot, offset)
    R, t = umeyama_rigid(cam, base)
    res = residuals_mm(R, t, cam, base)
    return R, t, base, res


# ---- 実機収集（Thor）-------------------------------------------------------
def gen_relative_poses(q0_deg: np.ndarray, d: float):
    """現在姿勢 q0 からの相対摂動で複数姿勢を生成（マーカを FOV 内で振る）。

    手先を視野内で並進させつつ奥行きも変える＝非共面な点群になるよう肩/肘/手首を振る。
    d を大きくするほど点が広がり較正は安定するが、FOV 外れ・自己干渉に注意。
    """
    poses = [q0_deg.copy()]
    for ji in (0, 1, 3, 5):              # joint_1/2/4/6（肩旋回・肩ピッチ・肘・手首）
        for s in (+d, -d):
            p = q0_deg.copy(); p[ji] += s; poses.append(p)
    for ja, jb, sa, sb in [(1, 3, +d, +d), (1, 3, -d, +d), (3, 5, +d, -d)]:  # 奥行き変化の組合せ
        p = q0_deg.copy(); p[ja] += sa; p[jb] += sb; poses.append(p)
    return poses


def drive_to(robot, q_goal_deg, *, hz=10.0, timeout_s=8.0, tol_deg=1.5, settle_s=0.6):
    """follower を q_goal[deg] へ寄せる（max_relative_target で各ステップclamp）→ 整定待ち。"""
    goal = {f"{JK[i]}.pos": float(q_goal_deg[i]) for i in range(7)}
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        robot.send_action(goal)
        obs = robot.get_observation()
        cur = np.array([obs[f"{j}.pos"] for j in JK])
        if np.max(np.abs(cur - q_goal_deg)) < tol_deg:
            break
        time.sleep(1.0 / hz)
    time.sleep(settle_s)


def capture_marker_cam(pipe, align, detect_fn, n=6):
    """数フレームで緑マーカ(最大ブロブ)のカメラ3Dを取り中央値[m]を返す。検出ゼロなら None。"""
    xs = []
    for _ in range(n):
        frames = align.process(pipe.wait_for_frames())
        c = frames.get_color_frame()
        dframe = frames.get_depth_frame()
        if not c or not dframe:
            continue
        color = np.asanyarray(c.get_data())
        intr = c.profile.as_video_stream_profile().get_intrinsics()
        cands, _ = detect_fn(color, dframe, intr, viz=False)
        if cands:
            xs.append(cands[0]["xyz_cam"])   # 面積最大＝マーカ
    if not xs:
        return None
    return np.median(np.array(xs, float), axis=0)


def cmd_collect(args):
    # 実機・カメラ・FK の依存はここで遅延 import（apply/solve/verify を numpy だけで動かすため）
    sys.path.insert(0, HERE)
    from openarm_left_ik import OpenArmLeftIK, lerobot_obs_to_q_rad  # noqa: E402
    from detect_okra import make_pipeline, detect_okra_3d           # noqa: E402
    from drive_left_ik import make_robot                            # noqa: E402

    ik = OpenArmLeftIK()
    offset = np.asarray(args.marker_offset, float)  # TCP フレームでのマーカ位置オフセット[m]
    robot = make_robot(args.port, args.side, args.id, args.max_rel)
    robot.connect()
    pipe = align = None
    try:
        obs = robot.get_observation()
        q0_deg = np.array([obs[f"{j}.pos"] for j in JK])
        poses = gen_relative_poses(q0_deg, args.spread_deg)
        print(f"開始姿勢 q0(deg)={np.round(q0_deg, 1).tolist()}")
        print(f"摂動 ±{args.spread_deg}°・{len(poses)} 姿勢・マーカオフセット(EE系)={offset.tolist()}")

        if not args.go:
            print("[dry-run] 以下の姿勢へ動かす予定（--go で実行）。予測は各姿勢のマーカ base 位置[m]:")
            for k, p in enumerate(poses):
                M = ik.fk(np.deg2rad(p))
                p_base = M.translation + M.rotation @ offset
                print(f"  [{k:2d}] dq={np.round(p - q0_deg, 1).tolist()}  base={np.round(p_base, 3).tolist()}")
            print(">> 緑マーカをグリッパ近傍に付け、これらの姿勢でマーカが chest FOV を外れないか確認してから --go")
            return

        pipe, align = make_pipeline(args.serial)
        for _ in range(10):           # 自動露出が落ち着くまで捨てフレーム
            pipe.wait_for_frames()

        cam_pts, tcp_pos, tcp_rot = [], [], []
        for k, p in enumerate(poses):
            drive_to(robot, p)
            M = ik.fk(lerobot_obs_to_q_rad(robot.get_observation()))
            p_cam = capture_marker_cam(pipe, align, detect_okra_3d)
            if p_cam is None:
                print(f"  [{k:2d}] 緑検出ゼロ → スキップ（FOV/閾値を確認）")
                continue
            cam_pts.append(p_cam)
            tcp_pos.append(np.asarray(M.translation, float))
            tcp_rot.append(np.asarray(M.rotation, float))
            p_base = M.translation + M.rotation @ offset
            print(f"  [{k:2d}] cam={np.round(p_cam, 3).tolist()}  base={np.round(p_base, 3).tolist()}")

        drive_to(robot, q0_deg)       # 開始姿勢へ戻す
    finally:
        robot.disconnect()
        if pipe is not None:
            pipe.stop()
        print("（disconnect: torque off。腕が脱力するので支えること）")

    if len(cam_pts) < 4:
        print(f"[abort] 有効ペアが {len(cam_pts)} 個（<4）。FOV/緑閾値/姿勢の広がりを見直して再収集。")
        return
    cam = np.array(cam_pts)
    tcp_pos = np.array(tcp_pos)
    tcp_rot = np.array(tcp_rot)
    np.savez(SAVE_PAIRS, cam=cam, tcp_pos=tcp_pos, tcp_rot=tcp_rot)   # 生ペア（オフセット非依存）
    R, t, _, res = _fit_and_report(cam, tcp_pos, tcp_rot, offset)
    np.savez(SAVE_TF, R=R, t=t, offset=offset)
    print(f"\n== 較正完了: {len(cam)} ペア・offset={offset.tolist()} ==")
    print(f"残差[mm] mean={res.mean():.1f} max={res.max():.1f} median={np.median(res):.1f}")
    print(f"保存: {SAVE_TF}（R,t,offset）, {SAVE_PAIRS}（生ペア cam/tcp_pos/tcp_rot）")
    print(">> 残差が大きい(例 >15mm)なら再収集せず: solve --marker-offset で当て直し / solve --sweep-z で +Z 自動探索")


def cmd_solve(args):
    if not os.path.exists(SAVE_PAIRS):
        print(f"[err] {SAVE_PAIRS} が無い。先に collect を。")
        return
    d = np.load(SAVE_PAIRS)
    cam, tcp_pos, tcp_rot = d["cam"], d["tcp_pos"], d["tcp_rot"]

    if args.sweep_z is not None:
        lo, hi, step = args.sweep_z
        x, y = args.marker_offset[0], args.marker_offset[1]
        best = None
        for z in np.arange(lo, hi + 1e-9, step):
            off = np.array([x, y, float(z)])
            R, t, _, res = _fit_and_report(cam, tcp_pos, tcp_rot, off)
            if best is None or res.mean() < best[3].mean():
                best = (R, t, off, res)
        R, t, offset, res = best
        print(f"sweep-z 最良: offset={np.round(offset, 4).tolist()}  残差[mm] mean={res.mean():.1f} max={res.max():.1f}")
    else:
        offset = np.asarray(args.marker_offset, float)
        R, t, _, res = _fit_and_report(cam, tcp_pos, tcp_rot, offset)
        print(f"再解: offset={offset.tolist()}  {len(cam)} ペア 残差[mm] mean={res.mean():.1f} max={res.max():.1f}")

    np.savez(SAVE_TF, R=R, t=t, offset=offset)
    print(f"保存: {SAVE_TF}")


def cmd_verify(args):
    if not (os.path.exists(SAVE_TF) and os.path.exists(SAVE_PAIRS)):
        print(f"[err] {SAVE_TF} か {SAVE_PAIRS} が無い。先に collect/solve を。")
        return
    tf = np.load(SAVE_TF)
    d = np.load(SAVE_PAIRS)
    R, t = tf["R"], tf["t"]
    offset = tf["offset"] if "offset" in tf.files else np.zeros(3)
    cam, tcp_pos, tcp_rot = d["cam"], d["tcp_pos"], d["tcp_rot"]
    base = base_from_raw(tcp_pos, tcp_rot, offset)
    res = residuals_mm(R, t, cam, base)
    for i, r in enumerate(res):
        print(f"  [{i:2d}] 残差={r:6.1f} mm")
    print(f"== {len(cam)} ペア・offset={np.round(offset, 4).tolist()} "
          f"残差[mm] mean={res.mean():.1f} max={res.max():.1f} median={np.median(res):.1f} ==")


def cmd_apply(args):
    if not os.path.exists(SAVE_TF):
        print(f"[err] {SAVE_TF} が無い。先に collect/solve を。")
        return
    tf = np.load(SAVE_TF)
    p_base = transform_cam_to_base(tf["R"], tf["t"], args.cam)
    print(f"cam={list(args.cam)} → base={np.round(p_base, 4).tolist()} [m]")
    print(f"  reach 例: python ik/drive_left_ik.py reach --xyz "
          f"{p_base[0]:.3f} {p_base[1]:.3f} {p_base[2]:.3f} --go --max-rel 3 --hz 30")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    pc = sub.add_parser("collect", help="実機を摂動駆動して生ペア収集→Umeyama→保存（dry-run 既定）")
    pc.add_argument("--go", action="store_true", help="実際に駆動・収集する（既定は dry-run で姿勢一覧のみ）")
    pc.add_argument("--serial", default=None, help="D435i シリアル（既定=detect_okra の chest）")
    pc.add_argument("--spread-deg", type=float, default=10.0, help="摂動角[deg]（大=広がる/FOV注意）")
    pc.add_argument("--marker-offset", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    metavar=("X", "Y", "Z"), help="マーカの TCP フレームでのオフセット[m]（既定=TCP上）")
    pc.add_argument("--port", default="can_fol_l")
    pc.add_argument("--side", default="left")
    pc.add_argument("--id", default="follower_left")
    pc.add_argument("--max-rel", type=float, default=3.0, help="1ステップ最大関節変化[deg]（安全弁）")

    ps = sub.add_parser("solve", help="保存済み生ペアからオフセットを当て直して再フィット（再収集不要）")
    ps.add_argument("--marker-offset", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    metavar=("X", "Y", "Z"), help="マーカの TCP フレームでのオフセット[m]")
    ps.add_argument("--sweep-z", type=float, nargs=3, default=None, metavar=("MIN", "MAX", "STEP"),
                    help="進入軸(+Z)オフセットを MIN..MAX を STEP で掃引し残差最小を採用（x,y は --marker-offset）")

    sub.add_parser("verify", help="保存済み生ペア＋変換の残差[mm]を表示")

    pa = sub.add_parser("apply", help="カメラ3D→腕ベース3D（保存済み変換を適用）")
    pa.add_argument("--cam", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"),
                    help="カメラ座標3D[m]（detect_okra の出力）")

    args = ap.parse_args()
    if args.mode == "collect":
        if args.serial is None:
            sys.path.insert(0, HERE)
            from detect_okra import CHEST_SERIAL
            args.serial = CHEST_SERIAL
        cmd_collect(args)
    elif args.mode == "solve":
        cmd_solve(args)
    elif args.mode == "verify":
        cmd_verify(args)
    else:
        cmd_apply(args)


if __name__ == "__main__":
    main()
