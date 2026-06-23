#!/usr/bin/env python
"""オクラ検出（D435i chest）→ カメラ座標3D。

eye-in-hand 把持パイプラインの最初段。D435i の RGB-D から机上のオクラを検出し、
カメラ座標系の3D位置を返す。後段で hand-eye 変換して腕ベース系（+x前/+y左/+z上）へ
移し、IK 粗接近（drive_left_ik.py reach）の目標 xyz にする。

- 知覚: HSV 緑閾値 + 輪郭（学習不要・初版）。机上＝背景が緑でない前提。
  圃場の green-on-green は深度クラスタ/軽量検出器へ後日強化（okra-grasping-act-plan §1.5.3）。
- 3D化: depth を color に整列し、intrinsics で pixel→camera 3D（rs2_deproject_pixel_to_point）。
- 出力はカメラ座標（メートル・RealSense 光学系: +x右/+y下/+z前）。
  hand-eye 変換は未実装（次の段）。本スクリプトはカメラ座標3Dまで。

実行: Thor（D435i 接続）。コードは Vector で編集可。依存: pyrealsense2, opencv-python, numpy。
使い方:
  python ik/detect_okra.py --viz     # ライブ可視化（マスク+候補・q で終了）。閾値合わせに使う
  python ik/detect_okra.py --once    # 1フレーム検出して候補の3D一覧を print
"""
import argparse

import numpy as np

CHEST_SERIAL = "344422070898"  # D435i (chest)

# --- 実機 tuning 前提の初期パラメータ（--viz で合わせる）-------------------
# HSV 緑閾値（OpenCV 表色: H 0-179, S/V 0-255）。オクラの緑を拾い、机/背景を落とす。
HSV_LOWER = (35, 60, 40)
HSV_UPPER = (85, 255, 255)
MIN_AREA_PX = 400        # 輪郭の最小面積[px]（小さなノイズを除去）
DEPTH_MIN_M = 0.15       # 有効深度レンジ[m]（机上把持の想定距離。床/遠景を除外）
DEPTH_MAX_M = 0.80
# --------------------------------------------------------------------------


def make_pipeline(serial, w=640, h=480, fps=30):
    """D435i の color+depth ストリームを開始し、depth→color 整列器を返す。"""
    import pyrealsense2 as rs

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    pipe.start(cfg)
    align = rs.align(rs.stream.color)  # depth を color フレームへ整列（同一 (u,v) で対応）
    return pipe, align


def _robust_depth(depth_frame, u, v, win=3):
    """(u,v) 近傍 (2win+1)^2 の有効深度の中央値[m]。全て無効なら None。

    深度は穴（0）やエッジノイズが出るので点1個でなく近傍中央値を取る。
    """
    ds = []
    for du in range(-win, win + 1):
        for dv in range(-win, win + 1):
            d = depth_frame.get_distance(u + du, v + dv)
            if d > 0:
                ds.append(d)
    if not ds:
        return None
    return float(np.median(ds))


def detect_okra_3d(color_bgr, depth_frame, intr, *, viz=False):
    """1フレームからオクラ候補を検出し camera 座標3D を返す。

    Returns:
        (candidates, mask)
        candidates: list of dict(u, v, area, xyz_cam=(X,Y,Z)[m])  面積の大きい順
        mask: 緑マスク（可視化/デバッグ用）
    """
    import cv2
    import pyrealsense2 as rs

    hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(HSV_LOWER), np.array(HSV_UPPER))
    # 形態学: open でノイズ点を消し、close で莢内部の穴を埋める
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA_PX:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])
        d = _robust_depth(depth_frame, u, v)
        if d is None or not (DEPTH_MIN_M <= d <= DEPTH_MAX_M):
            continue
        # pixel(u,v) + 深度 d → カメラ座標3D（RealSense 光学系）
        X, Y, Z = rs.rs2_deproject_pixel_to_point(intr, [u, v], d)
        out.append({"u": u, "v": v, "area": float(area), "xyz_cam": (X, Y, Z)})
        if viz:
            cv2.drawContours(color_bgr, [c], -1, (0, 0, 255), 2)
            cv2.circle(color_bgr, (u, v), 4, (255, 0, 0), -1)
            cv2.putText(color_bgr, f"{Z:.2f}m", (u + 6, v),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    out.sort(key=lambda o: -o["area"])  # 面積が大きい＝近い/主対象を先頭に
    return out, mask


def main():
    ap = argparse.ArgumentParser(description="オクラ検出（D435i）→ カメラ座標3D")
    ap.add_argument("--serial", default=CHEST_SERIAL, help="D435i シリアル（既定=chest）")
    ap.add_argument("--once", action="store_true", help="1フレーム検出して3D一覧を print")
    ap.add_argument("--viz", action="store_true", help="ライブ可視化（q で終了・閾値合わせ用）")
    args = ap.parse_args()

    import cv2

    pipe, align = make_pipeline(args.serial)
    try:
        for _ in range(10):  # 自動露出が落ち着くまで捨てフレーム
            pipe.wait_for_frames()
        while True:
            frames = align.process(pipe.wait_for_frames())
            cframe = frames.get_color_frame()
            dframe = frames.get_depth_frame()
            if not cframe or not dframe:
                continue
            color = np.asanyarray(cframe.get_data())
            intr = cframe.profile.as_video_stream_profile().get_intrinsics()
            cands, mask = detect_okra_3d(color, dframe, intr, viz=(args.viz or args.once))

            if args.once:
                for i, o in enumerate(cands):
                    X, Y, Z = o["xyz_cam"]
                    print(f"[{i}] u={o['u']} v={o['v']} area={o['area']:.0f} "
                          f"cam_xyz=({X:.3f}, {Y:.3f}, {Z:.3f}) m")
                print(f"検出 {len(cands)} 個（カメラ座標・要 hand-eye で腕ベースへ）")
                break

            if args.viz:
                cv2.imshow("okra detect (color)", color)
                cv2.imshow("green mask", mask)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        pipe.stop()
        if args.viz:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
