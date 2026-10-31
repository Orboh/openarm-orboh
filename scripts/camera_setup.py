#!/usr/bin/env python
"""RealSense の露出・ゲインをデバイスに焼く（D405 の「自動露出だと暗い」対策）。

事実（2026-09-21 板橋・実測）: D405 の自動露出は 30fps の露出上限 33ms に張り付いた後も
ゲインを 16 から上げない（wrist 平均輝度 11〜17/255）。手動ゲイン 128 で 82、248 で 115。
手動露出 33000us は読み出しが間に合わず 15fps に落ちる。28000us・ゲイン 200 で 30fps・平均 97。
D435i は自動露出で 122〜134。brightness +24 で 151、+40 で 169（自動露出は維持）。
D435i（chest）は自動露出で適正（134/255）。設定は **デバイス側に残り、別プロセスの pipeline
再起動後も生きる**（lerobot の RealSense 設定に露出項目が無いので、record はこれを継承する）。
USB 抜き差し・電源断で既定に戻るので、セッション開始時（preview / collect の起動時）に焼く。

使い方:
  python scripts/camera_setup.py --show                          # 現在値
  python scripts/camera_setup.py --d405-gain 200                 # 全 D405 を手動露出 33ms・ゲイン 200
  python scripts/camera_setup.py --d405-gain 200 --serial 409122272149   # 個体指定
  python scripts/camera_setup.py --d405-auto                     # D405 を自動露出（既定）へ戻す
  python scripts/camera_setup.py --d435-brightness 24            # D435i を自動露出のまま底上げ（0 で既定）
値は collect の console.log / command.sh に残す（record_bimanual_openarm.sh の D405_GAIN）。
"""
from __future__ import annotations

import argparse
import sys
import time


def color_sensor(dev, rs):
    for s in dev.query_sensors():
        if rs.stream.color in {p.stream_type() for p in s.get_stream_profiles()}:
            return s
    return None


def setopt(s, opt, v):
    # set_option は USB の一時エラー（RS2_USB_STATUS_PIPE）で落ちることがあるので数回試す
    last = None
    for _ in range(6):
        try:
            s.set_option(opt, v)
            return True
        except RuntimeError as e:  # noqa: PERF203
            last = e
            time.sleep(0.3)
    print(f"[camera-setup] set {opt} = {v} に失敗: {last}", file=sys.stderr)
    return False


def getopt(s, opt, default=-1.0):
    # get_option も USB の一時エラー（RS2_USB_STATUS_PIPE）で落ちることがある。数回試して駄目なら default
    if not s.supports(opt):
        return default
    for _ in range(6):
        try:
            return s.get_option(opt)
        except RuntimeError:
            time.sleep(0.3)
    return default


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--d405-gain", type=int, help="D405 を手動露出＋このゲインに（16..248）")
    ap.add_argument("--d405-exposure", type=int, default=28000, help="D405 手動露出 [us]。33000 だと読み出しが間に合わず 15fps に落ちる（実測）。28000 で 30fps")
    ap.add_argument("--d435-brightness", type=int, help="D435i の brightness オフセット（-64..64。自動露出のまま底上げ。24 で平均 122→151、40 で 169）")
    ap.add_argument("--d435-gain", type=int, help="D435i を手動露出＋このゲインに（brightness オフセットで足りない白飛び対策）")
    ap.add_argument("--d435-exposure", type=int, default=100, help="D435i 手動露出 [us相当の内部単位]。小さいほど暗い。既定 100（自動時の166より短い）")
    ap.add_argument("--d405-auto", action="store_true", help="D405 を自動露出へ戻す")
    ap.add_argument("--d435-auto", action="store_true", help="D435i を自動露出へ戻す（brightness も 0 に）")
    ap.add_argument("--serial", action="append", help="対象シリアル（省略時は全 D405）")
    args = ap.parse_args()

    import pyrealsense2 as rs

    ctx = rs.context()
    devs = list(ctx.query_devices())
    if not devs:
        print("[camera-setup] RealSense が見つからない", file=sys.stderr)
        return 1
    rc = 0
    for d in devs:
        name = d.get_info(rs.camera_info.name)
        serial = d.get_info(rs.camera_info.serial_number)
        s = color_sensor(d, rs)
        if s is None:
            continue
        is_d405 = "D405" in name
        is_d435 = "D435" in name
        target = is_d405 and (not args.serial or serial in args.serial)
        d435_target = is_d435 and (not args.serial or serial in args.serial)
        if d435_target and args.d435_brightness is not None:
            ok = setopt(s, rs.option.enable_auto_exposure, 1)
            ok &= setopt(s, rs.option.brightness, float(args.d435_brightness))
            rc |= 0 if ok else 1
        if d435_target and args.d435_gain is not None:
            ok = setopt(s, rs.option.enable_auto_exposure, 0)
            ok &= setopt(s, rs.option.exposure, float(args.d435_exposure))
            ok &= setopt(s, rs.option.gain, float(args.d435_gain))
            rc |= 0 if ok else 1
        elif d435_target and args.d435_auto:
            ok = setopt(s, rs.option.brightness, 0.0)
            ok &= setopt(s, rs.option.enable_auto_exposure, 1)
            rc |= 0 if ok else 1
        if target and args.d405_gain is not None:
            ok = setopt(s, rs.option.enable_auto_exposure, 0)
            ok &= setopt(s, rs.option.exposure, float(args.d405_exposure))
            ok &= setopt(s, rs.option.gain, float(args.d405_gain))
            rc |= 0 if ok else 1
        elif target and args.d405_auto:
            ok = setopt(s, rs.option.gain, 16.0)
            ok &= setopt(s, rs.option.enable_auto_exposure, 1)
            rc |= 0 if ok else 1
        ae = getopt(s, rs.option.enable_auto_exposure)
        ex = getopt(s, rs.option.exposure)
        g = getopt(s, rs.option.gain)
        br = getopt(s, rs.option.brightness, None) if s.supports(rs.option.brightness) else None
        extra = f" brightness={br:g}" if br is not None else ""
        print(f"[camera-setup] {name.replace('Intel RealSense ', ''):8s} {serial}: auto_exposure={ae:g} exposure={ex:g} gain={g:g}{extra}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
