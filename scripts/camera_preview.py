#!/usr/bin/env python
"""RealSense 複数台の color を rerun ビューアへ流す軽量プレビュー（Thor のモニター用）。

なぜ rerun か: venv の OpenCV は GUI 無し（imshow 不可）。record（lerobot）は表示に
rerun を使い、--display_ip/--display_port で「起動済みビューア」へ gRPC 接続する
（src/lerobot/utils/visualization_utils.py: rr.connect_grpc("rerun+http://IP:PORT/proxy")）。
同じビューアをモニターに常駐させておけば、練習中はこのスクリプトが、収集中は record が
同じ画面へ流せる。

⚠ RealSense は SDK 直読が排他。record（DISPLAY_DATA=true DISPLAY_IP=127.0.0.1 …）を
   起動する前に、このプレビューは Ctrl-C で止めること（止めないと record 側が
   カメラを開けず落ちる。09-19 の PICO sender と同じ種類の衝突）。

ラグ: SDK は既定で 0.2 秒ごとにまとめて送る（RERUN_FLUSH_TICK_SECS）。preview_on_monitor.sh は
0.01 にして即送りにしている。手で起動するときも同じ env を付ける。

使い方（Thor）:
  1. モニターのデスクトップで（または ssh から DISPLAY を指定して）ビューアを起動:
       ~/projects/lerobot/.venv/bin/rerun --port 9876 --memory-limit 2GB
  2. 任意の端末から:
       ~/projects/lerobot/.venv/bin/python scripts/camera_preview.py            # 既定 3 台
       ... camera_preview.py --cam chest=344422070898 --cam wrist_right=352122273417
       ... camera_preview.py --list                                             # 接続中の RealSense を列挙して終了
       ... camera_preview.py --no-viewer --frames 60                            # ビューア無しで疎通だけ（fps 表示）
  3. 止める: Ctrl-C（pipeline を stop してから終了）。
"""
from __future__ import annotations

import argparse
import signal
import sys
import time

DEFAULT_CAMS = {
    # 板橋・2026-09-21 実測（新ハブ配下 USB3.0）
    "chest": "344422070898",  # D435i
    "wrist_right": "352122273417",  # D405
    "wrist_left": "409122272149",  # D405（2026-09-21 追加）
}


def parse_cams(items: list[str] | None) -> dict[str, str]:
    if not items:
        return dict(DEFAULT_CAMS)
    cams: dict[str, str] = {}
    for it in items:
        if "=" not in it:
            raise SystemExit(f"--cam は name=serial の形式: {it!r}")
        name, serial = it.split("=", 1)
        cams[name.strip()] = serial.strip()
    return cams


def list_devices() -> int:
    import pyrealsense2 as rs

    ctx = rs.context()
    devs = ctx.query_devices()
    if len(devs) == 0:
        print("RealSense が見つからない（USB・ハブ・給電を確認）")
        return 1
    for d in devs:
        name = d.get_info(rs.camera_info.name)
        serial = d.get_info(rs.camera_info.serial_number)
        port = d.get_info(rs.camera_info.physical_port) if d.supports(rs.camera_info.physical_port) else "?"
        usb = d.get_info(rs.camera_info.usb_type_descriptor) if d.supports(rs.camera_info.usb_type_descriptor) else "?"
        tag = next((k for k, v in DEFAULT_CAMS.items() if v == serial), "-")
        print(f"{name:28s} serial={serial} usb={usb} port={port} role={tag}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cam", action="append", help="name=serial（複数可。省略時は既定 3 台）")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--ip", default="127.0.0.1", help="rerun ビューアの IP（record の DISPLAY_IP と同じ）")
    ap.add_argument("--port", type=int, default=9876, help="rerun ビューアの gRPC ポート（record の DISPLAY_PORT と同じ）")
    ap.add_argument("--spawn", action="store_true", help="接続でなく、この端末の DISPLAY にビューアを spawn する")
    ap.add_argument("--no-viewer", action="store_true", help="表示せず疎通と fps だけ見る")
    ap.add_argument("--frames", type=int, default=0, help="この枚数で自動終了（0=Ctrl-C まで）")
    ap.add_argument("--jpeg-quality", type=int, default=60, help="送信時の JPEG 品質（0=非圧縮の生 RGB）")
    ap.add_argument("--display-scale", type=int, default=2, help="表示用の間引き倍率（2=320x240 で送る。1=等倍）")
    ap.add_argument("--list", action="store_true", help="接続中の RealSense を列挙して終了")
    args = ap.parse_args()

    if args.list:
        return list_devices()

    import numpy as np  # noqa: F401
    import pyrealsense2 as rs

    cams = parse_cams(args.cam)

    rr = None
    if not args.no_viewer:
        import rerun as rr  # type: ignore

        rr.init("openarm_camera_preview")
        if args.spawn:
            rr.spawn(memory_limit="2GB")
        else:
            rr.connect_grpc(url=f"rerun+http://{args.ip}:{args.port}/proxy")
        print(f"[preview] rerun -> {'spawn' if args.spawn else f'{args.ip}:{args.port}'}")

    pipes: dict[str, rs.pipeline] = {}
    try:
        for name, serial in cams.items():
            cfg = rs.config()
            cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.rgb8, args.fps)
            p = rs.pipeline()
            p.start(cfg)
            pipes[name] = p
            print(f"[preview] {name}: serial={serial} {args.width}x{args.height}@{args.fps} started")
    except Exception as e:  # noqa: BLE001
        print(f"[preview] 起動失敗: {e}\n  → record や別プレビューがカメラを掴んでいないか、--list でシリアルを確認", file=sys.stderr)
        for p in pipes.values():
            p.stop()
        return 1

    import threading

    stop = threading.Event()

    def _sig(*_):
        stop.set()

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    counts = dict.fromkeys(pipes, 0)
    drops = dict.fromkeys(pipes, 0)

    # ラグ対策: カメラごとにスレッドで受けて「届いた瞬間に」送る（直列待ちだと 1 台の遅れが
    # 全体に乗る）。送信は JPEG 圧縮（生 RGB 0.9MB/枚 → 数十 KB）でビューアの gRPC と描画の
    # 負荷を落とす。pipeline のキューは最新 1 枚なので、送信が遅れても古い絵は溜まらない。
    def worker(name: str, p) -> None:
        i = 0
        while not stop.is_set() and (args.frames == 0 or i < args.frames):
            ok, frames = p.try_wait_for_frames(timeout_ms=500)
            if not ok:
                drops[name] += 1
                continue
            color = frames.get_color_frame()
            if not color:
                drops[name] += 1
                continue
            counts[name] += 1
            i += 1
            if rr is not None:
                # static=True: 履歴を持たず最新 1 枚だけ保持する。時系列で溜めるとビューアが数分で
                # メモリ上限（2GB）に達し、GC で詰まって送信側がブロックし「周期的なドロップ」になる
                # （2026-09-21 実測: ビューア無しでは 3 台 30fps・ドロップ 0）。プレビューに履歴は不要。
                arr = np.asanyarray(color.get_data())
                if args.display_scale > 1:  # 表示用に間引く（JPEG 圧縮と描画の負荷を 1/scale^2 に）
                    arr = np.ascontiguousarray(arr[:: args.display_scale, :: args.display_scale])
                img = rr.Image(arr)
                rr.log(f"cameras/{name}", img.compress(jpeg_quality=args.jpeg_quality) if args.jpeg_quality > 0 else img, static=True)
        stop.set() if args.frames and i >= args.frames else None

    threads = [threading.Thread(target=worker, args=(n, p), daemon=True, name=n) for n, p in pipes.items()]
    for t in threads:
        t.start()
    t0 = time.monotonic()
    last = t0
    while not stop.is_set():
        time.sleep(0.2)
        now = time.monotonic()
        if now - last >= 2.0:
            dt = now - t0
            stat = "  ".join(f"{n}:{counts[n] / dt:5.1f}fps(drop {drops[n]})" for n in pipes)
            print(f"[preview] {stat}")
            if rr is not None:
                rr.log("stats", rr.TextLog(stat), static=True)
            last = now
    for t in threads:
        t.join(timeout=2.0)

    for p in pipes.values():
        p.stop()
    print("[preview] stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
