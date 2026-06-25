#!/usr/bin/env python3
"""chest D435i の color を MJPEG で LAN 配信する Thor 用ビューア.

用途: オクラ収集のシーン設定（白画用紙の背景・マット・かごの画角確認、マットがフレームを
十分に埋めているかの下見）。ブラウザで http://<ThorのIP>:8080 を開くと chest 映像が見える。
（VLA フェーズ1 収集マニュアル §1 シーン設定）

env: Thor の system python3（pyrealsense2 + cv2 4.8.0 あり）。venv 不要。
camera: chest = Intel RealSense D435I serial 344422070898（手首 D405=260322272302 と取り違えないよう serial 指定）。
管理: openarm-orboh リポジトリ管理（PR 経由）。Thor は `git pull` で取得し `scripts/stream_chest.py` を使う。

注意:
- **セル線オーバーレイは持たない（素の映像のみ）**。セルは物理の棘で定義し（マニュアル §2.1）、斜めカメラでは
  画像の一様グリッドが平面と一致しないため、画面にセル線は描かない（誤解防止）。なお本ビューアの描画は
  ライブ映像のみで録画データには入らない（record は別プロセス・カメラ排他で同時起動不可）。
- **カメラ排他**: D435i は 1プロセス占有。record/teleop の前に必ず停止する（`pkill -f stream_chest.py`）。

使い方（Thor 上・リポジトリ取得後）:
    python3 ~/openarm-orboh/scripts/stream_chest.py
    python3 ~/openarm-orboh/scripts/stream_chest.py --port 8080 --width 640 --height 480 --fps 30
閲覧: 同一LANの PC ブラウザで  http://192.168.0.128:8080
停止: Ctrl+C もしくは  pkill -f stream_chest.py
"""
from __future__ import annotations

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import pyrealsense2 as rs

CHEST_SERIAL = "344422070898"  # Intel RealSense D435I（chest）

_latest_jpeg: bytes | None = None
_lock = threading.Lock()
_stop = threading.Event()


def capture_loop(serial, width, height, fps, quality):
    global _latest_jpeg
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    pipeline.start(cfg)
    print(f"[capture] D435i {serial} 起動 {width}x{height}@{fps}")
    enc = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    try:
        while not _stop.is_set():
            frames = pipeline.wait_for_frames(2000)
            color = frames.get_color_frame()
            if not color:
                continue
            img = np.asanyarray(color.get_data())
            ok, buf = cv2.imencode(".jpg", img, enc)
            if ok:
                with _lock:
                    _latest_jpeg = buf.tobytes()
    finally:
        pipeline.stop()
        print("[capture] 停止")


INDEX = (b"<!doctype html><meta charset=utf-8><title>chest stream</title>"
         b"<meta name=viewport content='width=device-width,initial-scale=1'>"
         b"<body style='margin:0;background:#111;overflow:hidden'>"
         b"<img src='/stream' style='width:100vw;height:100vh;"
         b"object-fit:contain;display:block'>")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):  # ログ抑制
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(INDEX)))
            self.end_headers()
            self.wfile.write(INDEX)
            return
        if self.path == "/healthz":
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"ok")
            return
        if self.path != "/stream":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while not _stop.is_set():
                with _lock:
                    frame = _latest_jpeg
                if frame is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                time.sleep(1.0 / 30)
        except (BrokenPipeError, ConnectionResetError):
            pass  # クライアント切断は正常


def main():
    ap = argparse.ArgumentParser(description="chest D435i を MJPEG 配信（Thor・素の映像）")
    ap.add_argument("--serial", default=CHEST_SERIAL, help="RealSense serial（既定=chest D435i）")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--quality", type=int, default=80, help="JPEG 品質 1-100")
    args = ap.parse_args()

    t = threading.Thread(
        target=capture_loop,
        args=(args.serial, args.width, args.height, args.fps, args.quality),
        daemon=True,
    )
    t.start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"[http] 配信開始: http://<ThorのIP>:{args.port}  (例 http://192.168.0.128:{args.port})")
    print("[http] 停止=Ctrl+C / pkill -f stream_chest.py")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[http] 終了中…")
    finally:
        _stop.set()
        server.shutdown()
        t.join(timeout=3)


if __name__ == "__main__":
    main()
