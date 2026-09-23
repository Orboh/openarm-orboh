#!/usr/bin/env python3
"""record の端末表示を「操作パネル」風に間引くフィルタ（2026-09-22 Kento 要望）。

record_bimanual_openarm.sh が

    lerobot-record ... 2>&1 | tee -a console.log | python -u scripts/record_console_filter.py

と挟む。console.log には tee で全行が残るので、ここでは操作者に要る行だけを画面に出す。
キー入力は端末の stdin（fd 0）から lerobot が直接読むので、stdout をここで加工しても効く。

画面に出すもの:
  - `====` の横線で囲んだ状態案内（待機中／録画中／戻し損ね）とその中身
  - キー入力の echo（'a' key pressed / Right arrow key pressed ...）
  - Recording episode / Waiting / Arm returned / Start pose / Re-record / Stop recording / Exiting
  - WARNING / ERROR / Traceback（stale frame の連発は最初の 1 行と件数だけ）
  - [record-bi] [collect-bi] [camera-setup] [monitor] [manifest] の wrapper 行
  - カメラ・腕の connected / disconnected
画面から消すもの:
  - 起動時の設定ダンプ（t_record.py の pformat・約 250 行）
  - CAN/カメラの接続手順ログ（socket・handshake・joint limits・gravity model・soft-align）
  - Damiao の集計行（正常時。drops がある WARNING は出す）
  - Streaming encoding の案内

CONSOLE_FILTER=off で素通し（従来表示）。
"""
from __future__ import annotations

import os
import re
import sys

PASS = re.compile(
    r"key pressed|Recording episode|Waiting:|Arm returned|[Ss]tart pose|Re-record|Stop recording|Exiting"
    r"|Manual episode advance|Press A|did not reach|without returning|lerobot-record exit"
    r"|^\[(record-bi|collect-bi|camera-setup|monitor|manifest)\]|connected\.|disconnected\."
    r"|GUI なし|No display detected"
)
DROP = re.compile(
    r"Batch refresh|Created a socket|Values used for joint limits|Streaming encoding|Gravity model ready"
    r"|Soft-aligning|Starting handshake|Handshake successful|Connecting arm|Connected to can_"
    r"|^Svt\[(info|warn)\]|^-{20,}$"  # SVT-AV1 encoder banner at finalize (2026-09-22 dry7: ~60 lines)
    # 3 眼ぶんのエンコーダが同時に stderr へ書くので `Svt[info]:` の接頭辞と本文が別行に割れることがある。
    # 割れた断片は上の行に当たらず端末へ漏れる（2026-09-23 の収集で保存のたびに不定期に出た）。
    r"|^SVT \[|^LIB Build date:|^Preset M\d+ is mapped"
)
ALERT = re.compile(r"\b(WARNING|ERROR|CRITICAL)\b")
DUMP_START = re.compile(r"t_record\.py:\d+ \{")
LEVEL = re.compile(r"^(INFO|WARNING|ERROR|DEBUG|CRITICAL) \d{4}-")


def main() -> None:
    out = sys.stdout

    def emit(s: str) -> None:
        out.write(s)
        out.flush()

    if os.environ.get("CONSOLE_FILTER", "on").lower() in ("off", "0", "false", "no"):
        for raw in sys.stdin:
            emit(raw)
        return

    in_dump = in_banner = in_tb = False
    stale_n = 0

    def flush_stale() -> None:
        nonlocal stale_n
        if stale_n > 1:
            emit(f"  … stale frame の警告 ×{stale_n}（全行は console.log）\n")
        stale_n = 0

    for raw in sys.stdin:
        line = raw.rstrip("\n")
        stripped = line.strip()

        # 状態案内: '=' だけの行で囲まれたブロックは丸ごと通す
        if stripped and set(stripped) == {"="}:
            in_banner = not in_banner
            flush_stale()
            emit(raw)
            continue
        if in_banner:
            emit(raw)
            continue

        # 起動時の設定ダンプ: 開始行から次のログ行まで捨てる
        if in_dump:
            if LEVEL.match(line):
                in_dump = False
            else:
                continue
        if DUMP_START.search(line):
            in_dump = True
            continue

        # Traceback: 次のログ行か wrapper 行まで通す
        if in_tb:
            emit(raw)
            if LEVEL.match(line) or line.startswith("["):
                in_tb = False
            continue
        if line.startswith("Traceback"):
            in_tb = True
            flush_stale()
            emit(raw)
            continue

        # stale frame の連発は最初の 1 行と件数だけ（間に挟まる集計行では区切らない）
        if "stale frame" in line:
            stale_n += 1
            if stale_n == 1:
                emit(raw)
            continue
        if ALERT.search(line):
            flush_stale()
            emit(raw)
            continue
        if DROP.search(line):
            continue
        if stale_n:
            flush_stale()
        if PASS.search(line):
            emit(raw)
            continue
        # ログ行（INFO ...）以外の素の print は通す（キー echo など）。空行は捨てる
        if stripped and not LEVEL.match(line):
            emit(raw)

    flush_stale()


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, BrokenPipeError):
        pass
