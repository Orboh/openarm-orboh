#!/usr/bin/env python
"""Damiao モータのフィードバックを複数 CAN バス同時に passive 記録する（送信ゼロ・teleop と並走可）。

目的: 「コイル温度 40C 表示で OT=100C のラッチが出た」矛盾を解くため、ラッチ前後の温度・トルクを
高頻度で CSV に残す。teleop（またはホストの何らかの送信元）が指令/refresh を送っているときだけ
モータは応答する（passive のみでは受信 0 件になりうる。その場合は damiao_motorcheck.py 等で能動 poll する）。

フィードバック 8 バイトのレイアウト（公式マニュアル DM-J4310-en.pdf p.9、openarm_can
dm_motor_control.cpp L88-93 と一致）:
  D0 = motor_id(下位4bit) | state/err(上位4bit)
  D1,D2 = pos 16bit (q_uint = D1<<8 | D2)
  D3,D4 = vel 12bit (dq_uint = D3<<4 | D4>>4)
  D4,D5 = tau 12bit (tau_uint = (D4&0xF)<<8 | D5)
  D6 = T_MOS[C]（ドライバ MOS の平均温度）
  D7 = T_Rotor[C]（マニュアル本文の定義はコイル＝巻線の平均温度。名前は Rotor だが実体は coil）
状態コード（マニュアル p.6/p.9 で確定。0/1=disable/enable は公式表になく実機観測ベース）:
  8=OVER_VOLT 9=UNDER_VOLT A=OVER_CURRENT B=MOS_OVERTEMP C=COIL_OVERTEMP(旧称 ROTOR_OVERTEMP)
  D=COMM_LOSS E=OVERLOAD

使い方:
  ~/projects/lerobot/.venv/bin/python scripts/damiao_feedback_log.py can_fol_r can_lead_r [--out FILE] [--print-every 2.0]
    Ctrl-C で停止。記録中に Enter でその文字列を MARK 行として同じ時刻軸に書ける（例: "squeeze start"）。
  ... damiao_feedback_log.py --report damiao-feedback-20260921T120000Z.csv [--pre 5 --window 30]
    記録済み CSV を解析してラッチ前後の証拠（ピーク/RMS/I^2t/上昇率/冷却曲線/位置誤差）を出す。
    2026-10-05 から 1 行ずつ読む（RAM は CSV の大きさに依らない・300MB の CSV で約 28MB。旧版は約 3.8GB）。
    読むのは CSV だけなので、いつでもあとから作れる。入口は scripts/dmlog_report.sh（latest・run_dir 可）。
  ... --report X.csv --episode N [--take K]   EP_START ep=N 〜 EP_END/EP_DISCARD/EXIT の区間だけを解析する
  ... --report X.csv --since 120 --until 180  t_rel（ロガー起動からの秒）で絞る（--episode と併用で重なりだけ）
  ... damiao_feedback_log.py --live X.csv [--every 5 --window 30 --from-start]
    記録中の CSV を末尾から追い、every 秒ごとに直近 window 秒の要約（モータごとの T_rotor 最大と傾き・LOST/BACK・
    エラー状態・追従誤差 |e|max・直近の MARK）を出す。CAN には触らず読むだけ。Ctrl-C で終わる。入口は scripts/dmlog_live.sh。
  ... damiao_feedback_log.py --selftest
    復号往復とレポート関数を合成データで検証する（実 CAN 不要）。

保存先の例（Thor）: ~/collections/dmlog-<UTC>.csv。--report は Thor でも Vector でも動く（stdlib + python-can のみ）。

裏で自動起動する口（2026-10-05・scripts/lib/dmlog.sh が使う。手で起こす従来の使い方はそのまま動く）:
  ... damiao_feedback_log.py can_fol_l ... --out X.csv --follow RUN_DIR/console.log --marks X.marks
      --summary-out X.summary.txt --pid-file X.pid --watch-pid <親の PID>  </dev/null
    --follow FILE   FILE の追記分を読み、lerobot-record のエピソード境界行（Recording episode N /
                    矢印キー / esc / Re-record / Stop recording 等）を MARK 行（EP_START ep=N ...）にする。
                    "MARK <文字列>" で始まる行はその文字列をそのまま MARK にする（シェルからの印）。
                    標準入力の Enter MARK と違って端末の stdin を読まないので、lerobot のキー入力を奪わない。
    --marks FILE    --follow と同じだが先頭から読む（シェルが MARK を追記する専用ファイル・起動との競合で落とさない）。
    --summary-out   停止時に短い要約（記録時間・受信レート・最大温度・LOST/BACK・エピソード表・MARK）を書く。
                    「判定材料」行（動作中の LOST・エラー状態のフレーム数・最大 T_rotor）を dmlog.sh が読み、
                    異常があったときだけ --report を裏で回す（DMLOG_REPORT=anomaly・2026-10-05）。
    --watch-pid     その PID が消えたら自分も止まる（親スクリプトが SIGKILL 等で trap を走らせずに死んだときの孤児防止）。
    SIGTERM / SIGHUP でも Ctrl-C と同じく CSV を flush して閉じる（裏で起動すると SIGINT は無視されるため）。
"""
from __future__ import annotations

import argparse
import bisect
import csv
import datetime as dt
import os
import queue
import re
import signal
import statistics
import sys
import threading
import time

STATE = {0: "disabled", 1: "enabled", 8: "OVER_VOLT", 9: "UNDER_VOLT", 0xA: "OVER_CURRENT", 0xB: "MOS_OVERTEMP",
         0xC: "COIL_OVERTEMP", 0xD: "COMM_LOSS", 0xE: "OVERLOAD"}
NAMES = {0x11: "joint_1", 0x12: "joint_2", 0x13: "joint_3", 0x14: "joint_4", 0x15: "joint_5", 0x16: "joint_6",
         0x17: "joint_7", 0x18: "gripper"}
# (PMAX, VMAX, TMAX)。dm_motor_constants.hpp L97-112 / lerobot damiao/tables.py L98-112。
# joint_3/4 は実機レジスタ VMAX=10（params-can_fol_r.txt L4-5）で表の DM4340(8) と食い違うが、
# 今回のロガーは openarm_can/lerobot と同じ表を使う（別課題としてコメントのみ残す・facts 参照）。
MTYPE = {
    "joint_1": ("dm8009", 12.5, 45, 54),
    "joint_2": ("dm8009", 12.5, 45, 54),
    "joint_3": ("dm4340", 12.5, 8, 28),
    "joint_4": ("dm4340", 12.5, 8, 28),
    "joint_5": ("dm4310", 12.5, 30, 10),
    "joint_6": ("dm4310", 12.5, 30, 10),
    "joint_7": ("dm4310", 12.5, 30, 10),
    "gripper": ("dm4310", 12.5, 30, 10),
}
# follower position_kp（config_openarm_follower.py L169-172）。位置誤差からの要求トルク推定に使う。
FOLLOWER_KP = {"joint_1": 240, "joint_2": 240, "joint_3": 240, "joint_4": 240,
               "joint_5": 24, "joint_6": 31, "joint_7": 25, "gripper": 25}

CSV_FIELDS = ["t_wall", "t_rel", "iface", "motor", "mtype", "state_hex", "state_name",
              "pos_rad", "vel_rad_s", "tau_nm", "t_mos", "t_rotor", "raw_hex"]


def uint_to_float(x: int, bits: int, lo: float, hi: float) -> float:
    return x / ((1 << bits) - 1) * (hi - lo) + lo


def float_to_uint(x: float, bits: int, lo: float, hi: float) -> int:
    x = min(max(x, lo), hi)
    return round((x - lo) / (hi - lo) * ((1 << bits) - 1))


def decode(data: bytes, motor: str) -> dict:
    """8バイト生フレーム -> 物理値。式は openarm_can dm_motor_control.cpp L88-93 と同一（facts 参照）。"""
    b = data
    state = b[0] >> 4
    q_uint = (b[1] << 8) | b[2]
    dq_uint = (b[3] << 4) | (b[4] >> 4)
    tau_uint = ((b[4] & 0xF) << 8) | b[5]
    t_mos, t_rotor = b[6], b[7]
    _mtype, pmax, vmax, tmax = MTYPE[motor]
    return {
        "state": state,
        "state_name": STATE.get(state, f"code_{state:X}"),
        "pos_rad": uint_to_float(q_uint, 16, -pmax, pmax),
        "vel_rad_s": uint_to_float(dq_uint, 12, -vmax, vmax),
        "tau_nm": uint_to_float(tau_uint, 12, -tmax, tmax),
        "t_mos": t_mos,
        "t_rotor": t_rotor,
    }


def encode_for_selftest(state: int, motor_id: int, pos_rad: float, vel_rad_s: float, tau_nm: float,
                         t_mos: int, t_rotor: int, motor: str) -> bytes:
    """decode() の逆演算。--selftest 専用（実機では使わない＝送信 API ではない）。"""
    _mtype, pmax, vmax, tmax = MTYPE[motor]
    q_uint = float_to_uint(pos_rad, 16, -pmax, pmax)
    dq_uint = float_to_uint(vel_rad_s, 12, -vmax, vmax)
    tau_uint = float_to_uint(tau_nm, 12, -tmax, tmax)
    d0 = ((state & 0xF) << 4) | (motor_id & 0xF)
    d1 = (q_uint >> 8) & 0xFF
    d2 = q_uint & 0xFF
    d3 = (dq_uint >> 4) & 0xFF
    d4 = ((dq_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)
    d5 = tau_uint & 0xFF
    return bytes([d0, d1, d2, d3, d4, d5, t_mos & 0xFF, t_rotor & 0xFF])


# ---------------------------------------------------------------- logger --

def _reader(iface: str, out_q: "queue.Queue", stop: threading.Event, t0_mono: float,
            open_failed: list | None = None) -> None:
    import can

    try:
        bus = can.Bus(channel=iface, interface="socketcan", fd=True)
    except Exception as exc:  # noqa: BLE001 - インタフェース不在等は起動時に握って続行
        print(f"!! {iface}: open failed: {exc}", file=sys.stderr)
        if open_failed is not None:
            open_failed.append(iface)
        return
    try:
        while not stop.is_set():
            msg = bus.recv(timeout=0.2)
            if msg is None or msg.arbitration_id not in NAMES or len(msg.data) < 8:
                continue
            motor = NAMES[msg.arbitration_id]
            dec = decode(msg.data, motor)
            t_wall = msg.timestamp if msg.timestamp else time.time()
            row = {
                "t_wall": f"{t_wall:.6f}",
                "t_rel": f"{time.monotonic() - t0_mono:.6f}",
                "iface": iface,
                "motor": motor,
                "mtype": MTYPE[motor][0],
                "state_hex": f"{dec['state']:X}",
                "state_name": dec["state_name"],
                "pos_rad": f"{dec['pos_rad']:.5f}",
                "vel_rad_s": f"{dec['vel_rad_s']:.5f}",
                "tau_nm": f"{dec['tau_nm']:.5f}",
                "t_mos": dec["t_mos"],
                "t_rotor": dec["t_rotor"],
                "raw_hex": msg.data.hex(),
            }
            out_q.put(row)
    finally:
        bus.shutdown()


def _stdin_marker(out_q: "queue.Queue", stop: threading.Event, t0_mono: float) -> None:
    while not stop.is_set():
        try:
            line = sys.stdin.readline()
        except Exception:  # noqa: BLE001
            return
        if not line:
            return
        out_q.put(_mark_row(line.strip(), t0_mono))


def _mark_row(text: str, t0_mono: float) -> dict:
    return {
        "t_wall": f"{time.time():.6f}",
        "t_rel": f"{time.monotonic() - t0_mono:.6f}",
        "iface": "MARK", "motor": text or "(mark)", "mtype": "", "state_hex": "", "state_name": "",
        "pos_rad": "", "vel_rad_s": "", "tau_nm": "", "t_mos": "", "t_rotor": "", "raw_hex": "",
    }


class EpisodeMarker:
    """lerobot-record の console.log の行 → エピソード境界の MARK 文字列（2026-10-05）。

    境界の行は lerobot_record.py L790/840/856/867/878/897 と control_utils.py のキー echo
    （2026-09-23 の accept run の console.log で実物を確認）。保存完了の行は無い（save_episode は無言）ので、
    「次の Recording episode N+1 が出た」「Re-record / No frames が出なかった」で保存を推定する。
    矢印キーの echo は録画中なら「終了」、待機中なら「開始の合図」で同じ文字列なので、状態はここで持つ。
    MARK の先頭トークンは機械可読（EP_START / EP_END / EP_DISCARD / RETURN / STOP / EXIT ...）。
    """

    REC = re.compile(r"\.py:\d+ Recording episode (\d+)\s*$")
    START_POSE = re.compile(r"\.py:\d+ Episode (\d+) start pose: (.*)$")
    WAITING = re.compile(r"\.py:\d+ Waiting:")
    RESET = re.compile(r"\.py:\d+ Reset the environment")
    RERECORD = re.compile(r"\.py:\d+ Re-record episode\s*$")
    STOP = re.compile(r"\.py:\d+ Stop recording\s*$")
    EXITING = re.compile(r"\.py:\d+ Exiting\s*$")
    RC = re.compile(r"\[record(?:-bi)?\] lerobot-record exit=(-?\d+)")

    def __init__(self) -> None:
        self.recording = False
        self.ep: int | None = None
        self.take = 0

    def _end(self, key: str) -> list[str]:
        if not self.recording:
            return []
        self.recording = False
        return [f"EP_END ep={self.ep} take={self.take} key={key}"]

    def feed(self, line: str) -> list[str]:
        line = line.rstrip()
        if not line:
            return []
        if line.startswith("MARK "):
            return [line[5:].strip()]
        m = self.REC.search(line)
        if m:
            out = self._end("none")  # 念のため（通常は直前に終了の行がある）
            self.ep = int(m.group(1))
            self.take += 1
            self.recording = True
            return out + [f"EP_START ep={self.ep} take={self.take}"]
        if "Right arrow key pressed" in line:
            return self._end("right") or [f"CUE key=right next_after_ep={self.ep}"]
        if "Left arrow key pressed" in line:
            return self._end("left") or [f"KEY left (待機中・直前テイクの破棄要求) ep={self.ep}"]
        if "Escape key pressed" in line:
            return self._end("esc") + ["STOP_REQUEST key=esc"]
        if "'a' key pressed" in line:
            return [f"RETURN ep={self.ep}"]
        if "Arm returned" in line:
            return ["RETURN_DONE"]
        if self.RERECORD.search(line):
            return [f"EP_DISCARD ep={self.ep} take={self.take} reason=rerecord"]
        if "No frames in the current take" in line:
            return [f"EP_DISCARD ep={self.ep} take={self.take} reason=no_frames"]
        if self.WAITING.search(line):
            return self._end("timeout")  # 15 秒ごとの再掲は録画中でなければ無視
        if self.RESET.search(line):
            return self._end("timeout") + ["RESET_WINDOW"]
        m = self.START_POSE.search(line)
        if m:
            return [f"START_POSE ep={m.group(1)} {m.group(2)[:160]}"]
        if self.STOP.search(line):
            return self._end("none") + ["EP_STOP"]
        if self.EXITING.search(line):
            return ["LEROBOT_EXITING"]
        m = self.RC.search(line)
        if m:
            return [f"EXIT rc={m.group(1)}"]
        return []


def _follower(paths: list[str], out_q: "queue.Queue", stop: threading.Event, t0_mono: float,
              poll_s: float = 0.1, from_start: tuple = ()) -> None:
    """paths の追記分だけを読み（tail -F 相当・通常ファイルなので書き手を止めない）、境界行を MARK にする。

    起動時に既にあるファイルは末尾から（設定要約など既存行は拾わない）、後から現れたファイルと
    from_start に挙げたファイル（--marks・シェルが印を打つ専用ファイル）は先頭から読む。
    完全な行だけを処理し、途中の行は次の読みに持ち越す。tqdm 等の \r 上書きは \r でも分割する。
    stop が立った後に 1 回だけ最後の読みをしてから抜ける（終了直前の Stop recording 等を落とさない）。
    """
    marker = EpisodeMarker()
    st: dict[str, dict] = {}
    for p in paths:
        st[p] = {"fh": None, "buf": b"", "pos": 0, "from_start": p in from_start or not os.path.exists(p)}

    def pump() -> None:
        for p, s in st.items():
            try:
                if s["fh"] is None:
                    if not os.path.exists(p):
                        continue
                    s["fh"] = open(p, "rb")
                    if not s["from_start"]:
                        s["fh"].seek(0, os.SEEK_END)
                    s["pos"] = s["fh"].tell()
                fh = s["fh"]
                size = os.fstat(fh.fileno()).st_size
                if size < s["pos"]:  # 切り詰められた → 頭から
                    fh.seek(0)
                    s["buf"] = b""
                chunk = fh.read()
                s["pos"] = fh.tell()
            except OSError:
                continue
            if not chunk:
                continue
            s["buf"] += chunk
            *lines, s["buf"] = s["buf"].split(b"\n")
            for raw in lines:
                for seg in raw.decode("utf-8", errors="replace").split("\r"):
                    for text in marker.feed(seg):
                        out_q.put(_mark_row(text, t0_mono))

    while not stop.is_set():
        pump()
        stop.wait(poll_s)
    pump()
    for s in st.values():
        if s["fh"] is not None:
            s["fh"].close()


def episode_table(marks: list[tuple[str, str]], exit_rc: str | None = None,
                  t_range: tuple | None = None) -> list[str]:
    """MARK（(t_rel, text) の列）からテイク表を作る。1 テイク 1 行・データセットの episode 番号と 1 対 1。

    保存の判定（lerobot は保存の行を出さないので推定）:
      EP_DISCARD が付いたテイク → 破棄。次のテイクの ep が +1 → 保存。最後のテイクは EP_STOP まで
      破棄が無く、終了コードが 0 なら保存（推定）、それ以外は不明（Ctrl-C で落ちたら未保存のことがある）。
    """
    takes: list[dict] = []
    kv = re.compile(r"(\w+)=(\S+)")
    for t_rel, text in marks:
        head = text.split(" ", 1)[0]
        d = dict(kv.findall(text))
        if head == "EP_START":
            takes.append({"ep": d.get("ep"), "take": d.get("take"), "t0": float(t_rel), "t1": None,
                          "key": "-", "status": None})
        elif head == "EP_END" and takes:
            takes[-1]["t1"] = float(t_rel)
            takes[-1]["key"] = d.get("key", "-")
        elif head == "EP_DISCARD" and takes:
            takes[-1]["status"] = "破棄(" + d.get("reason", "?") + ")"
        elif head == "EXIT" and exit_rc is None:
            exit_rc = d.get("rc")
    for i, tk in enumerate(takes):
        if tk["status"]:
            continue
        nxt = takes[i + 1] if i + 1 < len(takes) else None
        if nxt is not None:
            try:
                tk["status"] = "保存" if int(nxt["ep"]) == int(tk["ep"]) + 1 else "破棄(番号が進まず)"
            except (TypeError, ValueError):
                tk["status"] = "不明"
        else:
            tk["status"] = "保存(推定)" if exit_rc == "0" else f"不明(exit={exit_rc})"
    lines = []
    for tk in takes:
        if t_range is not None:  # 切り出し（2026-10-05）: 範囲に掛かるテイクだけ出す（保存の判定はファイル全体で済ませた後）
            lo, hi = t_range
            if (hi is not None and tk["t0"] > hi) or (lo is not None and (tk["t1"] if tk["t1"] is not None else tk["t0"]) < lo):
                continue
        dur = f"{tk['t1'] - tk['t0']:.1f}s" if tk["t1"] is not None else "-"
        t1 = f"{tk['t1']:.1f}" if tk["t1"] is not None else "-"
        lines.append(f"   ep={tk['ep']} take={tk['take']} t_rel={tk['t0']:.1f}-{t1}s ({dur}) "
                     f"end={tk['key']} → {tk['status']}")
    kept = sum(1 for tk in takes if tk["status"].startswith("保存"))
    if takes:
        lines.insert(0, f"テイク {len(takes)} 本（保存 {kept}・それ以外 {len(takes) - kept}）"
                        + ("・範囲に掛かるものだけ表示" if t_range is not None else ""))
    return lines


# LOST の分類（2026-10-05・停止時の自動解析を「異常があったときだけ」にするための判定材料）。
# LOST は「同じバスの誰かが 0.3 秒以内に応答しているのに、そのモータだけ 0.3 秒以上無応答」。起動の整列や
# エピソードの保存・終了の脱力でバス全体が黙ると、最後に応答したモータの 0.3 秒の間だけ他の全モータが LOST に
# なり、まとまった束が出る（2026-09-21 の t_rel 20.43s の LOST/BACK x7・dmlog-elbow-check3 の 12.55〜15.06s。
# どちらも偽陽性と確認済み・dev-log 2026-09-21 §2 の 13/16）。本物（肘のコネクタが開いて J5〜gripper だけ落ちる等）は、
# その無応答の間も同じバスの他モータがループの周期で流れ続ける。そこで「無応答の間に同じバスの他モータから
# 届いたフレーム数」が、他モータ 1 台あたり LOST_RUN_MIN_FRAMES_PER_OTHER 以上なら「動作中の LOST」と数える。
# 束では他モータは高々 1〜2 回（handshake・最後の 1 周）しか応答しない。30Hz でも 0.3 秒で 1 台あたり 9 回は届く。
LOST_RUN_MIN_FRAMES_PER_OTHER = 5


class LostTracker:
    """バスごとの受信数で、モータの無応答の間に同じバスの他モータがどれだけ応答したかを数える（O(1)/行）。"""

    def __init__(self) -> None:
        self.bus_frames: dict[str, int] = {}
        self.at_seen: dict[tuple, int] = {}
        self.motors: dict[str, set] = {}

    def frame(self, iface: str, motor: str) -> int:
        """データ行 1 つを数え、このモータの直前の無応答の間に同じバスの他モータから届いた数を返す。"""
        n = self.bus_frames.get(iface, 0)
        key = (iface, motor)
        prev = self.at_seen.get(key)
        other = n - prev if prev is not None else 0
        n += 1
        self.bus_frames[iface] = n
        self.at_seen[key] = n
        self.motors.setdefault(iface, set()).add(motor)
        return other

    def silence(self, iface: str, motor: str) -> int:
        """まだ無応答のモータについて、ここまでに同じバスの他モータから届いた数。"""
        prev = self.at_seen.get((iface, motor))
        return self.bus_frames.get(iface, 0) - prev if prev is not None else 0

    def is_running(self, iface: str, other: int) -> bool:
        n_other = max(len(self.motors.get(iface, ())) - 1, 1)
        return other >= LOST_RUN_MIN_FRAMES_PER_OTHER * n_other


def run_logger(ifaces: list[str], out_path: str, print_every: float, follow: list[str] | None = None,
               summary_out: str | None = None, watch_pid: int | None = None,
               pid_file: str | None = None, marks_files: list[str] | None = None) -> int:
    import can  # noqa: F401 - 早期に import エラーを出す

    t0_mono = time.monotonic()
    start_wall = dt.datetime.now(dt.timezone.utc)
    out_q: "queue.Queue" = queue.Queue()
    stop = threading.Event()

    # 裏で起動されたとき（bash の & は SIGINT を無視させる）は SIGTERM / SIGHUP で止める。
    # 例外を投げず旗を立てるだけにする（行の書きかけで割り込まない）。メインループが 0.2 秒以内に拾う。
    stop_reason: list[str] = []

    def _on_signal(signum, _frame) -> None:
        stop_reason.append(signal.Signals(signum).name)

    for _sig in (signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(_sig, _on_signal)
        except (ValueError, OSError):
            pass
    if pid_file:
        try:
            with open(pid_file, "w") as pf:
                pf.write(f"{os.getpid()}\n")
        except OSError as exc:
            print(f"!! pid-file: {exc}", file=sys.stderr)

    f = open(out_path, "w", newline="")
    f.write(f"# start_wall={start_wall.isoformat()} host={__import__('socket').gethostname()} "
            f"ifaces={','.join(ifaces)}\n")
    writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
    writer.writeheader()

    open_failed: list[str] = []
    readers = [threading.Thread(target=_reader, args=(i, out_q, stop, t0_mono, open_failed), daemon=True)
               for i in ifaces]
    marker = threading.Thread(target=_stdin_marker, args=(out_q, stop, t0_mono), daemon=True)
    follower = None
    if follow or marks_files:
        follower = threading.Thread(target=_follower, daemon=True,
                                    args=(list(follow or []) + list(marks_files or []), out_q, stop, t0_mono),
                                    kwargs={"from_start": tuple(marks_files or [])})
    for t in readers:
        t.start()
    marker.start()
    if follower is not None:
        follower.start()

    # 状態管理: 直近5秒の |tau| リングバッファと現在 state（初回遷移検出用）
    ring: dict[tuple, list[tuple[float, float]]] = {}
    cur_state: dict[tuple, int] = {}
    # 無応答イベント検出（接点の開閉を姿勢と結びつける・2026-09-21 右 leader J5〜gripper の姿勢依存断）:
    # バスに他モータの応答が続いているのに特定モータだけ LOST_AFTER_S 以上応答が無ければ LOST、
    # 戻ったら BACK。どちらも同バスの joint_1..4 の角度を添えて CSV（iface=EVENT）と画面に出す。
    LOST_AFTER_S = 0.3
    last_seen: dict[tuple, float] = {}
    last_pos: dict[tuple, float] = {}
    lost: set = set()
    bus_last: dict[str, float] = {}

    def blank_row(t_rel_v: float, iface_v: str, text: str) -> dict:
        r = {k: "" for k in CSV_FIELDS}
        r.update({"t_wall": dt.datetime.now(dt.timezone.utc).isoformat(), "t_rel": f"{t_rel_v:.4f}",
                  "iface": iface_v, "motor": text})
        return r

    def posture(iface_v: str) -> str:
        return " ".join(f"J{i}={last_pos.get((iface_v, f'joint_{i}'), float('nan'))*57.2958:.1f}" for i in range(1, 5))

    def can_stats(iface_v: str) -> str:
        import subprocess
        try:
            out = subprocess.run(["ip", "-s", "-details", "link", "show", iface_v], capture_output=True, text=True, timeout=1).stdout
        except Exception:
            return "n/a"
        st = ""; berr = ""; cnts = ""
        lines_ = out.splitlines()
        for i, l in enumerate(lines_):
            if "berr-counter" in l:
                st = l.strip().split("berr-counter")[0].strip(); berr = "berr-counter" + l.split("berr-counter", 1)[1].strip()
            if "bus-errors" in l and i + 1 < len(lines_):
                cnts = "restarted/bus-err/arb-lost/err-warn/err-pass/bus-off=" + "/".join(lines_[i + 1].split()[:6])
        return f"{st} {berr} {cnts}".strip()

    last_stats = time.monotonic()
    counts: dict[str, int] = {i: 0 for i in ifaces}
    tmax_seen: dict[tuple, int] = {}
    tmos_seen: dict[tuple, int] = {}
    marks: list[tuple[str, str]] = []
    events: list[str] = []          # LOST / BACK / 状態遷移（要約用）
    n_lost = 0
    n_back = 0
    # 停止時の自動解析の判定材料（2026-10-05・dmlog.sh が要約の「判定材料」行を読む）
    lost_tr = LostTracker()
    n_lost_run = 0                  # 動作中の LOST（束を除く・LostTracker 参照）
    n_bad = 0                       # 状態コードが disabled/enabled 以外のフレーム数（記録開始時点のラッチも含む）
    first_bad = ""
    last_watch = time.monotonic()
    last_lost_check = 0.0
    last_print = time.monotonic()
    last_any = time.monotonic()
    last_flush = time.monotonic()
    print(f"logging to {out_path} (Ctrl-C to stop; Enter=MARK) ifaces={ifaces}")

    try:
        while True:
            try:
                row = out_q.get(timeout=0.2)
            except queue.Empty:
                row = None
            if row is not None:
                writer.writerow(row)
                if row["iface"] != "MARK":
                    key = (row["iface"], row["motor"])
                    counts[row["iface"]] += 1
                    t_rel = float(row["t_rel"])
                    tau = float(row["tau_nm"])
                    buf = ring.setdefault(key, [])
                    buf.append((t_rel, tau))
                    while buf and t_rel - buf[0][0] > 5.0:
                        buf.pop(0)
                    st = int(row["state_hex"], 16)
                    other = lost_tr.frame(row["iface"], row["motor"])
                    if st not in (0, 1):
                        n_bad += 1
                        if not first_bad:
                            first_bad = f"{row['iface']} {row['motor']} {row['state_name']} t_rel={t_rel:.2f}s"
                    last_seen[key] = t_rel
                    bus_last[row["iface"]] = t_rel
                    try:
                        last_pos[key] = float(row["pos_rad"])
                    except (KeyError, ValueError):
                        pass
                    if key in lost:
                        lost.discard(key)
                        msg = f"BACK {row['iface']} {row['motor']} at t_rel={t_rel:.2f}s posture[{posture(row['iface'])}] canstat[{can_stats(row['iface'])}]"
                        print(">>> " + msg)
                        writer.writerow(blank_row(t_rel, "EVENT", msg))
                        n_back += 1
                        events.append(msg)
                        if lost_tr.is_running(row["iface"], other):
                            n_lost_run += 1
                    prev = cur_state.get(key)
                    cur_state[key] = st
                    tmax_seen[key] = max(tmax_seen.get(key, 0), int(row["t_rotor"]))
                    tmos_seen[key] = max(tmos_seen.get(key, 0), int(row["t_mos"]))
                    if prev is not None and prev in (0, 1) and st not in (0, 1):
                        tau_peak5 = max((abs(x[1]) for x in buf), default=0.0)
                        msg = (f"{row['iface']} {row['motor']} {STATE.get(prev, prev)}->"
                               f"{row['state_name']} at t_rel={t_rel:.2f}s wall={row['t_wall']} "
                               f"T_mos={row['t_mos']} T_rotor={row['t_rotor']} "
                               f"|tau| max(prev 5s)={tau_peak5:.2f}")
                        print("!!! " + msg)
                        events.append("STATE " + msg)
                else:
                    print(f"MARK t_rel={row['t_rel']}s: {row['motor']}")
                    marks.append((row["t_rel"], row["motor"]))
                last_any = time.monotonic()

            now = time.monotonic()
            # LOST 判定: バスは生きている（0.3 秒以内に誰かが応答）のに、そのモータだけ 0.3 秒以上無応答
            t_now_rel = now - t0_mono
            # 判定は 50ms に 1 回で足りる（閾値 0.3 秒）。従来は受信 1 行ごとに全モータを走査していて、
            # teleop 200Hz×32 モータでは CPU の大半がここだった（2026-10-05・record と並走させるため間引く）。
            if now - last_lost_check < 0.05:
                last_seen_items = ()
            else:
                last_lost_check = now
                last_seen_items = list(last_seen.items())
            for key, seen in last_seen_items:
                iface_v, motor_v = key
                if key in lost:
                    continue
                if t_now_rel - seen > LOST_AFTER_S and t_now_rel - bus_last.get(iface_v, 0.0) < LOST_AFTER_S:
                    lost.add(key)
                    msg = f"LOST {iface_v} {motor_v} at t_rel={t_now_rel:.2f}s (last seen {seen:.2f}s) posture[{posture(iface_v)}] canstat[{can_stats(iface_v)}]"
                    print(">>> " + msg)
                    writer.writerow(blank_row(t_now_rel, "EVENT", msg))
                    n_lost += 1
                    events.append(msg)
            if now - last_stats >= 1.0:
                for iface_v in ifaces:
                    writer.writerow(blank_row(t_now_rel, "CANSTAT", f"{iface_v} {can_stats(iface_v)}"))
                last_stats = now
            if now - last_flush > 1.0:
                f.flush()
                last_flush = now
            if now - last_print >= print_every:
                elapsed = max(now - t0_mono, 1e-6)
                for iface in ifaces:
                    rate = counts[iface] / elapsed
                    motors_here = [m for (i, m) in ring if i == iface]
                    if not motors_here:
                        continue
                    worst_motor = max(motors_here, key=lambda m: tmax_seen.get((iface, m), 0))
                    bad = [m for m in motors_here if cur_state.get((iface, m)) not in (0, 1, None)]
                    print(f"{iface}: {rate:.1f} f/s  max T_rotor={tmax_seen.get((iface, worst_motor), 0)}C"
                          f"@{worst_motor}  bad={','.join(bad) if bad else '-'}")
                if now - last_any > 10.0:
                    print("(受信なし: teleop 未起動か未給電)")
                last_print = now
            if watch_pid and now - last_watch >= 1.0:
                last_watch = now
                try:
                    os.kill(watch_pid, 0)
                    with open(f"/proc/{watch_pid}/stat") as sf_:  # 回収待ちのゾンビも「消えた」扱い
                        if sf_.read().rsplit(")", 1)[-1].split()[0] == "Z":
                            stop_reason.append(f"watch-pid {watch_pid} gone (zombie)")
                except ProcessLookupError:
                    stop_reason.append(f"watch-pid {watch_pid} gone")
                except (OSError, IndexError):
                    pass
            if stop_reason:
                print(f"stop requested: {stop_reason[0]}")
                break
    except KeyboardInterrupt:
        stop_reason.append("SIGINT")
    finally:
        stop.set()
        for t in readers:
            t.join(timeout=1.0)
        if follower is not None:
            follower.join(timeout=2.0)
        # 止める直前に積まれた行（受信・MARK）も書く（従来は queue に残った分を捨てていた）
        while True:
            try:
                row = out_q.get_nowait()
            except queue.Empty:
                break
            try:
                writer.writerow(row)
            except Exception:  # noqa: BLE001
                continue
            if row.get("iface") == "MARK":
                marks.append((row["t_rel"], row["motor"]))
        f.flush()
        f.close()
        for key in lost:  # 止めた時点でまだ無応答のもの（BACK が無い）も同じ基準で数える
            if lost_tr.is_running(key[0], lost_tr.silence(*key)):
                n_lost_run += 1
        if summary_out:
            try:
                lines = _logger_summary(out_path, start_wall, time.monotonic() - t0_mono, ifaces, counts,
                                        open_failed, tmax_seen, tmos_seen, n_lost, n_back, events, marks,
                                        stop_reason, can_stats,
                                        anomaly={"run_lost": n_lost_run, "bad": n_bad, "first_bad": first_bad})
                with open(summary_out, "w") as sf:
                    sf.write("\n".join(lines) + "\n")
            except Exception as exc:  # noqa: BLE001 - 要約の失敗で CSV を巻き込まない
                print(f"!! summary failed: {exc}", file=sys.stderr)
        if pid_file:
            try:
                os.remove(pid_file)
            except OSError:
                pass
    return 0


def _logger_summary(out_path, start_wall, elapsed, ifaces, counts, open_failed, tmax_seen, tmos_seen,
                    n_lost, n_back, events, marks, stop_reason, can_stats, anomaly: dict | None = None) -> list[str]:
    """停止時の短い要約（CSV を読み直さない・一瞬で終わる）。'== MARK (全件)' 以降は端末に出さない前提。

    「判定材料」行（2026-10-05）は dmlog.sh が読んで、--report を裏で回すか決める（DMLOG_REPORT=anomaly）。
    末尾の [run_lost=.. bad_state=.. max_t_rotor=..] は機械可読（書式を変えるときは dmlog.sh も直す）。
    """
    el = max(elapsed, 1e-6)
    out = [f"Damiao フィードバック記録の要約: {out_path}",
           f"記録時間 {elapsed:.0f}s（start_wall={start_wall.isoformat(timespec='seconds')}）"
           f" 停止={stop_reason[0] if stop_reason else '-'}"]
    for i in ifaces:
        motors = sorted(m for (ii, m) in tmax_seen if ii == i)
        if i in open_failed:
            out.append(f"{i}: open failed（この iface は記録していない）")
            continue
        if not motors:
            out.append(f"{i}: 受信 0 件")
            continue
        worst = max(motors, key=lambda m: tmax_seen[(i, m)])
        out.append(f"{i}: {counts.get(i, 0) / el:.0f} f/s  モータ {len(motors)}  最大 T_rotor="
                   f"{tmax_seen[(i, worst)]}C@{worst}  最大 T_mos={max(tmos_seen.get((i, m), 0) for m in motors)}C")
    grip = [f"{i}={tmax_seen[(i, 'gripper')]}C" for i in ifaces if (i, "gripper") in tmax_seen]
    out.append("gripper 最大 T_rotor: " + (" ".join(grip) if grip else "受信なし"))
    n_state = sum(1 for e in events if e.startswith("STATE "))
    out.append(f"LOST {n_lost} 回 / BACK {n_back} 回 / エラー状態への遷移 {n_state} 回")
    if anomaly is not None:
        tmax_all = max(tmax_seen.values(), default=0)
        where = ""
        if tmax_seen:
            wi, wm = max(tmax_seen, key=lambda k: tmax_seen[k])
            where = f"@{wi} {wm}"
        out.append(f"判定材料: 動作中の LOST {anomaly['run_lost']} 回（起動・保存・停止の束は除く）/ "
                   f"エラー状態のフレーム {anomaly['bad']}（最初: {anomaly['first_bad'] or '-'}）/ "
                   f"最大 T_rotor {tmax_all}C{where}  "
                   f"[run_lost={anomaly['run_lost']} bad_state={anomaly['bad']} max_t_rotor={tmax_all}]")
    for e in events[:8]:
        out.append("  " + e[:220])
    if len(events) > 8:
        out.append(f"  ...ほか {len(events) - 8} 件（CSV の iface=EVENT と --report を参照）")
    tbl = episode_table(marks)
    if tbl:
        out.append("エピソード（MARK から）: " + tbl[0])
        out.extend(tbl[1:])
    for i in ifaces:
        if i not in open_failed:
            out.append(f"canstat {i}: {can_stats(i) or 'n/a'}")
    out.append(f"MARK {len(marks)} 行")
    out.append("== MARK (全件)")
    out.extend(f"   t_rel={t}s: {txt}" for t, txt in marks)
    return out


# ---------------------------------------------------------------- report --

def _lstsq_slope(xs: list[float], ys: list[float]) -> float:
    """最小二乗の傾き（単位: ys単位/秒）。点が2未満なら0。"""
    n = len(xs)
    if n < 2:
        return 0.0
    mx = sum(xs) / n
    my = sum(ys) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    den = sum((x - mx) ** 2 for x in xs)
    return num / den if den else 0.0


def _trapz_sq(ts: list[float], vs: list[float]) -> float:
    """積分 v^2 dt（台形則）。I^2t の代理（トルクの二乗積分＝発熱の目安）。"""
    if len(ts) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(ts)):
        dt_ = ts[i] - ts[i - 1]
        if dt_ <= 0:
            continue
        total += 0.5 * (vs[i] ** 2 + vs[i - 1] ** 2) * dt_
    return total


class _Spool:
    """行の溜め置き。少ないうちはメモリ、多くなったら一時ファイルへ（出力の行数が CSV に比例しても RAM は増えない）。"""

    MEM_LINES = 2000

    def __init__(self, tmpdir: str | None) -> None:
        self.lines: list[str] = []
        self.fh = None
        self.tmpdir = tmpdir
        self.n = 0

    def add(self, line: str) -> None:
        self.n += 1
        if self.fh is None:
            self.lines.append(line)
            if len(self.lines) > self.MEM_LINES:
                import tempfile
                self.fh = tempfile.TemporaryFile("w+", encoding="utf-8", dir=self.tmpdir)
                self.fh.writelines(x + "\n" for x in self.lines)
                self.lines = []
        else:
            self.fh.write(line + "\n")

    def __iter__(self):
        if self.fh is None:
            yield from self.lines
            return
        self.fh.flush()
        self.fh.seek(0)
        for x in self.fh:
            yield x.rstrip("\n")

    def close(self) -> None:
        if self.fh is not None:
            self.fh.close()


class _FolSeries:
    """follower の (t_rel, pos) を一時ファイルに double 2 個ずつ書き、2 パス目に先頭から順に読む。

    位置誤差の最近傍は、旧版の bisect_left（t 以上の最初の点と、その 1 つ前）と同じ点を、lead の時刻順に
    進む 2 本指で選ぶ（1 キーの t_rel はその iface の受信スレッドが単調に付けるので、ファイル順＝時刻順）。
    """

    CHUNK = 4096

    def __init__(self, tmpdir: str | None) -> None:
        import array
        import tempfile
        self._array = array.array
        self.fh = tempfile.TemporaryFile("w+b", dir=tmpdir)
        self.buf = array.array("d")
        self.n = 0

    def add(self, t: float, pos: float) -> None:
        self.buf.append(t)
        self.buf.append(pos)
        self.n += 1
        if len(self.buf) >= 2 * self.CHUNK:
            self.buf.tofile(self.fh)
            del self.buf[:]

    def finish(self) -> None:
        if self.buf:
            self.buf.tofile(self.fh)
            del self.buf[:]
        self.fh.flush()

    def cursor(self) -> "_FolCursor":
        return _FolCursor(self)


class _FolCursor:
    def __init__(self, series: _FolSeries) -> None:
        self.s = series
        self.s.fh.seek(0)
        self.chunk = self.s._array("d")
        self.ci = 0
        self.left = series.n
        self.prev: tuple | None = None
        self.cur: tuple | None = self._next()

    def _next(self) -> tuple | None:
        if self.ci >= len(self.chunk):
            if self.left <= 0:
                return None
            k = min(self.left, _FolSeries.CHUNK)
            self.chunk = self.s._array("d")
            self.chunk.fromfile(self.s.fh, 2 * k)
            self.left -= k
            self.ci = 0
        v = (self.chunk[self.ci], self.chunk[self.ci + 1])
        self.ci += 2
        return v

    def nearest(self, t: float) -> float | None:
        while self.cur is not None and self.cur[0] < t:
            self.prev = self.cur
            self.cur = self._next()
        cands = []
        if self.cur is not None:
            cands.append(self.cur)
        if self.prev is not None:
            cands.append(self.prev)
        if not cands:
            return None
        return min(cands, key=lambda tv: abs(tv[0] - t))[1]


def _iter_csv(path: str):
    """CSV を 1 行ずつ返す（'#' 行は飛ばす・全行をメモリに載せない）。最初に列名→位置の dict を 1 回返す。"""
    with open(path, newline="") as f:
        rd = csv.reader(l for l in f if not l.startswith("#"))  # noqa: E741
        try:
            header = next(rd)
        except StopIteration:
            return
        n = len(header)
        yield {k: i for i, k in enumerate(header)}
        for row in rd:
            if len(row) < n:  # 書きかけの末尾行（SIGKILL 等）。欠けた列は空にして下で弾く
                row = row + [""] * (n - len(row))
            yield row


def scan_marks(path: str) -> list[tuple[str, str]]:
    """MARK 行だけを拾う（切り出しの範囲を決めるための前読み・速い）。"""
    out: list[tuple[str, str]] = []
    it = _iter_csv(path)
    idx = next(it, None)
    if idx is None:
        return out
    ii, it_, im = idx["iface"], idx["t_rel"], idx["motor"]
    for row in it:
        if row[ii] == "MARK":
            out.append((row[it_], row[im]))
    return out


def episode_range(marks: list[tuple[str, str]], ep: int | None, take: int | None):
    """EP_START ep=N [take=K] から、対応する EP_END / EP_DISCARD / EP_STOP / EXIT / 次の EP_START までの t_rel。

    同じ ep が録り直しで複数あるときは、take を指定しなければ最後のテイク（保存されたもの）を選ぶ。
    戻り値 (t_lo, t_hi|None, 始点の MARK, 終点の MARK|None)。見つからなければ None。
    """
    kv = re.compile(r"(\w+)=(\S+)")
    starts = []
    for i, (_t, text) in enumerate(marks):
        if text.split(" ", 1)[0] != "EP_START":
            continue
        d = dict(kv.findall(text))
        if ep is not None and d.get("ep") != str(ep):
            continue
        if take is not None and d.get("take") != str(take):
            continue
        starts.append(i)
    if not starts:
        return None
    i0 = starts[-1]
    t_lo = float(marks[i0][0])
    for t, text in marks[i0 + 1:]:
        if text.split(" ", 1)[0] in ("EP_END", "EP_DISCARD", "EP_STOP", "EXIT", "EP_START"):
            return t_lo, float(t), marks[i0][1], text
    return t_lo, None, marks[i0][1], None


def run_report(path: str, pre_s: float, window_s: float, t_lo: float | None = None, t_hi: float | None = None,
               range_label: str | None = None) -> int:
    """記録済み CSV の解析。2026-10-05 にストリーミングへ作り直した（出力の節・数値は旧版と同じ）。

    旧版は CSV 全行を dict で持ち（CSV の約 10〜15 倍の RAM）、収集を続けて回す日に裏で走らせられなかった。
    今は 2 回読む: 1 パス目でキーごとの件数・状態・遷移・最初のエラー時刻・全区間の和を数え、follower の位置を
    一時ファイルへ書く。2 パス目でエラー前の窓（--pre・--window）・冷却曲線・位置誤差を出す。RAM はモータ数と
    窓の大きさ（窓内の行だけを持つ）で決まり、CSV の大きさに比例しない。窓・平均・中央値は旧版と同じ順序で
    同じ式を当てるので、数値は一致する（tests は --selftest と、旧版との突き合わせ）。
    t_lo / t_hi を与えると t_rel がその範囲の行だけを解析する（--episode / --since / --until）。
    テイク表は範囲に関係なくファイル全体の MARK から作る（端の保存判定が狂わないように）。
    """
    import tempfile

    tmp_parent = os.path.dirname(os.path.abspath(path))
    if not os.access(tmp_parent, os.W_OK):
        tmp_parent = None
    with tempfile.TemporaryDirectory(prefix=".dmlog-report-", dir=tmp_parent) as tmpdir:
        return _run_report_stream(path, pre_s, window_s, t_lo, t_hi, range_label, tmpdir)


def _run_report_stream(path, pre_s, window_s, t_lo, t_hi, range_label, tmpdir) -> int:
    SPECIAL = ("MARK", "EVENT", "CANSTAT")

    def inr(tv: float) -> bool:
        return (t_lo is None or tv >= t_lo) and (t_hi is None or tv <= t_hi)

    # ---------------- 1 パス目
    keys: dict[tuple, dict] = {}
    first_error_t: dict[str, float] = {}
    marks_all: list[tuple[str, str]] = []
    marks: list[tuple[str, str]] = []
    events = _Spool(tmpdir)
    n_lost_ev = n_back_ev = 0
    canstat_last: dict[str, str] = {}
    fol_series: dict[tuple, _FolSeries] = {}
    lost_tr = LostTracker()
    last_other: dict[tuple, int] = {}
    open_lost: dict[tuple, tuple] = {}       # key -> (t_rel 文字列, 範囲内か)
    run_lost: list[str] = []
    n_lost_in = 0
    n_data = 0
    skipped = 0
    unsorted = 0

    it = _iter_csv(path)
    idx = next(it, None)
    if idx is None:
        print(f"== report: {path}  (0 data rows, 0 marks)")
        return 0
    I = {k: idx.get(k) for k in CSV_FIELDS}
    ii, itr, im = I["iface"], I["t_rel"], I["motor"]
    ist, istn, itw = I["state_hex"], I["state_name"], I["t_wall"]
    ipos, itau, imos, irot = I["pos_rad"], I["tau_nm"], I["t_mos"], I["t_rotor"]

    for row in it:
        iface = row[ii]
        if iface in SPECIAL:
            try:
                tv = float(row[itr]) if row[itr] else 0.0
            except ValueError:
                skipped += 1
                continue
            text = row[im]
            if iface == "MARK":
                marks_all.append((row[itr], text))
                if inr(tv):
                    marks.append((row[itr], text))
            elif iface == "EVENT":
                head, _, rest = text.partition(" ")
                parts = rest.split(" ", 2)
                ekey = (parts[0], parts[1]) if len(parts) >= 2 else None
                inside = inr(tv)
                if inside:
                    events.add(f"   t_rel={row[itr]}s: {text}")
                    if text.startswith("LOST"):
                        n_lost_ev += 1
                    elif text.startswith("BACK"):
                        n_back_ev += 1
                if head == "LOST" and ekey is not None:
                    open_lost[ekey] = (row[itr], inside)
                    if inside:
                        n_lost_in += 1
                elif head == "BACK" and ekey is not None and ekey in open_lost:
                    t_l, was_in = open_lost.pop(ekey)
                    if was_in and lost_tr.is_running(ekey[0], last_other.get(ekey, 0)):
                        run_lost.append(f"{ekey[0]} {ekey[1]} t_rel={t_l}〜{row[itr]}s")
            else:  # CANSTAT
                if inr(tv):
                    name, _, rest = text.partition(" ")
                    canstat_last[name] = rest
            continue
        # データ行
        try:
            tv = float(row[itr]) if row[itr] else 0.0
            sh = row[ist]
            st = int(sh, 16) if sh else None
            tau = float(row[itau]) if row[itau] else 0.0
            r_s, m_s = row[irot], row[imos]
            r_f = float(r_s) if r_s != "" else None
            m_f = float(m_s) if m_s != "" else None
            pos = float(row[ipos]) if row[ipos] else 0.0
        except (ValueError, TypeError):
            skipped += 1
            continue
        key = (iface, row[im])
        last_other[key] = lost_tr.frame(*key)
        if not inr(tv):
            continue
        n_data += 1
        k = keys.get(key)
        if k is None:
            k = keys[key] = {"n": 0, "t0": tv, "t1": tv, "stc": {}, "stf": {}, "prev": None,
                             "lines": _Spool(tmpdir), "sx": 0, "sr": 0, "sm": 0, "nr": 0, "nm": 0,
                             "s2": 0, "rmax": None}
        elif tv < k["t1"]:
            unsorted += 1
        k["n"] += 1
        k["t1"] = tv
        if st is not None:
            k["stc"][st] = k["stc"].get(st, 0) + 1
            if st not in k["stf"]:
                k["stf"][st] = tv
            prev = k["prev"]
            if prev is None and st not in (0, 1):
                first_error_t[iface] = min(first_error_t.get(iface, tv), tv)
                k["lines"].add(f"   記録開始時点で既にラッチ: t_rel={tv:.2f}s wall={row[itw]} "
                               f"state={row[istn]} T_mos={m_s} T_rotor={r_s} "
                               f"(遷移は未記録・pre統計はスキップ)")
            elif prev is not None and prev in (0, 1) and st not in (0, 1):
                first_error_t[iface] = min(first_error_t.get(iface, tv), tv)
                k["lines"].add(f"   transition t_rel={tv:.2f}s wall={row[itw]} "
                               f"{STATE.get(prev, prev)}->{row[istn]} "
                               f"T_mos={m_s} T_rotor={r_s} tau={row[itau]}")
            k["prev"] = st
        # エラーの無い iface の「全区間」窓の和（旧版と同じ順に足す）
        k["sx"] += tv
        k["s2"] += tau * tau
        if r_f is not None:
            k["sr"] += r_f
            k["nr"] += 1
            k["rmax"] = r_f if k["rmax"] is None else max(k["rmax"], r_f)
        if m_f is not None:
            k["sm"] += m_f
            k["nm"] += 1
        if "fol" in iface:
            fs = fol_series.get(key)
            if fs is None:
                fs = fol_series[key] = _FolSeries(tmpdir)
            fs.add(tv, pos)
    for ekey, (t_l, was_in) in open_lost.items():  # 最後まで戻らなかった LOST
        if was_in and lost_tr.is_running(ekey[0], lost_tr.silence(*ekey)):
            run_lost.append(f"{ekey[0]} {ekey[1]} t_rel={t_l}s〜（戻らず）")
    for fs in fol_series.values():
        fs.finish()
    if unsorted:
        print(f"!! report: 同じモータの t_rel が逆行した行が {unsorted} 件（旧版は並べ替えてから解析していた。"
              f"この CSV は数値が旧版と少し違いうる）", file=sys.stderr)
    if skipped:
        print(f"!! report: 読めない行を {skipped} 件飛ばした（書きかけの末尾行など）", file=sys.stderr)

    print(f"== report: {path}  ({n_data} data rows, {len(marks)} marks)")
    if range_label is not None:
        print(f"== 範囲: {range_label}  t_rel={'-' if t_lo is None else f'{t_lo:.3f}'}"
              f"〜{'(末尾)' if t_hi is None else f'{t_hi:.3f}'}s（この区間の行だけを解析・テイク表はファイル全体から）")

    # (a) frames / rate / state分布 / transitions
    for key in sorted(keys):
        k = keys[key]
        iface, motor = key
        n = k["n"]
        rate = (n - 1) / (k["t1"] - k["t0"]) if n >= 2 and k["t1"] > k["t0"] else 0.0
        print(f"-- {iface} {motor}: frames={n} rate={rate:.1f}Hz")
        if k["stc"]:
            dist = ", ".join(f"{STATE.get(s, hex(s))}={k['stc'][s]}(first@{k['stf'][s]:.2f}s)"
                             for s in sorted(k["stc"]))
            print(f"   state分布: {dist}")
        for line in k["lines"]:
            print(line)
        k["lines"].close()

    # ---------------- 2 パス目（窓・冷却曲線・位置誤差）
    ifaces = sorted({kk[0] for kk in keys})
    pairs = []
    for i in ifaces:
        if "lead" in i:
            fol = i.replace("lead", "fol")
            if fol in ifaces:
                pairs.append((i, fol))
    pre_buf: dict[tuple, list] = {}
    win_buf: dict[tuple, list] = {}
    cool: dict[tuple, dict] = {}
    dev: dict[tuple, list] = {}        # エラー無し iface の「全区間」傾き: [num_r, num_m, den]
    means: dict[tuple, tuple] = {}
    for key, k in keys.items():
        if first_error_t.get(key[0]) is None:
            nx = k["n"]
            means[key] = (k["sx"] / nx, k["sr"] / nx, k["sm"] / nx)
            dev[key] = [0, 0, 0]
    err_st: dict[tuple, dict] = {}
    for lead_i, fol_i in pairs:
        t_err_p = first_error_t.get(fol_i, first_error_t.get(lead_i))
        for motor in sorted({m for (i, m) in keys if i in (lead_i, fol_i)}):
            lk, fk = (lead_i, motor), (fol_i, motor)
            if lk in keys and fk in fol_series:
                err_st[lk] = {"t0": keys[lk]["t0"], "t_err": t_err_p, "cur": fol_series[fk].cursor(),
                              "buf": [], "base": None, "n": 0, "sum": 0, "max": None, "any": False}

    def err_take(s: dict, t: float, e: float) -> None:
        t_err = s["t_err"]
        if t_err is not None and not (t_err - pre_s <= t < t_err):
            return
        v = abs(e - s["base"])
        s["n"] += 1
        s["sum"] += v
        s["max"] = v if s["max"] is None else max(s["max"], v)

    def err_feed(s: dict, t: float, e: float) -> None:
        s["any"] = True
        if s["base"] is None:
            if t - s["t0"] <= 5.0:
                s["buf"].append((t, e))
                return
            err_fix(s)
        err_take(s, t, e)

    def err_fix(s: dict) -> None:
        if s["base"] is not None:
            return
        vals = [e for (t, e) in s["buf"] if t - s["t0"] <= 5.0]
        s["base"] = statistics.median(vals) if vals else 0.0
        for t, e in s["buf"]:
            err_take(s, t, e)
        s["buf"] = []

    need_pass2 = bool(keys)
    if need_pass2:
        it = _iter_csv(path)
        next(it, None)
        for row in it:
            iface = row[ii]
            if iface in SPECIAL:
                continue
            key = (iface, row[im])
            k = keys.get(key)
            if k is None:
                continue
            try:
                tv = float(row[itr]) if row[itr] else 0.0
                if row[ist]:
                    int(row[ist], 16)
                tau = float(row[itau]) if row[itau] else 0.0
                r_s, m_s = row[irot], row[imos]
                r_f = float(r_s) if r_s != "" else None
                m_f = float(m_s) if m_s != "" else None
                pos = float(row[ipos]) if row[ipos] else 0.0
            except (ValueError, TypeError):
                continue
            if not inr(tv):
                continue
            tw = tv or -1e9   # 旧版の (_num(r, "t_rel") or -1e9)：t_rel=0.0 の行は窓に入れない
            t_err = first_error_t.get(iface)
            if t_err is not None:
                if t_err - pre_s <= tw < t_err:
                    pre_buf.setdefault(key, []).append((tv, tau, r_s))
                if max(0.0, t_err - window_s) <= tw < t_err:
                    win_buf.setdefault(key, []).append((tv, tau, r_f, m_f))
                if r_s != "" and tw >= t_err:
                    c = cool.setdefault(key, {})
                    for step in range(0, 61, 10):
                        if tv <= t_err + step:
                            c[step] = r_f
            else:
                mx, mr, mm = means[key]
                d = dev[key]
                if r_f is not None:
                    d[0] += (tv - mx) * (r_f - mr)
                if m_f is not None:
                    d[1] += (tv - mx) * (m_f - mm)
                d[2] += (tv - mx) ** 2
            s = err_st.get(key)
            if s is not None:
                t_e = s["t_err"]
                if t_e is not None:
                    t0 = s["t0"]
                    if not ((t0 <= tw <= t0 + 5.0) or (t_e - pre_s <= tw < t_e)):
                        continue
                fp = s["cur"].nearest(tv)
                if fp is not None:
                    err_feed(s, tv, pos - fp)

    # (b)+(c)+(d) per-iface
    for iface in ifaces:
        t_err = first_error_t.get(iface)
        motors = sorted({kk[1] for kk in keys if kk[0] == iface})
        pre_label = f"最初のエラー前 {pre_s:g}s" if t_err is not None else "(エラー無し: pre統計スキップ)"
        print(f"== {iface}: {pre_label}")
        if t_err is not None:
            for motor in motors:
                rows_ = pre_buf.get((iface, motor))
                if not rows_:
                    continue
                taus = [x[1] for x in rows_]
                ts = [x[0] for x in rows_]
                trotors = [int(x[2]) for x in rows_ if x[2] != ""]
                tau_peak = max((abs(x) for x in taus), default=0.0)
                tau_rms = (sum(x * x for x in taus) / len(taus)) ** 0.5 if taus else 0.0
                i2t = _trapz_sq(ts, taus)
                print(f"   {motor}: |tau|peak={tau_peak:.2f}Nm tau_rms={tau_rms:.2f}Nm "
                      f"T_rotor[min-max]={min(trotors) if trotors else '-'}-{max(trotors) if trotors else '-'}C "
                      f"I2t~{i2t:.2f}")

        print(f"== {iface}: 全区間 or 最初のエラー前 {window_s:g}s")
        for motor in motors:
            key = (iface, motor)
            if t_err is not None:
                rows_ = win_buf.get(key)
                if not rows_:
                    continue
                taus = [x[1] for x in rows_]
                ts = [x[0] for x in rows_]
                trotors = [x[2] for x in rows_ if x[2] is not None]
                tmoss = [x[3] for x in rows_ if x[3] is not None]
                tau_rms = (sum(x * x for x in taus) / len(taus)) ** 0.5 if taus else 0.0
                slope_rotor = _lstsq_slope(ts, trotors) * 60.0 if trotors else 0.0
                slope_mos = _lstsq_slope(ts, tmoss) * 60.0 if tmoss else 0.0
                rmax = max(trotors) if trotors else "-"
            else:
                k = keys[key]
                n = k["n"]
                num_r, num_m, den = dev[key]
                tau_rms = (k["s2"] / n) ** 0.5
                slope_rotor = ((num_r / den if den else 0.0) * 60.0 if n >= 2 else 0.0) if k["nr"] else 0.0
                slope_mos = ((num_m / den if den else 0.0) * 60.0 if n >= 2 else 0.0) if k["nm"] else 0.0
                rmax = k["rmax"] if k["rmax"] is not None else "-"
            print(f"   {motor}: tau_rms={tau_rms:.2f}Nm T_rotor_slope={slope_rotor:.2f}C/min "
                  f"T_mos_slope={slope_mos:.2f}C/min T_rotor_max={rmax}C")

        if t_err is not None:
            print(f"== {iface}: エラー後の T_rotor 冷却曲線（10秒刻み・60秒。測定値か推定値かの判別材料）")
            for motor in motors:
                c = cool.get((iface, motor))
                if c is None:
                    continue
                vals = [f"{step}s={c[step]:.0f}C" if step in c else f"{step}s=-" for step in range(0, 61, 10)]
                print(f"   {motor}: {' '.join(vals)}")

    # (e) leader/follower pairing
    for lead_i, fol_i in pairs:
        print(f"== 位置誤差 e(t)=pos_lead-pos_fol: {lead_i} vs {fol_i} "
              f"(基準: 最初の5秒の中央値を較正オフセット差として近似的に減算)")
        for motor in sorted({m for (i, m) in keys if i in (lead_i, fol_i)}):
            s = err_st.get((lead_i, motor))
            if s is None or not s["any"]:
                continue
            err_fix(s)
            if not s["n"]:
                continue
            kp = FOLLOWER_KP.get(motor, 0)
            emax, esum, en = s["max"], s["sum"], s["n"]
            print(f"   {motor}: |e|max={emax:.3f}rad |e|mean={esum/en:.3f}rad "
                  f"kp*|e|max~{kp*emax:.2f}Nm kp*|e|mean~{kp*esum/en:.2f}Nm")

    if events.n or canstat_last:
        print(f"== EVENT: LOST {n_lost_ev} 回 / BACK {n_back_ev} 回")
        # 2026-10-05 追加: 束（起動の整列・保存・停止でバス全体が黙る間）と動作中の LOST を分けて数える（LostTracker）
        if n_lost_ev:
            print(f"   うち動作中の LOST {len(run_lost)} 回（同じバスの他モータが流れ続けた間の無応答）・"
                  f"束 {n_lost_in - len(run_lost)} 回（起動・保存・停止でバス全体が黙る間の見かけ上の LOST）")
            for x in run_lost[:20]:
                print(f"   動作中: {x}")
            if len(run_lost) > 20:
                print(f"   動作中: ...ほか {len(run_lost) - 20} 件")
        for line in events:
            print(line)
        for name, rest in sorted(canstat_last.items()):
            print(f"   canstat(最後) {name}: {rest or 'n/a'}")
    events.close()

    if marks or (range_label is not None and marks_all):
        tbl = (episode_table(marks_all, t_range=(t_lo, t_hi)) if range_label is not None
               else episode_table(marks))
        if tbl:
            print("== エピソード（MARK から）: " + tbl[0])
            for line in tbl[1:]:
                print(line)
        print("== MARK")
        for t, text in marks:
            print(f"   t_rel={t}s: {text}")

    return 0


# ------------------------------------------------------------------ live --

def _short(motor: str) -> str:
    return "grip" if motor == "gripper" else motor.replace("joint_", "j")


def _live_baseline(path: str, max_rows: int = 600_000) -> tuple[dict, bool]:
    """位置誤差の基準（--report と同じ定義: lead の最初の 5 秒の e の中央値）を CSV の先頭だけ読んで作る。

    --live は末尾から読むので、先頭の数十秒（最初の受信 +30 秒・最大 max_rows 行）を別に読む。最近傍の
    follower 点は lead の基準窓の前後 2 秒（と窓の外の最寄り 1 点ずつ）から探す。
    戻り値の 2 つ目は「基準が確定したか」（先頭が 30 秒に満たない記録の始めは False。--live が後で読み直す）。
    """
    def head_rows(limit):
        it = _iter_csv(path)
        idx = next(it, None)
        if idx is None:
            return
        ii, itr, im, ipos = idx["iface"], idx["t_rel"], idx["motor"], idx["pos_rad"]
        for n, row in enumerate(it):
            if n >= limit:
                return
            if row[ii] in ("MARK", "EVENT", "CANSTAT"):
                continue
            try:
                tv = float(row[itr]) if row[itr] else 0.0
                pos = float(row[ipos]) if row[ipos] else 0.0
            except ValueError:
                continue
            yield n, (row[ii], row[im]), tv, pos

    first: dict[tuple, float] = {}
    t_any = None
    n_used = 0
    done = False
    for n, key, tv, _pos in head_rows(max_rows):
        n_used = n + 1
        if t_any is None:
            t_any = tv
        first.setdefault(key, tv)
        if tv > t_any + 30.0 and tv > max(first.values()) + 6.0:
            done = True
            break
    done = done or n_used >= max_rows
    pairs = {}
    for (i, m), t0 in first.items():
        if "lead" in i and (i.replace("lead", "fol"), m) in first:
            pairs[(i, m)] = ((i.replace("lead", "fol"), m), t0)
    if not pairs:
        return {}, done
    lead_s: dict[tuple, list] = {k: [] for k in pairs}
    fol_win = {fk: (t0 - 2.0, t0 + 7.0) for _lk, (fk, t0) in pairs.items()}
    fol_s: dict[tuple, list] = {fk: [] for fk in fol_win}
    fol_before: dict[tuple, tuple] = {}
    fol_after: dict[tuple, tuple] = {}
    for _n, key, tv, pos in head_rows(n_used):
        if key in lead_s and tv - pairs[key][1] <= 5.0:
            lead_s[key].append((tv, pos))
        if key in fol_win:
            lo, hi = fol_win[key]
            if tv < lo:
                fol_before[key] = (tv, pos)
            elif tv <= hi:
                fol_s[key].append((tv, pos))
            elif key not in fol_after:
                fol_after[key] = (tv, pos)
    out = {}
    for lk, (fk, _t0) in pairs.items():
        pts = ([fol_before[fk]] if fk in fol_before else []) + fol_s[fk] + ([fol_after[fk]] if fk in fol_after else [])
        if not pts or not lead_s[lk]:
            continue
        times = [p[0] for p in pts]
        errs = []
        for t, pos in lead_s[lk]:
            i = bisect.bisect_left(times, t)
            cands = ([pts[i]] if i < len(pts) else []) + ([pts[i - 1]] if i > 0 else [])
            errs.append(pos - min(cands, key=lambda tv: abs(tv[0] - t))[1])
        out[lk] = statistics.median(errs)
    return out, done


class LiveView:
    """--live の集計（2026-10-05）。行を feed() で入れ、render() で直近の窓の要約を返す（ファイルを知らない）。"""

    def __init__(self, window_s: float, baseline: dict | None = None) -> None:
        import collections
        self._deque = collections.deque
        self.window = window_s
        self.data: dict[tuple, "collections.deque"] = {}
        self.events = collections.deque()          # (t, text)
        self.run_lost = collections.deque()        # (t, text)
        self.marks = collections.deque(maxlen=3)   # (t, text)
        self.marks_win = collections.deque()       # t
        self.lost_tr = LostTracker()
        self.last_other: dict[tuple, int] = {}
        self.open_lost: dict[tuple, float] = {}
        self.baseline = baseline or {}
        self.t_now: float | None = None
        self.n_rows = 0

    def feed(self, iface: str, motor: str, t_rel: str, state_hex: str, state_name: str, pos: str,
             t_rotor: str) -> None:
        try:
            tv = float(t_rel) if t_rel else 0.0
        except ValueError:
            return
        self.n_rows += 1
        if iface == "MARK":
            self.marks.append((tv, motor))
            self.marks_win.append(tv)
        elif iface == "EVENT":
            head, _, rest = motor.partition(" ")
            parts = rest.split(" ", 2)
            if len(parts) < 2 or head not in ("LOST", "BACK"):
                return
            key = (parts[0], parts[1])
            self.events.append((tv, head))
            if head == "LOST":
                self.open_lost[key] = tv
            elif key in self.open_lost:
                t_l = self.open_lost.pop(key)
                if self.lost_tr.is_running(key[0], self.last_other.get(key, 0)):
                    self.run_lost.append((tv, f"{key[0]} {_short(key[1])} {t_l:.1f}〜{tv:.1f}s"))
        elif iface != "CANSTAT":
            try:
                st = int(state_hex, 16) if state_hex else None
                r = float(t_rotor) if t_rotor != "" else None
                p = float(pos) if pos else 0.0
            except ValueError:
                return
            key = (iface, motor)
            self.last_other[key] = self.lost_tr.frame(iface, motor)
            dq = self.data.get(key)
            if dq is None:
                dq = self.data[key] = self._deque()
            dq.append((tv, r, st, p, state_name))
            while dq and dq[0][0] < tv - self.window:
                dq.popleft()
            if self.t_now is None or tv > self.t_now:
                self.t_now = tv

    def _trim(self) -> None:
        if self.t_now is None:
            return
        lo = self.t_now - self.window
        for dq in (self.events, self.run_lost):
            while dq and dq[0][0] < lo:
                dq.popleft()
        while self.marks_win and self.marks_win[0] < lo:
            self.marks_win.popleft()
        for dq in self.data.values():
            while dq and dq[0][0] < lo:
                dq.popleft()

    def render(self, header: str = "") -> list[str]:
        self._trim()
        out = [header] if header else []
        if self.t_now is None:
            out.append("  (まだモータの受信行が無い)")
            return out
        ifaces = sorted({k[0] for k in self.data})
        for iface in ifaces:
            cells = []
            worst = None
            for motor in sorted(m for (i, m) in self.data if i == iface):
                dq = self.data[(iface, motor)]
                pts = [(x[0], x[1]) for x in dq if x[1] is not None]
                if not pts:
                    continue
                tmax = max(v for _t, v in pts)
                slope = _lstsq_slope([t for t, _v in pts], [v for _t, v in pts]) * 60.0
                cells.append(f"{_short(motor)} {tmax:.0f}C{slope:+.1f}")
                if worst is None or tmax > worst[0]:
                    worst = (tmax, motor)
            if cells:
                out.append(f"  {iface}: 最大 {worst[0]:.0f}C@{_short(worst[1])}  | " + " ".join(cells)
                           + "  (T_rotor 窓内最大 C・傾き C/min)")
        n_l = sum(1 for _t, h in self.events if h == "LOST")
        n_b = sum(1 for _t, h in self.events if h == "BACK")
        line = f"  LOST {n_l} / BACK {n_b}（窓内）・うち動作中の LOST {len(self.run_lost)}"
        if self.run_lost:
            line += "  " + " / ".join(x for _t, x in list(self.run_lost)[-3:])
        if self.open_lost:
            line += "  いま無応答: " + ", ".join(f"{i} {_short(m)}" for (i, m) in sorted(self.open_lost))
        out.append(line)
        bad = []
        for key in sorted(self.data):
            dq = self.data[key]
            names: dict[str, int] = {}
            for x in dq:
                if x[2] is not None and x[2] not in (0, 1):
                    nm = x[4] or STATE.get(x[2], f"code_{x[2]:X}")
                    names[nm] = names.get(nm, 0) + 1
            for nm, c in names.items():
                bad.append(f"{key[0]} {_short(key[1])} {nm}x{c}")
        out.append("  エラー状態（窓内）: " + (" / ".join(bad) if bad else "なし"))
        for lead_i in ifaces:
            if "lead" not in lead_i:
                continue
            fol_i = lead_i.replace("lead", "fol")
            if fol_i not in ifaces:
                continue
            cells = []
            for motor in sorted(m for (i, m) in self.data if i == lead_i):
                fdq = self.data.get((fol_i, motor))
                ldq = self.data[(lead_i, motor)]
                if not fdq or not ldq:
                    continue
                ft = [x[0] for x in fdq]
                fp = [x[3] for x in fdq]
                base = self.baseline.get((lead_i, motor))
                emax = 0.0
                for x in ldq:
                    i = bisect.bisect_left(ft, x[0])
                    cands = ([i] if i < len(ft) else []) + ([i - 1] if i > 0 else [])
                    j = min(cands, key=lambda c: abs(ft[c] - x[0]))
                    emax = max(emax, abs(x[3] - fp[j] - (base or 0.0)))
                cells.append(f"{_short(motor)} {emax:.3f}" + ("" if base is not None else "*"))
            if cells:
                out.append(f"  |e|max {lead_i}-{fol_i}: " + " ".join(cells)
                           + " rad（基準=最初の5秒の中央値・*は基準なし）")
        if self.marks:
            t, text = self.marks[-1]
            out.append(f"  直近の MARK: t_rel={t:.1f}s {text[:120]}（窓内 {len(self.marks_win)} 件）")
        else:
            out.append("  直近の MARK: なし")
        return out


def run_live(path: str, every: float, window_s: float, from_start: bool) -> int:
    """記録中の CSV を tail で追い、every 秒ごとに直近 window_s 秒の要約を出す（CAN には触らない・読むだけ）。"""
    try:
        return _run_live(path, every, window_s, from_start)
    except (KeyboardInterrupt, BrokenPipeError):
        try:
            print("\n[live] 終了")
        except (BrokenPipeError, OSError):
            pass
        return 0


def _run_live(path: str, every: float, window_s: float, from_start: bool) -> int:
    while True:  # ヘッダが書かれるまで待つ（ロガーの起動直後）
        it = _iter_csv(path) if os.path.exists(path) else iter(())
        idx = next(it, None)
        it = None
        if idx is not None:
            break
        time.sleep(0.5)
    cols = [idx[k] for k in ("iface", "motor", "t_rel", "state_hex", "state_name", "pos_rad", "t_rotor")]
    nmax = max(cols)
    base, base_done = _live_baseline(path)
    view = LiveView(window_s, base)
    fh = open(path, "rb")
    drop_first = False
    if not from_start:
        size = os.fstat(fh.fileno()).st_size
        if size:
            fh.seek(size - 1)
            drop_first = fh.read(1) != b"\n"  # 書きかけの行の途中から始まるので、最初の改行までは捨てる
    print(f"[live] {path} を{'先頭' if from_start else '末尾'}から追う（{every:g} 秒ごと・窓 {window_s:g} 秒・"
          f"CAN には触らない・Ctrl-C で終わる）")
    buf = b""
    last_render = time.monotonic()
    rows_at_render = 0
    idle_since = time.monotonic()
    while True:
        chunk = fh.read()
        if chunk:
            idle_since = time.monotonic()
            buf += chunk
            lines = buf.split(b"\n")
            buf = lines.pop()
            if drop_first and lines:
                lines = lines[1:]
                drop_first = False
            text = [ln.decode("utf-8", errors="replace") for ln in lines]
            for row in csv.reader(x for x in text if x and not x.startswith("#") and not x.startswith("t_wall,")):
                if len(row) <= nmax:
                    continue
                view.feed(*(row[c] for c in cols))
        else:
            try:
                if os.stat(path).st_size < fh.tell():  # 切り詰め・作り直し → 頭から
                    fh.close()
                    fh = open(path, "rb")
                    buf = b""
                    drop_first = False
            except OSError:
                pass
        now = time.monotonic()
        if now - last_render >= every:
            if not base_done:  # 記録の始め（30 秒未満）から見ているときは、基準が確定するまで先頭を読み直す
                base, base_done = _live_baseline(path)
                view.baseline = base
            rate = (view.n_rows - rows_at_render) / max(now - last_render, 1e-6)
            try:
                mb = os.stat(path).st_size / 1048576
            except OSError:
                mb = 0.0
            hdr = (f"[live {time.strftime('%H:%M:%S')}] t_rel={view.t_now if view.t_now is not None else 0:.1f}s "
                   f"窓 {window_s:g}s  {rate:.0f} 行/s  {os.path.basename(path)} {mb:.0f}MB")
            lines_out = view.render(hdr)
            if now - idle_since > max(every, 3.0):
                lines_out.append(f"  (追記なし {now - idle_since:.0f}s: 記録が止まったか、まだ受信していない)")
            print("\n".join(lines_out), flush=True)
            last_render = now
            rows_at_render = view.n_rows
        time.sleep(0.2)


# -------------------------------------------------------------- selftest --

def run_selftest() -> int:
    ok = True

    # 1) decode/encode 往復（量子化誤差以内で一致）
    cases = [
        ("joint_1", 1, 3.0, 10.0, 5.0, 25, 30),
        ("joint_3", 1, -2.0, -3.0, 4.0, 20, 22),
        ("joint_7", 0xC, 1.0, 2.0, -3.0, 45, 40),
        ("gripper", 0, 0.0, 0.0, 0.0, 22, 22),
    ]
    for motor, state, pos, vel, tau, tmos, trotor in cases:
        raw = encode_for_selftest(state, 0x1, pos, vel, tau, tmos, trotor, motor)
        dec = decode(raw, motor)
        _mtype, pmax, vmax, tmax = MTYPE[motor]
        tol_pos = pmax * 2 / 65535 * 1.5
        tol_vel = vmax * 2 / 4095 * 1.5
        tol_tau = tmax * 2 / 4095 * 1.5
        if dec["state"] != state:
            print(f"FAIL state {motor}: {dec['state']:X} != {state:X}"); ok = False
        if abs(dec["pos_rad"] - pos) > tol_pos:
            print(f"FAIL pos {motor}: {dec['pos_rad']} != {pos}"); ok = False
        if abs(dec["vel_rad_s"] - vel) > tol_vel:
            print(f"FAIL vel {motor}: {dec['vel_rad_s']} != {vel}"); ok = False
        if abs(dec["tau_nm"] - tau) > tol_tau:
            print(f"FAIL tau {motor}: {dec['tau_nm']} != {tau}"); ok = False
        if dec["t_mos"] != tmos or dec["t_rotor"] != trotor:
            print(f"FAIL temps {motor}: {dec['t_mos']}/{dec['t_rotor']} != {tmos}/{trotor}"); ok = False
    print("decode/encode roundtrip: " + ("PASS" if ok else "FAIL"))

    # 2) 合成 CSV を作って --report を通す（1回のエラー遷移を含む）
    import io
    import os
    import tempfile

    rows = []
    motor = "joint_7"
    mtype = MTYPE[motor][0]
    # 0-5s: enabled, tau small, T_rotor slowly rising, pos_lead/fol tracking closely -> ここでは fol データのみ
    for i in range(50):
        t = i * 0.1
        rows.append({"t_wall": f"{1758000000+t:.6f}", "t_rel": f"{t:.6f}", "iface": "can_fol_r", "motor": motor,
                     "mtype": mtype, "state_hex": "1", "state_name": "enabled", "pos_rad": f"{0.01*i:.5f}",
                     "vel_rad_s": "0.1", "tau_nm": "0.5", "t_mos": str(25 + i // 10), "t_rotor": str(30 + i // 5),
                     "raw_hex": ""})
    # 5-6s: エラー遷移（0xC）・トルク飽和
    for i in range(10):
        t = 5.0 + i * 0.1
        rows.append({"t_wall": f"{1758000000+t:.6f}", "t_rel": f"{t:.6f}", "iface": "can_fol_r", "motor": motor,
                     "mtype": mtype, "state_hex": "C", "state_name": "COIL_OVERTEMP", "pos_rad": "0.6",
                     "vel_rad_s": "0.0", "tau_nm": "10.0", "t_mos": "48", "t_rotor": "42", "raw_hex": ""})
    # 6-70s: 冷却
    for i in range(64):
        t = 6.0 + i
        val = max(42 - i, 20)
        rows.append({"t_wall": f"{1758000000+t:.6f}", "t_rel": f"{t:.6f}", "iface": "can_fol_r", "motor": motor,
                     "mtype": mtype, "state_hex": "C", "state_name": "COIL_OVERTEMP", "pos_rad": "0.6",
                     "vel_rad_s": "0.0", "tau_nm": "0.0", "t_mos": "30", "t_rotor": str(val), "raw_hex": ""})
    # leader 対応バス（大きめの位置差を pre-error 区間に作る）
    for i in range(50):
        t = i * 0.1
        rows.append({"t_wall": f"{1758000000+t:.6f}", "t_rel": f"{t:.6f}", "iface": "can_lead_r", "motor": motor,
                     "mtype": mtype, "state_hex": "1", "state_name": "enabled",
                     "pos_rad": f"{0.01*i + (0.4 if t > 3.0 else 0.0):.5f}",
                     "vel_rad_s": "0.1", "tau_nm": "0.0", "t_mos": "25", "t_rotor": "26", "raw_hex": ""})

    # 反証レビュー指摘(a)の回帰用: 記録開始時点で既に COIL_OVERTEMP でラッチしているモータ
    # （2026-09-21 実機 errcheck-latched と同型・遷移行は一度も出ない）。
    # 別 iface（can_fol_l）に置く: first_error_t は iface 単位なので、can_fol_r/joint_7 の
    # 遷移（t_rel=5.0s）と同じ iface に混ぜると t_err が上書きされ既存テストの pre/I2t/位置誤差
    # 検証が壊れる（iface 単位の仕様自体は今回の2件の指摘の対象外なので変更しない）。
    latched_motor = "joint_5"
    latched_mtype = MTYPE[latched_motor][0]
    for i in range(30):
        t = i * 0.1
        val = max(50 - i, 30)
        rows.append({"t_wall": f"{1758000000+t:.6f}", "t_rel": f"{t:.6f}", "iface": "can_fol_l",
                     "motor": latched_motor, "mtype": latched_mtype, "state_hex": "C",
                     "state_name": "COIL_OVERTEMP", "pos_rad": "0.0", "vel_rad_s": "0.0", "tau_nm": "0.0",
                     "t_mos": "35", "t_rotor": str(val), "raw_hex": ""})

    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    try:
        with open(path, "w", newline="") as f:
            f.write("# start_wall=2026-09-21T00:00:00+00:00 host=selftest ifaces=can_fol_r,can_lead_r\n")
            w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            w.writeheader()
            for r in rows:
                w.writerow(r)
            w.writerow({"t_wall": "1758000005.500000", "t_rel": "5.5", "iface": "MARK", "motor": "squeeze start",
                        "mtype": "", "state_hex": "", "state_name": "", "pos_rad": "", "vel_rad_s": "", "tau_nm": "",
                        "t_mos": "", "t_rotor": "", "raw_hex": ""})

        buf = io.StringIO()
        old_stdout = sys.stdout
        sys.stdout = buf
        try:
            rc = run_report(path, pre_s=5.0, window_s=30.0)
        finally:
            sys.stdout = old_stdout
        out = buf.getvalue()
        print(out)
        needed = ["transition", "I2t", "T_rotor_slope", "冷却曲線", "|e|max", "MARK",
                   "state分布", "記録開始時点で既にラッチ"]
        for token in needed:
            if token not in out:
                print(f"FAIL report missing token: {token}"); ok = False
        if f"-- can_fol_l {latched_motor}" not in out:
            print(f"FAIL report missing {latched_motor} section"); ok = False
        if rc != 0:
            print("FAIL report returned nonzero"); ok = False
    finally:
        os.remove(path)

    print("report synthetic test: " + ("PASS" if ok else "FAIL"))

    # 3) --follow: console.log の追記からエピソード境界の MARK を作る（2026-10-05）
    ok_follow = _selftest_follow()
    ok = ok and ok_follow
    print("follow/episode-mark test: " + ("PASS" if ok_follow else "FAIL"))
    # 4) 2026-10-05: LOST の分類・切り出し・判定材料の行・live の要約・溜め置き
    for name, fn in (("lost-classify", _selftest_lost), ("episode-slice", _selftest_slice),
                     ("anomaly-line", _selftest_anomaly_line), ("live-view", _selftest_live),
                     ("spool", _selftest_spool)):
        try:
            r = fn()
        except Exception as exc:  # noqa: BLE001
            print(f"FAIL {name}: {exc!r}")
            r = False
        print(f"{name} test: " + ("PASS" if r else "FAIL"))
        ok = ok and r
    print("SELFTEST " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def _selftest_follow() -> bool:
    """既存行は拾わない・途中行の持ち越し・\r 区切り・後から現れるファイル・保存の推定、を確かめる。"""
    import tempfile

    ok = True
    with tempfile.TemporaryDirectory() as td:
        console = os.path.join(td, "console.log")
        marks_path = os.path.join(td, "x.marks")
        with open(console, "w") as fc:
            fc.write("INFO 2026-10-05 10:00:00 ls/utils.py:143 Recording episode 99\n")  # 起動前の行（拾わない）
        q: "queue.Queue" = queue.Queue()
        stop = threading.Event()
        th = threading.Thread(target=_follower, args=([console, marks_path], q, stop, time.monotonic(), 0.02),
                              daemon=True)
        th.start()
        time.sleep(0.1)
        chunks = [
            "INFO 2026-10-05 10:00:01 t_record.py:346 Episode 0 start pose: worst left_joint_1.pos=1.3deg "
            "(tolerance 3.0deg, outcome=started).\n",
            "INFO 2026-10-05 10:00:01 ls/utils.py:143 Recording episode 0\n",
            "Right arrow key pressed. Exiting loop...\n",
            "INFO 2026-10-05 10:00:20 ol_utils.py:784 Waiting: press 'a' to return both arms ...\n",
            "INFO 2026-10-05 10:00:35 ol_utils.py:784 Waiting: press 'a' to return both arms ...\n",
            "'a' key pressed. Aligning the arm to the start pose...\n",
            "INFO 2026-10-05 10:00:40 ls/utils.py:143 Arm returned. Restore the scene, then press the right arrow\n",
            "Right arrow key pressed. Exiting loop...\n",
            "INFO 2026-10-05 10:00:41 ls/utils.py:143 Recording episode 1\n",
            "Left arrow key pressed. Exiting loop and rerecord the last episode...\n",
            "INFO 2026-10-05 10:00:45 ls/utils.py:143 Re-record episode\n",
            "INFO 2026-10-05 10:00:50 ls/utils.py:143 Record",  # 途中行（次の書き込みで完成）
            "ing episode 1\n",
            "Map:   0%|          | 0/1 [00:00<?, ?it/s]\rEscape key pressed. Stopping data recording...\n",
            "INFO 2026-10-05 10:01:10 ls/utils.py:143 Stop recording\n",
            "INFO 2026-10-05 10:01:14 ls/utils.py:143 Exiting\n",
        ]
        for c in chunks:
            with open(console, "a") as fc:
                fc.write(c)
            time.sleep(0.04)
        with open(marks_path, "a") as fm:  # 起動後に現れたファイルは先頭から読む
            fm.write("MARK EXIT rc=0\n")
        time.sleep(0.1)
        stop.set()
        th.join(timeout=2.0)
    got = []
    while not q.empty():
        got.append(q.get()["motor"])
    expect_prefix = [
        "START_POSE ep=0", "EP_START ep=0 take=1", "EP_END ep=0 take=1 key=right", "RETURN ep=0",
        "RETURN_DONE", "CUE key=right", "EP_START ep=1 take=2", "EP_END ep=1 take=2 key=left",
        "EP_DISCARD ep=1 take=2 reason=rerecord", "EP_START ep=1 take=3", "EP_END ep=1 take=3 key=esc",
        "STOP_REQUEST key=esc", "EP_STOP", "LEROBOT_EXITING", "EXIT rc=0",
    ]
    if len(got) != len(expect_prefix) or any(not g.startswith(e) for g, e in zip(got, expect_prefix)):
        print("FAIL follow marks:")
        for g in got:
            print("   got: " + g)
        ok = False
    tbl = episode_table([(f"{i}.0", g) for i, g in enumerate(got)])
    want = ["→ 保存", "→ 破棄(rerecord)", "→ 保存(推定)"]
    if len(tbl) != 4 or any(not tbl[i + 1].endswith(w) for i, w in enumerate(want)):
        print("FAIL episode table:")
        for line in tbl:
            print("   " + line)
        ok = False
    return ok


def _selftest_rows_bus(t0: float, t1: float, hz: float, iface: str, motors: list[str], drop: dict | None = None,
                       state: dict | None = None, temp: dict | None = None) -> list[dict]:
    """合成のモータ行。drop={motor: (lo, hi)} の間はそのモータを出さない。state/temp は motor -> f(t)。"""
    rows = []
    n = int(round((t1 - t0) * hz))
    for i in range(n):
        t = t0 + i / hz
        for m in motors:
            if drop and m in drop and drop[m][0] <= t < drop[m][1]:
                continue
            st = state[m](t) if state and m in state else 1
            tr = temp[m](t) if temp and m in temp else 30
            rows.append({"t_wall": f"{1758000000 + t:.6f}", "t_rel": f"{t:.6f}", "iface": iface, "motor": m,
                         "mtype": MTYPE[m][0], "state_hex": f"{st:X}", "state_name": STATE.get(st, f"code_{st:X}"),
                         "pos_rad": f"{0.1 * t:.5f}", "vel_rad_s": "0.0", "tau_nm": "0.5", "t_mos": "30",
                         "t_rotor": str(tr), "raw_hex": ""})
    return rows


def _selftest_lost() -> bool:
    """束（handshake 後のバス全体の沈黙）は動作中と数えず、他モータが流れる間の無応答は数える。"""
    motors = list(NAMES.values())
    lt = LostTracker()
    # 起動: handshake で 1 回ずつ応答 → 2.8 秒沈黙 → ループ（200Hz）
    for m in motors:
        lt.frame("b", m)
    other_burst = {}
    for m in motors:  # ループ開始の 1 周目で戻る
        other_burst[m] = lt.frame("b", m)
    burst_ok = all(not lt.is_running("b", v) for v in other_burst.values())
    # 動作中: J5〜gripper が 0.5 秒落ちる（他 4 台は 200Hz で流れ続ける）
    for _ in range(100):
        for m in motors[:4]:
            lt.frame("b", m)
    run_ok = all(lt.is_running("b", lt.frame("b", m)) for m in motors[4:])
    # 30Hz の記録でも 0.3 秒で数える（7 台 x 9 周 = 63 >= 35）
    lt2 = LostTracker()
    for m in motors:
        lt2.frame("c", m)
    for _ in range(9):
        for m in motors[:-1]:
            lt2.frame("c", m)
    rec_ok = lt2.is_running("c", lt2.frame("c", motors[-1]))
    if not (burst_ok and run_ok and rec_ok):
        print(f"FAIL lost-classify burst={burst_ok} run={run_ok} rec30={rec_ok}")
    return burst_ok and run_ok and rec_ok


def _selftest_write_csv(path: str, rows: list[dict]) -> None:
    with open(path, "w", newline="") as f:
        f.write("# start_wall=2026-10-05T00:00:00+00:00 host=selftest ifaces=can_fol_r\n")
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        for r in sorted(rows, key=lambda r: float(r["t_rel"])):
            w.writerow(r)


def _selftest_mark(t: float, text: str, iface: str = "MARK") -> dict:
    r = {k: "" for k in CSV_FIELDS}
    r.update({"t_wall": f"{1758000000 + t:.6f}", "t_rel": f"{t:.6f}", "iface": iface, "motor": text})
    return r


def _selftest_slice() -> bool:
    """--episode / --take / --since の範囲と、範囲内だけを数えること。"""
    import io
    import tempfile

    rows = _selftest_rows_bus(0.0, 30.0, 10.0, "can_fol_r", ["joint_1"])
    for t, text in ((1.0, "EP_START ep=0 take=1"), (5.0, "EP_END ep=0 take=1 key=right"),
                    (10.0, "EP_START ep=1 take=2"), (12.0, "EP_END ep=1 take=2 key=left"),
                    (12.5, "EP_DISCARD ep=1 take=2 reason=rerecord"), (20.0, "EP_START ep=1 take=3"),
                    (25.0, "EP_END ep=1 take=3 key=esc"), (26.0, "EXIT rc=0")):
        rows.append(_selftest_mark(t, text))
    ok = True
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, "s.csv")
        _selftest_write_csv(path, rows)
        marks = scan_marks(path)
        cases = [((1, None), (20.0, 25.0)), ((1, 2), (10.0, 12.0)), ((0, None), (1.0, 5.0)), ((None, 3), (20.0, 25.0))]
        for (ep, tk), want in cases:
            got = episode_range(marks, ep, tk)
            if got is None or (got[0], got[1]) != want:
                print(f"FAIL slice ep={ep} take={tk}: {got}"); ok = False
        if episode_range(marks, 7, None) is not None:
            print("FAIL slice: ep=7 should be missing"); ok = False
        buf = io.StringIO()
        old = sys.stdout
        sys.stdout = buf
        try:
            run_report(path, 5.0, 30.0, 20.0, 25.0, "ep=1")
        finally:
            sys.stdout = old
        out = buf.getvalue()
        # 20.0〜25.0 を 10Hz で両端込み = 51 行・MARK 2 行（EP_START と EP_END）
        if "(51 data rows, 2 marks)" not in out or "== 範囲: ep=1" not in out or "→ 保存(推定)" not in out:
            print("FAIL slice report:\n" + out); ok = False
    return ok


def _selftest_anomaly_line() -> bool:
    lines = _logger_summary("x.csv", dt.datetime.now(dt.timezone.utc), 10.0, ["can_fol_r"], {"can_fol_r": 100}, [],
                            {("can_fol_r", "gripper"): 82}, {("can_fol_r", "gripper"): 40}, 1, 1, [], [], [],
                            lambda _i: "", anomaly={"run_lost": 2, "bad": 5, "first_bad": "can_fol_r gripper"})
    mat = [x for x in lines if x.startswith("判定材料")]
    ok = len(mat) == 1 and "[run_lost=2 bad_state=5 max_t_rotor=82]" in mat[0]
    ok = ok and lines.index(mat[0]) < lines.index("== MARK (全件)")
    if not ok:
        print("FAIL anomaly line: " + repr(mat))
    return ok


def _selftest_live() -> bool:
    motors = ["joint_1", "joint_7", "gripper"]
    fol = _selftest_rows_bus(0.0, 40.0, 50.0, "can_fol_r", motors, drop={"gripper": (30.0, 31.0)},
                             state={"joint_7": lambda t: 0xC if t >= 35.0 else 1},
                             temp={"gripper": lambda t: 40 + int(t)})
    lead = _selftest_rows_bus(0.0, 40.0, 50.0, "can_lead_r", motors)
    for r in lead:  # leader を 0.2rad 先行させる（最初の 5 秒は差 0 = 基準）
        t = float(r["t_rel"])
        r["pos_rad"] = f"{0.1 * t + (0.2 if t > 5.0 else 0.0):.5f}"
    rows = sorted(fol + lead, key=lambda r: float(r["t_rel"]))
    rows.append(_selftest_mark(30.31, "LOST can_fol_r gripper at t_rel=30.31s", "EVENT"))
    rows.append(_selftest_mark(31.0, "BACK can_fol_r gripper at t_rel=31.00s", "EVENT"))
    rows.append(_selftest_mark(33.0, "EP_START ep=4 take=5"))
    rows.sort(key=lambda r: (float(r["t_rel"]), r["iface"] == "EVENT"))
    view = LiveView(30.0, {("can_lead_r", m): 0.0 for m in motors})
    for r in rows:
        view.feed(r["iface"], r["motor"], r["t_rel"], r["state_hex"], r["state_name"], r["pos_rad"], r["t_rotor"])
    out = "\n".join(view.render("hdr"))
    need = ["最大 79C@grip", "動作中の LOST 1", "j7 COIL_OVERTEMP", "|e|max can_lead_r-can_fol_r",
            "j1 0.200", "EP_START ep=4 take=5"]
    miss = [x for x in need if x not in out]
    if miss:
        print("FAIL live view missing " + repr(miss) + "\n" + out)
    return not miss


def _selftest_spool() -> bool:
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        sp = _Spool(td)
        want = [f"line {i}" for i in range(_Spool.MEM_LINES * 3)]
        for x in want:
            sp.add(x)
        got = list(sp)
        sp.close()
    ok = got == want and sp.n == len(want)
    if not ok:
        print("FAIL spool")
    return ok


# -------------------------------------------------------------------- cli --

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ifaces", nargs="*", help="記録対象の socketcan インタフェース（例 can_fol_r can_lead_r）")
    ap.add_argument("--out", default=None, help="CSV 出力先（既定: ./damiao-feedback-<UTC>.csv）")
    ap.add_argument("--print-every", type=float, default=2.0)
    ap.add_argument("--report", metavar="CSV", default=None, help="記録済み CSV を解析する（送信も受信もしない）")
    ap.add_argument("--pre", type=float, default=5.0, help="--report: 最初のエラー前の統計窓[s]")
    ap.add_argument("--window", type=float, default=30.0, help="--report: 傾き計算等の窓[s] / --live: 要約の窓[s]")
    ap.add_argument("--episode", type=int, default=None, help="--report: EP_START ep=N の区間だけを解析する")
    ap.add_argument("--take", type=int, default=None, help="--report: テイク K（take=K）の区間だけを解析する")
    ap.add_argument("--since", type=float, default=None, help="--report: t_rel がこの秒以上の行だけ")
    ap.add_argument("--until", type=float, default=None, help="--report: t_rel がこの秒以下の行だけ")
    ap.add_argument("--live", metavar="CSV", default=None, help="記録中の CSV を追って直近の窓の要約を出す（読むだけ）")
    ap.add_argument("--every", type=float, default=5.0, help="--live: 要約を出す間隔[s]")
    ap.add_argument("--from-start", action="store_true", help="--live: 末尾でなく先頭から読む")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--follow", action="append", default=[], metavar="FILE",
                    help="追記を読んでエピソード境界や 'MARK <text>' 行を MARK にする（複数可）")
    ap.add_argument("--marks", action="append", default=[], metavar="FILE",
                    help="--follow と同じだが先頭から読む（シェルが 'MARK <text>' を追記する専用ファイル）")
    ap.add_argument("--summary-out", default=None, help="停止時の短い要約の書き出し先")
    ap.add_argument("--watch-pid", type=int, default=None, help="この PID が消えたら止まる（孤児防止）")
    ap.add_argument("--pid-file", default=None, help="自分の PID を書く（停止側が使う・終了時に消す）")
    args = ap.parse_args()

    if args.selftest:
        return run_selftest()
    if args.live:
        return run_live(args.live, args.every, args.window, args.from_start)
    if args.report:
        # timeout（dmlog.sh）の SIGTERM でも一時ディレクトリ（CSV の横の .dmlog-report-*）を消してから終わる
        signal.signal(signal.SIGTERM, lambda *_a: sys.exit(143))
        t_lo, t_hi, label = None, None, None
        if args.episode is not None or args.take is not None:
            found = episode_range(scan_marks(args.report), args.episode, args.take)
            want = " ".join(x for x in (f"ep={args.episode}" if args.episode is not None else "",
                                        f"take={args.take}" if args.take is not None else "") if x)
            if found is None:
                print(f"!! {args.report}: EP_START {want} の MARK が無い（収集の CSV か、番号を確かめる）", file=sys.stderr)
                return 2
            t_lo, t_hi, m0, m1 = found
            label = f"{m0} 〜 {m1 or '(終わりの MARK なし・末尾まで)'}"
        if args.since is not None:
            t_lo = args.since if t_lo is None else max(t_lo, args.since)
        if args.until is not None:
            t_hi = args.until if t_hi is None else min(t_hi, args.until)
        if label is None and (args.since is not None or args.until is not None):
            label = "--since/--until"
        return run_report(args.report, args.pre, args.window, t_lo, t_hi, label)
    if not args.ifaces:
        ap.error("ifaces が必要です（--report / --selftest でない場合）")
    out_path = args.out or f"./damiao-feedback-{dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.csv"
    return run_logger(args.ifaces, out_path, args.print_every, follow=args.follow, summary_out=args.summary_out,
                      watch_pid=args.watch_pid, pid_file=args.pid_file, marks_files=args.marks)


if __name__ == "__main__":
    sys.exit(main())
