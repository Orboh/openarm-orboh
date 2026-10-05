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


def episode_table(marks: list[tuple[str, str]], exit_rc: str | None = None) -> list[str]:
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
        dur = f"{tk['t1'] - tk['t0']:.1f}s" if tk["t1"] is not None else "-"
        t1 = f"{tk['t1']:.1f}" if tk["t1"] is not None else "-"
        lines.append(f"   ep={tk['ep']} take={tk['take']} t_rel={tk['t0']:.1f}-{t1}s ({dur}) "
                     f"end={tk['key']} → {tk['status']}")
    kept = sum(1 for tk in takes if tk["status"].startswith("保存"))
    if takes:
        lines.insert(0, f"テイク {len(takes)} 本（保存 {kept}・それ以外 {len(takes) - kept}）")
    return lines


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
        if summary_out:
            try:
                lines = _logger_summary(out_path, start_wall, time.monotonic() - t0_mono, ifaces, counts,
                                        open_failed, tmax_seen, tmos_seen, n_lost, n_back, events, marks,
                                        stop_reason, can_stats)
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
                    n_lost, n_back, events, marks, stop_reason, can_stats) -> list[str]:
    """停止時の短い要約（CSV を読み直さない・一瞬で終わる）。'== MARK (全件)' 以降は端末に出さない前提。"""
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

def _read_csv_rows(path: str) -> list[dict]:
    rows = []
    with open(path, newline="") as f:
        for line in f:
            if line.startswith("#"):
                continue
            break
        f.seek(0)
        lines = [l for l in f if not l.startswith("#")]  # noqa: E741
    reader = csv.DictReader(lines)
    for r in reader:
        rows.append(r)
    return rows


def _num(r: dict, k: str) -> float | None:
    v = r.get(k, "")
    if v in (None, ""):
        return None
    return float(v)


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


def run_report(path: str, pre_s: float, window_s: float) -> int:
    rows = _read_csv_rows(path)
    # EVENT（LOST/BACK）と CANSTAT（毎秒のバス統計）はモータの行ではないので分ける（2026-10-05）。
    # 従来はモータ扱いで 1 行ずつ "-- CANSTAT ..." が出て、長い記録では出力が数千行に膨れていた。
    data_rows = [r for r in rows if r.get("iface") not in ("MARK", "EVENT", "CANSTAT")]
    marks = [r for r in rows if r.get("iface") == "MARK"]
    event_rows = [r for r in rows if r.get("iface") == "EVENT"]
    canstat_last: dict[str, str] = {}
    for r in rows:
        if r.get("iface") == "CANSTAT":
            name, _, rest = (r.get("motor") or "").partition(" ")
            canstat_last[name] = rest

    by_key: dict[tuple, list[dict]] = {}
    for r in data_rows:
        by_key.setdefault((r["iface"], r["motor"]), []).append(r)
    for lst in by_key.values():
        lst.sort(key=lambda r: _num(r, "t_rel") or 0.0)

    print(f"== report: {path}  ({len(data_rows)} data rows, {len(marks)} marks)")

    # (a) frames / rate / transitions
    transitions: dict[tuple, list[dict]] = {}
    first_error_t: dict[str, float] = {}  # iface -> t_rel of first error across any motor on that iface
    for key, lst in sorted(by_key.items()):
        iface, motor = key
        if len(lst) >= 2:
            t0 = _num(lst[0], "t_rel") or 0.0
            t1 = _num(lst[-1], "t_rel") or 0.0
            rate = (len(lst) - 1) / (t1 - t0) if t1 > t0 else 0.0
        else:
            rate = 0.0
        print(f"-- {iface} {motor}: frames={len(lst)} rate={rate:.1f}Hz")

        # 反証レビュー指摘(a): state 分布（state_hex -> 件数・初出 t_rel）を先に出す。
        # 点滅（ラッチ）後にロガーを起動した場合、記録開始時点で既にエラー state のことがあり
        # （2026-09-21 実機: errcheck-latched は t_rel=0.0s から連続 0xC）、その場合は下の
        # 遷移検出（0/1 -> 非0/1）に一度も掛からないため、分布を見ないと見落とす。
        state_counts: dict[int, int] = {}
        state_first_t: dict[int, float] = {}
        for r in lst:
            st = int(r["state_hex"], 16) if r["state_hex"] else None
            if st is None:
                continue
            t_rel_s = _num(r, "t_rel") or 0.0
            state_counts[st] = state_counts.get(st, 0) + 1
            if st not in state_first_t:
                state_first_t[st] = t_rel_s
        if state_counts:
            dist = ", ".join(
                f"{STATE.get(s, hex(s))}={state_counts[s]}(first@{state_first_t[s]:.2f}s)"
                for s in sorted(state_counts))
            print(f"   state分布: {dist}")

        prev_state = None
        trs = []
        for r in lst:
            st = int(r["state_hex"], 16) if r["state_hex"] else None
            if st is None:
                continue
            if prev_state is None and st not in (0, 1):
                # 記録開始時点で既にラッチ（点滅後の起動）。遷移は記録されていないので別扱いで
                # first_error_t だけ立て、pre統計はスキップされても冷却曲線は出るようにする。
                t_rel = _num(r, "t_rel") or 0.0
                first_error_t[iface] = min(first_error_t.get(iface, t_rel), t_rel)
                print(f"   記録開始時点で既にラッチ: t_rel={t_rel:.2f}s wall={r['t_wall']} "
                      f"state={r['state_name']} T_mos={r['t_mos']} T_rotor={r['t_rotor']} "
                      f"(遷移は未記録・pre統計はスキップ)")
            elif prev_state is not None and prev_state in (0, 1) and st not in (0, 1):
                trs.append(r)
                t_rel = _num(r, "t_rel") or 0.0
                first_error_t[iface] = min(first_error_t.get(iface, t_rel), t_rel)
                print(f"   transition t_rel={t_rel:.2f}s wall={r['t_wall']} "
                      f"{STATE.get(prev_state, prev_state)}->{r['state_name']} "
                      f"T_mos={r['t_mos']} T_rotor={r['t_rotor']} tau={r['tau_nm']}")
            prev_state = st
        transitions[key] = trs

    # (b)+(c) per-iface pre/window stats
    for iface in sorted({k[0] for k in by_key}):
        t_err = first_error_t.get(iface)
        pre_label = f"最初のエラー前 {pre_s:g}s" if t_err is not None else "(エラー無し: pre統計スキップ)"
        print(f"== {iface}: {pre_label}")
        if t_err is not None:
            for motor in sorted({k[1] for k in by_key if k[0] == iface}):
                lst = by_key[(iface, motor)]
                pre_rows = [r for r in lst if t_err - pre_s <= (_num(r, "t_rel") or -1e9) < t_err]
                if not pre_rows:
                    continue
                taus = [_num(r, "tau_nm") or 0.0 for r in pre_rows]
                ts = [_num(r, "t_rel") or 0.0 for r in pre_rows]
                trotors = [int(r["t_rotor"]) for r in pre_rows if r["t_rotor"] != ""]
                tau_peak = max((abs(x) for x in taus), default=0.0)
                tau_rms = (sum(x * x for x in taus) / len(taus)) ** 0.5 if taus else 0.0
                i2t = _trapz_sq(ts, taus)
                print(f"   {motor}: |tau|peak={tau_peak:.2f}Nm tau_rms={tau_rms:.2f}Nm "
                      f"T_rotor[min-max]={min(trotors) if trotors else '-'}-{max(trotors) if trotors else '-'}C "
                      f"I2t~{i2t:.2f}")

        win_label = f"全区間 or 最初のエラー前 {window_s:g}s"
        print(f"== {iface}: {win_label}")
        for motor in sorted({k[1] for k in by_key if k[0] == iface}):
            lst = by_key[(iface, motor)]
            if t_err is not None:
                win_rows = [r for r in lst if max(0.0, t_err - window_s) <= (_num(r, "t_rel") or -1e9) < t_err]
            else:
                win_rows = lst
            if not win_rows:
                continue
            taus = [_num(r, "tau_nm") or 0.0 for r in win_rows]
            ts = [_num(r, "t_rel") or 0.0 for r in win_rows]
            trotors = [float(r["t_rotor"]) for r in win_rows if r["t_rotor"] != ""]
            tmoss = [float(r["t_mos"]) for r in win_rows if r["t_mos"] != ""]
            tau_rms = (sum(x * x for x in taus) / len(taus)) ** 0.5 if taus else 0.0
            slope_rotor = _lstsq_slope(ts, trotors) * 60.0 if trotors else 0.0
            slope_mos = _lstsq_slope(ts, tmoss) * 60.0 if tmoss else 0.0
            print(f"   {motor}: tau_rms={tau_rms:.2f}Nm T_rotor_slope={slope_rotor:.2f}C/min "
                  f"T_mos_slope={slope_mos:.2f}C/min T_rotor_max={max(trotors) if trotors else '-'}C")

        # (d) cooling curve after first error (10s steps, 60s)
        if t_err is not None:
            print(f"== {iface}: エラー後の T_rotor 冷却曲線（10秒刻み・60秒。測定値か推定値かの判別材料）")
            for motor in sorted({k[1] for k in by_key if k[0] == iface}):
                lst = by_key[(iface, motor)]
                post = [(_num(r, "t_rel") or 0.0, float(r["t_rotor"])) for r in lst
                        if r["t_rotor"] != "" and (_num(r, "t_rel") or -1e9) >= t_err]
                if not post:
                    continue
                vals = []
                for step in range(0, 61, 10):
                    target = t_err + step
                    cand = [v for (t, v) in post if t <= target]
                    vals.append(f"{step}s={cand[-1]:.0f}C" if cand else f"{step}s=-")
                print(f"   {motor}: {' '.join(vals)}")

    # (e) leader/follower pairing
    ifaces = sorted({k[0] for k in by_key})
    pairs = []
    for i in ifaces:
        if "lead" in i:
            fol = i.replace("lead", "fol")
            if fol in ifaces:
                pairs.append((i, fol))
    for lead_i, fol_i in pairs:
        t_err = first_error_t.get(fol_i, first_error_t.get(lead_i))
        print(f"== 位置誤差 e(t)=pos_lead-pos_fol: {lead_i} vs {fol_i} "
              f"(基準: 最初の5秒の中央値を較正オフセット差として近似的に減算)")
        motors = sorted({m for (i, m) in by_key if i in (lead_i, fol_i)})
        for motor in motors:
            lead_rows = by_key.get((lead_i, motor), [])
            fol_rows = by_key.get((fol_i, motor), [])
            if not lead_rows or not fol_rows:
                continue
            # 反証レビュー指摘(b): fol_rows は by_key ソート済み（L313-314）で t_rel 昇順なので
            # bisect で二分探索（O(N log N)）。旧実装は lead 行ごとに fol 全行を線形走査する
            # O(N_lead*N_fol) で、練習5〜10分の実運用記録長では --report が実質終わらなかった。
            fol_ts = [(_num(r, "t_rel") or 0.0, _num(r, "pos_rad") or 0.0) for r in fol_rows]
            fol_times = [tv[0] for tv in fol_ts]

            def nearest(t: float) -> float | None:
                if not fol_ts:
                    return None
                i = bisect.bisect_left(fol_times, t)
                cands = []
                if i < len(fol_ts):
                    cands.append(fol_ts[i])
                if i > 0:
                    cands.append(fol_ts[i - 1])
                best = min(cands, key=lambda tv: abs(tv[0] - t))
                return best[1]

            # 指摘(b)の追加提案: 突き合わせ対象を較正窓（先頭5s）と pre 窓（t_err-pre_s〜t_err）
            # だけに絞る（エラーが無い iface ペアでは従来どおり全行を使う）。
            lead_t0 = _num(lead_rows[0], "t_rel") or 0.0
            if t_err is not None:
                lead_rows_for_err = [
                    r for r in lead_rows
                    if (lead_t0 <= (_num(r, "t_rel") or -1e9) <= lead_t0 + 5.0)
                    or (t_err - pre_s <= (_num(r, "t_rel") or -1e9) < t_err)
                ]
            else:
                lead_rows_for_err = lead_rows

            errs = []
            for r in lead_rows_for_err:
                t = _num(r, "t_rel") or 0.0
                fp = nearest(t)
                if fp is None:
                    continue
                errs.append((t, (_num(r, "pos_rad") or 0.0) - fp))
            if not errs:
                continue
            t0 = lead_t0
            baseline_vals = [e for (t, e) in errs if t - t0 <= 5.0]
            baseline = statistics.median(baseline_vals) if baseline_vals else 0.0
            if t_err is not None:
                pre = [(t, e - baseline) for (t, e) in errs if t_err - pre_s <= t < t_err]
            else:
                pre = [(t, e - baseline) for (t, e) in errs]
            if not pre:
                continue
            abs_e = [abs(e) for (_t, e) in pre]
            kp = FOLLOWER_KP.get(motor, 0)
            print(f"   {motor}: |e|max={max(abs_e):.3f}rad |e|mean={sum(abs_e)/len(abs_e):.3f}rad "
                  f"kp*|e|max~{kp*max(abs_e):.2f}Nm kp*|e|mean~{kp*sum(abs_e)/len(abs_e):.2f}Nm")

    if event_rows or canstat_last:
        n_lost = sum(1 for r in event_rows if (r.get("motor") or "").startswith("LOST"))
        n_back = sum(1 for r in event_rows if (r.get("motor") or "").startswith("BACK"))
        print(f"== EVENT: LOST {n_lost} 回 / BACK {n_back} 回")
        for r in event_rows:
            print(f"   t_rel={r['t_rel']}s: {r['motor']}")
        for name, rest in sorted(canstat_last.items()):
            print(f"   canstat(最後) {name}: {rest or 'n/a'}")

    if marks:
        tbl = episode_table([(m["t_rel"], m["motor"]) for m in marks])
        if tbl:
            print("== エピソード（MARK から）: " + tbl[0])
            for line in tbl[1:]:
                print(line)
        print("== MARK")
        for m in marks:
            print(f"   t_rel={m['t_rel']}s: {m['motor']}")

    return 0


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


# -------------------------------------------------------------------- cli --

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ifaces", nargs="*", help="記録対象の socketcan インタフェース（例 can_fol_r can_lead_r）")
    ap.add_argument("--out", default=None, help="CSV 出力先（既定: ./damiao-feedback-<UTC>.csv）")
    ap.add_argument("--print-every", type=float, default=2.0)
    ap.add_argument("--report", metavar="CSV", default=None, help="記録済み CSV を解析する（送信も受信もしない）")
    ap.add_argument("--pre", type=float, default=5.0, help="--report: 最初のエラー前の統計窓[s]")
    ap.add_argument("--window", type=float, default=30.0, help="--report: 傾き計算等の窓[s]")
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
    if args.report:
        return run_report(args.report, args.pre, args.window)
    if not args.ifaces:
        ap.error("ifaces が必要です（--report / --selftest でない場合）")
    out_path = args.out or f"./damiao-feedback-{dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.csv"
    return run_logger(args.ifaces, out_path, args.print_every, follow=args.follow, summary_out=args.summary_out,
                      watch_pid=args.watch_pid, pid_file=args.pid_file, marks_files=args.marks)


if __name__ == "__main__":
    sys.exit(main())
