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
"""
from __future__ import annotations

import argparse
import bisect
import csv
import datetime as dt
import queue
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

def _reader(iface: str, out_q: "queue.Queue", stop: threading.Event, t0_mono: float) -> None:
    import can

    try:
        bus = can.Bus(channel=iface, interface="socketcan", fd=True)
    except Exception as exc:  # noqa: BLE001 - インタフェース不在等は起動時に握って続行
        print(f"!! {iface}: open failed: {exc}", file=sys.stderr)
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
        text = line.strip()
        out_q.put({
            "t_wall": f"{time.time():.6f}",
            "t_rel": f"{time.monotonic() - t0_mono:.6f}",
            "iface": "MARK", "motor": text or "(mark)", "mtype": "", "state_hex": "", "state_name": "",
            "pos_rad": "", "vel_rad_s": "", "tau_nm": "", "t_mos": "", "t_rotor": "", "raw_hex": "",
        })


def run_logger(ifaces: list[str], out_path: str, print_every: float) -> int:
    import can  # noqa: F401 - 早期に import エラーを出す

    t0_mono = time.monotonic()
    start_wall = dt.datetime.now(dt.timezone.utc)
    out_q: "queue.Queue" = queue.Queue()
    stop = threading.Event()

    f = open(out_path, "w", newline="")
    f.write(f"# start_wall={start_wall.isoformat()} host={__import__('socket').gethostname()} "
            f"ifaces={','.join(ifaces)}\n")
    writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
    writer.writeheader()

    readers = [threading.Thread(target=_reader, args=(i, out_q, stop, t0_mono), daemon=True) for i in ifaces]
    marker = threading.Thread(target=_stdin_marker, args=(out_q, stop, t0_mono), daemon=True)
    for t in readers:
        t.start()
    marker.start()

    # 状態管理: 直近5秒の |tau| リングバッファと現在 state（初回遷移検出用）
    ring: dict[tuple, list[tuple[float, float]]] = {}
    cur_state: dict[tuple, int] = {}
    counts: dict[str, int] = {i: 0 for i in ifaces}
    tmax_seen: dict[tuple, int] = {}
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
                    prev = cur_state.get(key)
                    cur_state[key] = st
                    tmax_seen[key] = max(tmax_seen.get(key, 0), int(row["t_rotor"]))
                    if prev is not None and prev in (0, 1) and st not in (0, 1):
                        tau_peak5 = max((abs(x[1]) for x in buf), default=0.0)
                        print(f"!!! {row['iface']} {row['motor']} {STATE.get(prev, prev)}->"
                              f"{row['state_name']} at t_rel={t_rel:.2f}s wall={row['t_wall']} "
                              f"T_mos={row['t_mos']} T_rotor={row['t_rotor']} "
                              f"|tau| max(prev 5s)={tau_peak5:.2f}")
                else:
                    print(f"MARK t_rel={row['t_rel']}s: {row['motor']}")
                last_any = time.monotonic()

            now = time.monotonic()
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
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for t in readers:
            t.join(timeout=1.0)
        f.flush()
        f.close()
    return 0


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
    data_rows = [r for r in rows if r.get("iface") != "MARK"]
    marks = [r for r in rows if r.get("iface") == "MARK"]

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

    if marks:
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
    print("SELFTEST " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


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
    args = ap.parse_args()

    if args.selftest:
        return run_selftest()
    if args.report:
        return run_report(args.report, args.pre, args.window)
    if not args.ifaces:
        ap.error("ifaces が必要です（--report / --selftest でない場合）")
    out_path = args.out or f"./damiao-feedback-{dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.csv"
    return run_logger(args.ifaces, out_path, args.print_every)


if __name__ == "__main__":
    sys.exit(main())
