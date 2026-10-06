#!/usr/bin/env python3
"""
test_eval_scheduler.py — eval_openarm.py の新しいスケジューラ（sync+rtc / async / async+rtc）の単体試験（2026-10-07）

実機も PolicyServer も使わない。偽のロボット（送った指令の時刻と値を記録）と、偽のポリシー
（時刻の関数 g(t) からチャンクを作り、推論の遅れを sleep で再現）で回す。

確かめること:
  1. async で、送った指令がその時刻の g(t) とほぼ一致する（= 切り替えの位置 k の取り方が正しい）。
  2. rtc で送る前のチャンク（prev_action）の j 番目が、今回の観測の時刻から j step 後の g と一致する。
  3. sync+rtc の overlap が「チャンク長 - action_horizon」になり、frozen が min(rtc_frozen, overlap) になる。
  4. underrun の数（推論が遅いほど増える）と、Ctrl-C 相当の停止。

実行（Vector・lerobot の venv）:
    ~/projects/lerobot/.venv/bin/python scripts/g4/test_eval_scheduler.py
"""

import csv
import io
import os
import sys
import threading
import time
import types

# msgpack / zmq が無い venv でも読み込めるように差し替える（スケジューラは使わない）
for _m in ("msgpack", "zmq"):
    try:
        __import__(_m)
    except ImportError:
        sys.modules[_m] = types.ModuleType(_m)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import eval_openarm as ev  # noqa: E402

FPS = 30
PERIOD = 1.0 / FPS
H = 16
STATE_KEYS = ev.BIMANUAL_STATE_KEYS


def g(t: float, j: int) -> float:
    """関節 j の目標角（度）を時刻の関数にする。1 秒で ±20 度ほど動く。"""
    return 20.0 * np.sin(2 * np.pi * 0.5 * t + 0.3 * j)


class FakeRobot:
    def __init__(self, t0: float):
        self.t0 = t0
        self.sent = []  # (wall_t, action_dict)
        self.last = {k: g(0.0, i) for i, k in enumerate(STATE_KEYS)}
        self.bus_lock = threading.Lock()
        self.main_thread = threading.get_ident()
        self.wrong_thread = 0

    def _check_thread(self):
        if threading.get_ident() != self.main_thread:
            self.wrong_thread += 1

    def get_observation(self):
        self._check_thread()
        o = dict(self.last)
        o["_t"] = time.time() - self.t0
        o["chest"] = np.zeros((2, 2, 3), dtype=np.uint8)
        return o

    def send_action(self, a):
        self._check_thread()
        self.sent.append((time.time() - self.t0, dict(a)))
        self.last = dict(a)


class FakePolicy:
    """OpenArmAdapter.get_chunk の代わり。観測の時刻 t_obs から j step 後の g を 0..H-1 番目に置く。"""

    def __init__(self, latency_s: float):
        self.latency_s = latency_s
        self.calls = []  # (t_obs, options の要約)
        self.rtc_records = []  # (t_obs, prev (overlap, 16), frozen)

    def get_chunk(self, obs, options=None):
        t_obs = obs["_t"]
        time.sleep(self.latency_s)
        summary = None
        if options and "rtc" in options:
            rtc = options["rtc"]
            prev = rtc["prev_action"]
            overlap = prev["right_arm"].shape[1]
            summary = (overlap, rtc["frozen"])
            full = np.concatenate([prev[key][0] for key, _, _ in ev.BIMANUAL_GROUPS], axis=-1)
            self.rtc_records.append((t_obs, full, rtc["frozen"]))
        self.calls.append((t_obs, summary))
        raw = {}
        for key, s, e in ev.BIMANUAL_GROUPS:
            raw[key] = np.array([[[g(t_obs + t * PERIOD, s + d) for d in range(e - s)] for t in range(H)]],
                                dtype=np.float32)
        actions = []
        for t in range(H):
            full = np.concatenate([raw[key][0, t] for key, _, _ in ev.BIMANUAL_GROUPS])
            actions.append({name: float(full[i]) for i, name in enumerate(STATE_KEYS)})
        return actions, raw, {}


def rtc_alignment(mode, sent, records):
    """rtc の prev_action の位置合わせを、壁時計ではなく実際に送った指令の列で確かめる（度の最大）。

    最初のチャンクと sync の推論待ちの間は腕が止まるので、チャンクの時間軸は壁時計から推論の時間だけ遅れる。
    そのため g(t) とは比べない。
      - async: 観測の直後に送った指令（前のチャンクの続き）が prev の 0〜3 番目と一致する。
        220ms の推論では切り替えが 6〜7 step 後なので、0〜3 番目は必ず前のチャンクから送られる。
      - sync: 推論を待つ間は送らないので、prev の 0 番目が「最後に送った指令の 1 step 先」になる。
        g の速さは最大 62.8 度/秒なので、1 step（33ms）で 2.1 度以内の差に収まる。
    """
    if not records:
        return None
    errs = []
    ts = [t for t, _ in sent]
    for t_obs, full, _frozen in records:
        i0 = next((i for i, t in enumerate(ts) if t >= t_obs), None)
        if i0 is None:
            continue
        if mode == "async":
            for j in range(min(4, full.shape[0])):
                if i0 + j < len(sent):
                    a = sent[i0 + j][1]
                    errs.append(max(abs(full[j][d] - a[k]) for d, k in enumerate(STATE_KEYS)))
        else:
            a = sent[i0 - 1][1]
            errs.append(max(abs(full[0][d] - a[k]) for d, k in enumerate(STATE_KEYS)) - 2.2)
    return float(max(errs)) if errs else None


class FakeRunner:
    def __init__(self):
        self.should_stop = False


def make_cfg(mode: str, rtc: bool, action_horizon: int = 16, max_steps: int = 150) -> ev.EvalConfig:
    cfg = ev.EvalConfig.__new__(ev.EvalConfig)
    cfg.inference_mode = mode
    cfg.rtc = rtc
    cfg.rtc_overlap = 8
    cfg.rtc_frozen = 8
    cfg.rtc_ramp_rate = 5.0
    cfg.action_horizon = action_horizon
    cfg.fps = FPS
    cfg.max_steps = max_steps
    cfg.dry_run = False
    cfg.first_action_max_dev = 1e9
    cfg.log_obs_every_step = True
    cfg.frame_log_dir = None
    cfg.frame_log_every = 1
    cfg.lang_instruction = "test"
    return cfg


def run(mode: str, rtc: bool, latency_s: float, action_horizon: int = 16, max_steps: int = 150, stop_at=None):
    t0 = time.time()
    robot, policy, runner = FakeRobot(t0), FakePolicy(latency_s), FakeRunner()
    buf = io.StringIO()
    w = csv.writer(buf)
    cfg = make_cfg(mode, rtc, action_horizon, max_steps)
    if stop_at is not None:
        threading.Timer(stop_at, lambda: setattr(runner, "should_stop", True)).start()
    track = {}
    ev._run_scheduled(cfg, robot, runner, policy, STATE_KEYS, w, lambda o, s: None, t0, track)
    rows = list(csv.reader(io.StringIO(buf.getvalue())))
    # 送った指令と、その時刻の g とのずれ（腕の関節 j=0 で見る）
    err = [abs(a[STATE_KEYS[0]] - g(t, 0)) for t, a in robot.sent]
    gaps = np.diff([t for t, _ in robot.sent]) * 1000 if len(robot.sent) > 1 else np.array([0.0])
    underrun = sum(int(r[-1]) for r in rows)
    return {
        "sent": len(robot.sent), "rows": len(rows), "underrun": underrun,
        "gap_max_ms": float(gaps.max()), "gap_p50_ms": float(np.median(gaps)),
        "err_p50": float(np.median(err)), "err_max_after_first": float(max(err[H:] or [0])),
        "rtc_calls": [c[1] for c in policy.calls if c[1] is not None],
        "align_err_max": rtc_alignment(mode, robot.sent, policy.rtc_records),
        "wrong_thread": robot.wrong_thread, "track": bool(track.get("first_obs")),
    }


def check(name, cond, detail=""):
    print(f"  {'OK ' if cond else 'NG '} {name} {detail}")
    if not cond:
        check.failed += 1


check.failed = 0


def main():
    import logging
    logging.basicConfig(level=logging.WARNING)

    for lat in (0.15, 0.22, 0.30):
        r = run("async", False, lat)
        print(f"[async latency {lat*1000:.0f}ms] {r}")
        check("CAN に触ったのは表のスレッドだけ", r["wrong_thread"] == 0)
        check("時刻に合った指令（g とのずれの中央値 < 0.5 度）", r["err_p50"] < 0.5, f"{r['err_p50']:.3f}")
        if lat <= 0.22:
            check("220ms 以下では止まらない（間隔の最大 < 50ms）", r["gap_max_ms"] < 50, f"{r['gap_max_ms']:.0f}ms")
        check("行数 = max_steps", r["rows"] == 150)
        check("開始・終了の関節角を track に入れた", r["track"])

    r = run("async", True, 0.22)
    print(f"[async+rtc 220ms] underrun={r['underrun']} rtc={r['rtc_calls'][:5]} align_err_max={r['align_err_max']}")
    check("rtc の prev_action が観測の後に実際に送った指令と一致する（< 1e-3 度）",
          r["align_err_max"] is not None and r["align_err_max"] < 1e-3, f"{r['align_err_max']}")
    check("async の frozen は rtc_frozen 以下・overlap 以下", all(f <= min(8, o) for o, f in r["rtc_calls"]))

    r = run("sync", True, 0.22, action_horizon=12, max_steps=60)
    print(f"[sync+rtc 220ms h=12] rtc={r['rtc_calls']} gap_max={r['gap_max_ms']:.0f}ms align_err_max={r['align_err_max']}")
    check("sync+rtc の overlap は 16-12=4・frozen は min(8, 4)=4", r["rtc_calls"] and all(c == (4, 4) for c in r["rtc_calls"]))
    check("sync+rtc の prev の 0 番目は最後に送った指令の 1 step 先（差 ≤ 2.2 度）",
          r["align_err_max"] is not None and r["align_err_max"] <= 0, f"超過 {r['align_err_max']}")
    check("sync は推論中に止まる（間隔の最大 > 200ms）", r["gap_max_ms"] > 200)

    try:
        run("sync", True, 0.05, action_horizon=16, max_steps=10)
        check("sync+rtc で action_horizon=チャンク長は中止する", False)
    except SystemExit:
        check("sync+rtc で action_horizon=チャンク長は中止する", True)

    t = time.time()
    r = run("async", False, 0.22, max_steps=100000, stop_at=1.0)
    el = time.time() - t
    check("停止要求で 1 秒あまりで抜ける", el < 1.6, f"{el:.2f}s")

    print("FAILED" if check.failed else "ALL OK", f"({check.failed} 件 NG)")
    sys.exit(1 if check.failed else 0)


if __name__ == "__main__":
    main()
