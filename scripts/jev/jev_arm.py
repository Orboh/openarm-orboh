#!/usr/bin/env python3
"""
jev_arm.py — Jev（TypeSafe System One モデル）を判断層に置いた OpenArm 単腕の関節ステップ制御。

設計正本: orboh-company-vault `07Member's-free-space/Kento/02-Tech-Plan/openarm-jev-decision-layer-design.md`
（§3 段A = 関節ステップ選択。§6 安全設計）。

構造（二層）:
  - 判断層  decide_hz（既定 2Hz）: 現在関節角・目標関節角・履歴を JSON 状態にして Decider に渡し、
            有限のアクション（j1_plus … j7_minus / gripper_open / gripper_close / hold / done）から1つ選ぶ。
            Decider は "rules"（決定論・Jev 不要・配線と安全経路の検証用）と "jev"（本番）を切替。
  - 実行層  fps（既定 30Hz）: 「意図姿勢 q_des」を持ち、選ばれた関節に ±step_deg を q_des に足した目標へ線形補間して
            robot.send_action() で送る。判断が無い間は q_des を送り続ける（hold ＝ 意図姿勢の保持）。
            補間の起点も hold の値も q_des であり、実測 current には再基準化しない（2026-09-20 の実機で、実測起点に
            していたため重力で垂れた位置を毎判断「新しい基準」として受け入れ、手首が 74°→1° まで落ちた）。

安全（安全プロトコル §0 と設計ノート §6）:
  - SafeRunner（g4/eval_openarm.py から複製）: どの終了経路でも disable_torque → disconnect に到達させる。
  - ソフト E-stop: estop_path（既定 /tmp/openarm_estop）が存在したら hold → 停止処理。
    別端末から `touch /tmp/openarm_estop` で止められる（安全プロトコル TODO#3 の最小実装・このスクリプト内のみ）。
  - リース: lease_s 以内に判断が来なければ hold を続ける（動かす判断だけを Decider に委ね、止まる判断は依存しない）。
  - 予算: budget_usd を超えたら Jev を呼ばず hold → 終了。
  - 認証エラー（401/403）は即中止。429/529 は指数バックオフで 1 回だけ再試行、失敗なら hold。
  - confidence < confidence_min の判断は hold に置換。
  - 追従誤差ガード: 関節 1..7 のどれかで |実測 − q_des| > track_err_max_deg なら「hold が保持になっていない」
    （重力負け・接触・モータ異常）とみなして停止処理へ。
  - max_relative_target は MIT 位置制御では「1 tick の移動量」でなく「復元トルク上限 = kp × clamp」として効く
    （2026-09-20 実測: clamp 1.0° × kp 25 = 0.44 Nm は手首の重力負荷 約1.2 Nm に負け、kp 240 × 1.0° = 4.2 Nm は
    肩の 約6.5 Nm に負けた）。起動時に関節ごとの上限を表示し、実測負荷を下回る関節に警告する。
    実機では関節別 dict（肩肘 2.0 / 手首 4.0 / グリッパ 1.0）を指定する。executor 自身の補間が 0.33°/tick なので
    実効速度はそこで抑えられ、クランプは executor のバグに対する最後の砦として機能する。

3つの実行モード:
  --no_robot=true   腕を一切構築しない。initial_pose YAML の角度を仮想関節として持ち、指令をそのまま反映する。
                    Decider・状態生成・ループ・記録の検証用（通電しない）。
  --dry_run=true    腕を接続する（通電・自動整列 2.2 秒は起きる）が send_action は送らない。
  既定              実駆動。

例（Thor・rules・腕なし）:
  scripts/jev/run_jev.sh smoke-rules -- --no_robot=true --decider=rules \
      --target_offsets_deg='{"joint_4": 10.0}'
例（Thor・Jev・実機・右腕）:
  TYPESAFE_API_KEY=... scripts/jev/run_jev.sh jev-real -- --decider=jev \
      --robot.type=openarm_follower --robot.port=can_fol_r --robot.side=right --robot.id=follower_right \
      --robot.cameras='{}' --dry_run=false \
      --robot.max_relative_target='{"joint_1":2.0,"joint_2":2.0,"joint_3":2.0,"joint_4":2.0,"joint_5":4.0,"joint_6":4.0,"joint_7":4.0,"gripper":1.0}' \
      --goal="肘（joint_4）を 10 度曲げる" --target_offsets_deg='{"joint_4": 10.0}'
"""


import csv
import json
import logging
import os
import signal
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from pprint import pformat
from typing import Any, Dict, List, Optional

import draccus
import requests
import yaml

from lerobot.robots import (  # noqa: F401  — draccus の choice registry 登録のため
    Robot,
    RobotConfig,
    make_robot_from_config,
    openarm_follower,
)

import lerobot.cameras.opencv  # noqa: F401
import lerobot.cameras.realsense  # noqa: F401
from lerobot.utils.utils import init_logging

# ----------------------------------------------------------------------------------------------
# 定数
# ----------------------------------------------------------------------------------------------

JOINT_KEYS = [f"joint_{i}.pos" for i in range(1, 8)]
GRIPPER_KEY = "gripper.pos"
STATE_KEYS = JOINT_KEYS + [GRIPPER_KEY]

JEV_ENDPOINT = "https://api.typesafe.ai/v1/systemone"
JEV_PRICE_USD_PER_INPUT_TOKEN = 0.042 / 1_000_000  # 公開一覧値（出力無料）


def build_action_catalog(step_deg: float, enable_gripper: bool) -> Dict[str, str]:
    """Jev の choice.criteria（アクション名 → 説明）。説明文が Jev にとっての唯一の運動学知識になる。"""
    cat: Dict[str, str] = {}
    for i in range(1, 8):
        cat[f"j{i}_plus"] = f"joint_{i} を +{step_deg:g} 度動かす（joint_{i}.pos が増える方向）"
        cat[f"j{i}_minus"] = f"joint_{i} を -{step_deg:g} 度動かす（joint_{i}.pos が減る方向）"
    if enable_gripper:
        cat["gripper_open"] = "グリッパを開く方向へ 1 段動かす"
        cat["gripper_close"] = "グリッパを閉じる方向へ 1 段動かす"
    cat["hold"] = "今の姿勢を保つ（判断を保留する・危ないと感じたら選ぶ）"
    cat["done"] = "目標に十分到達したので終了する"
    return cat


# ----------------------------------------------------------------------------------------------
# Decider
# ----------------------------------------------------------------------------------------------


@dataclass
class Decision:
    action: str
    confidence: float
    probabilities: Dict[str, float]
    latency_s: float
    source: str
    input_tokens: int = 0
    cost_usd: float = 0.0
    done_prob: Optional[float] = None
    raw: Optional[dict] = None
    error: Optional[str] = None


class FatalDeciderError(RuntimeError):
    """認証エラー等、続行しても意味が無い失敗。ループを止める。"""


class RulesDecider:
    """決定論バックエンド。目標との誤差が最大の関節を 1 ステップ動かす。Jev 無しで配線・安全経路を検証する。"""

    def __init__(self, catalog: Dict[str, str], tol_deg: float):
        self.catalog = catalog
        self.tol_deg = tol_deg

    def decide(self, state: dict) -> Decision:
        t0 = time.time()
        err = state["error_deg"]  # joint_1..7 → target - current
        worst_k = max(err, key=lambda k: abs(err[k]))
        if abs(err[worst_k]) <= self.tol_deg:
            act = "done"
        else:
            idx = worst_k.split("_")[1]
            act = f"j{idx}_plus" if err[worst_k] > 0 else f"j{idx}_minus"
        probs = {k: (1.0 if k == act else 0.0) for k in self.catalog}
        return Decision(act, 1.0, probs, time.time() - t0, "rules")


class JevDecider:
    """TypeSafe /v1/systemone を requests で直接叩く。SDK 非依存（API 仕様: docs.typesafe.ai/api）。"""

    def __init__(
        self,
        catalog: Dict[str, str],
        model: str,
        timeout_s: float,
        api_key: Optional[str] = None,
        endpoint: str = JEV_ENDPOINT,
    ):
        self.catalog = catalog
        self.model = model
        self.timeout_s = timeout_s
        self.endpoint = endpoint
        self.api_key = api_key or os.environ.get("TYPESAFE_API_KEY", "")
        if not self.api_key:
            raise FatalDeciderError("TYPESAFE_API_KEY が未設定（環境変数で渡す。リポジトリには書かない）")
        self.session = requests.Session()
        self.session.headers.update(
            {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        )

    def _payload(self, state: dict) -> dict:
        return {
            "model": self.model,
            "state": state,
            "questions": {
                "next_action": {
                    "type": "choice",
                    "instructions": (
                        "あなたはロボットアーム（7関節＋グリッパ）の判断層。state の goal を達成するため、"
                        "target_q_deg と q_deg の差 error_deg を小さくする次の1手を選ぶ。"
                        "誤差が最大の関節を、error の符号方向に動かすのが基本。"
                        "全関節の |error_deg| が tol_deg 以下なら done。迷う・危ないと思えば hold。"
                    ),
                    "criteria": self.catalog,
                },
                "goal_reached": {
                    "type": "noul",
                    "instructions": "全関節の |error_deg| が tol_deg 以下で、目標に到達していると言えるか。",
                },
            },
        }

    def decide(self, state: dict) -> Decision:
        payload = self._payload(state)
        t0 = time.time()
        last_err = None
        for attempt in range(2):
            try:
                r = self.session.post(self.endpoint, json=payload, timeout=self.timeout_s)
            except requests.RequestException as e:
                last_err = f"{type(e).__name__}: {e}"
                break
            if r.status_code in (401, 403):
                raise FatalDeciderError(f"Jev 認証エラー HTTP {r.status_code}: {r.text[:200]}")
            if r.status_code in (429, 529) and attempt == 0:
                time.sleep(0.5)
                continue
            if r.status_code != 200:
                last_err = f"HTTP {r.status_code}: {r.text[:200]}"
                break
            return self._parse(r.json(), time.time() - t0)
        return Decision("hold", 0.0, {}, time.time() - t0, "jev", error=last_err)

    def _parse(self, body: dict, latency: float) -> Decision:
        answers = body.get("answers") or body.get("choices") or {}
        na = answers.get("next_action", {})
        gr = answers.get("goal_reached", {})
        action = na.get("choice", "hold")
        if action not in self.catalog:
            return Decision("hold", 0.0, {}, latency, "jev", raw=body, error=f"未知のアクション {action!r}")
        usage = body.get("usage", {}) or {}
        in_tok = int(usage.get("input_tokens", 0) or 0)
        return Decision(
            action=action,
            confidence=float(na.get("confidence", 0.0) or 0.0),
            probabilities={k: float(v) for k, v in (na.get("probabilities") or {}).items()},
            latency_s=latency,
            source="jev",
            input_tokens=in_tok,
            cost_usd=in_tok * JEV_PRICE_USD_PER_INPUT_TOKEN,
            done_prob=(float(gr["noul"]) if "noul" in gr else None),
            raw=body,
        )


# ----------------------------------------------------------------------------------------------
# 腕の抽象（実機 / 仮想）
# ----------------------------------------------------------------------------------------------


class VirtualArm:
    """--no_robot 用。initial_pose YAML の角度から始め、指令をそのまま現在角に反映する。通電しない。

    sag_deg_s を与えると、その関節は指令に関わらず毎 tick 一定速度で垂れる（重力負けの模擬）。
    追従誤差ガードの回帰試験用。
    """

    is_connected = True
    bus = None

    def __init__(self, initial_pose_deg: Dict[str, float], sag_deg_s: Optional[Dict[str, float]] = None, fps: float = 30.0):
        self.q = {k: 0.0 for k in STATE_KEYS}
        for k, v in initial_pose_deg.items():
            key = k if k.endswith(".pos") else f"{k}.pos"
            if key in self.q:
                self.q[key] = float(v)
        self.sag_per_tick = {}
        for k, v in (sag_deg_s or {}).items():
            key = k if k.endswith(".pos") else f"{k}.pos"
            if key in self.q:
                self.sag_per_tick[key] = float(v) / fps

    def connect(self):
        pass

    def disconnect(self):
        pass

    def get_observation(self) -> Dict[str, float]:
        return dict(self.q)

    def send_action(self, action: Dict[str, float]) -> Dict[str, float]:
        # 垂れ関節は指令に追従せず一定速度で落ちる（重力に完全に負けた状態の模擬）。他は指令どおり
        self.q.update({k: float(v) for k, v in action.items() if k in self.q and k not in self.sag_per_tick})
        for k, s in self.sag_per_tick.items():
            self.q[k] -= s
        return dict(action)


class SafeRunner:
    """
    どの終了経路でも torque disable → disconnect に到達させる（g4/eval_openarm.py の同名クラスを複製）。
    ソフト層は保証ではない。保証層は物理（DC PSU 遮断＋人間）にしか置けない。
    """

    def __init__(self, robot):
        self.robot = robot
        self._stopping = False
        self.stop_reason = ""

    def install_signal_handlers(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, self._on_signal)

    def _on_signal(self, signum, _frame):
        if self._stopping:
            logging.error("2回目のシグナル受信 → 即時終了する。腕の通電を目視で確認すること。")
            sys.exit(130)
        self.request_stop(f"signal {signum}")

    def request_stop(self, reason: str):
        if not self._stopping:
            self._stopping = True
            self.stop_reason = reason
            logging.warning(f"停止要求（{reason}）→ 停止処理に入る（torque disable まで待つ）")

    @property
    def should_stop(self) -> bool:
        return self._stopping

    def shutdown(self):
        disable_ok = False
        bus = getattr(self.robot, "bus", None)
        if bus is not None:
            try:
                bus.disable_torque()
                disable_ok = True
                logging.info("torque disable 成功")
            except Exception as e:
                logging.error(
                    f"**torque disable に失敗した（{type(e).__name__}: {e}）。"
                    "error latch の可能性がある。直ちに power cycle（DC PSU OFF→10秒→ON）を行うこと。**"
                )
        try:
            if getattr(self.robot, "is_connected", False):
                self.robot.disconnect()
            elif bus is not None and getattr(bus, "is_connected", False):
                bus.disconnect(disable_torque=True)
        except Exception:
            logging.exception("disconnect に失敗した。**直ちに power cycle を行うこと。**")
            raise
        finally:
            if isinstance(self.robot, VirtualArm):
                logging.info("仮想腕を終了した（通電なし）")
            elif disable_ok:
                logging.info("後片付け完了。**表示に関わらず腕の脱力を物理確認すること**")
            else:
                logging.error("**torque が落ちた確証が無い。腕に触れる前に脱力を確認し、疑わしければ power cycle。**")


# ----------------------------------------------------------------------------------------------
# 設定
# ----------------------------------------------------------------------------------------------


@dataclass
class JevArmConfig:
    robot: Optional[RobotConfig] = None

    decider: str = "rules"  # "rules" | "jev"
    goal: str = "指定した関節オフセットに到達する"
    # 開始姿勢（接続直後の実測 or 仮想腕の initial_pose）に対する目標オフセット [deg]。例 '{"joint_4": 10.0}'
    target_offsets_deg: Dict[str, float] = field(default_factory=dict)
    step_deg: float = 5.0
    tol_deg: float = 2.0
    enable_gripper: bool = False

    decide_hz: float = 2.0
    fps: float = 30.0
    max_decisions: int = 60
    confidence_min: float = 0.6
    lease_s: float = 3.0
    budget_usd: float = 2.0
    estop_path: str = "/tmp/openarm_estop"
    # 追従誤差ガード [deg]: 関節 1..7 のどれかで |実測 − q_des| がこれを超えたら停止処理へ（重力負け・接触・モータ異常）
    track_err_max_deg: float = 8.0

    jev_model: str = "jev-latest"
    jev_timeout_s: float = 1.5

    no_robot: bool = False
    initial_pose_path: Optional[str] = None  # no_robot 時の仮想腕の開始姿勢 YAML
    # no_robot 時のみ: 指定関節を毎秒この角度だけ垂らす（重力負けの模擬・ガードの回帰試験用）。例 '{"joint_7": 30.0}'
    no_robot_sag_deg_s: Dict[str, float] = field(default_factory=dict)
    dry_run: bool = True
    skip_pose_confirm: bool = False

    run_dir: Optional[str] = None


# ----------------------------------------------------------------------------------------------
# 本体
# ----------------------------------------------------------------------------------------------


def load_pose_yaml(path: str) -> Dict[str, float]:
    with open(path) as f:
        d = yaml.safe_load(f) or {}
    return {k: float(v) for k, v in d.items()}


def apply_action(action: str, current: Dict[str, float], step_deg: float, gripper_step: float = 5.0) -> Dict[str, float]:
    """アクション名 → 目標角 dict（現在角ベース）。hold/done は現在角。"""
    target = dict(current)
    if action.startswith("j") and ("_plus" in action or "_minus" in action):
        idx = int(action[1:action.index("_")])
        key = f"joint_{idx}.pos"
        target[key] = current[key] + (step_deg if action.endswith("_plus") else -step_deg)
    elif action == "gripper_open":
        target[GRIPPER_KEY] = current[GRIPPER_KEY] + gripper_step
    elif action == "gripper_close":
        target[GRIPPER_KEY] = current[GRIPPER_KEY] - gripper_step
    return target


# 2026-09-20 に整列後の残差 × 整列 kp から逆算した、開始姿勢（initial_pose_right.yaml）での重力負荷の実測値 [Nm]。
# 復元トルク上限（kp × clamp）がこれを下回る関節は「hold で垂れる」。姿勢が変われば負荷も変わるので目安。
MEASURED_GRAVITY_LOAD_NM = {"joint_1": 6.5, "joint_2": 6.5, "joint_3": 6.5, "joint_4": 6.5,
                            "joint_5": 1.2, "joint_6": 1.2, "joint_7": 1.2, "gripper": 0.0}


def log_torque_caps(robot_cfg, max_rel) -> None:
    """MIT 位置制御では clamp が復元トルクの上限になる（τ = kp × (cmd − present)、|cmd − present| ≤ clamp）。関節ごとに表示・警告する。"""
    import math

    kp = getattr(robot_cfg, "position_kp", None)
    motors = [k.removesuffix(".pos") for k in STATE_KEYS]
    if not isinstance(kp, list) or len(kp) != len(motors):
        logging.warning("position_kp が取得できないので復元トルク上限を計算できない")
        return
    lines, weak = [], []
    for i, m in enumerate(motors):
        clamp = max_rel[m] if isinstance(max_rel, dict) else max_rel
        cap = kp[i] * math.radians(clamp)
        need = MEASURED_GRAVITY_LOAD_NM.get(m, 0.0)
        lines.append(f"{m}: kp {kp[i]:g} × {clamp:g}° = {cap:.2f} Nm")
        if need > 0 and cap < need * 1.2:
            weak.append(f"{m} 上限 {cap:.2f} Nm < 実測負荷 {need:.1f} Nm ×1.2")
    logging.info("復元トルク上限（kp × max_relative_target）: " + " / ".join(lines))
    if weak:
        logging.warning(
            "**hold で垂れる可能性**（2026-09-20 実機で joint_7 が 74°→1° まで落ちた条件）: " + "; ".join(weak)
            + " → 関節別 dict（肩肘 2.0 / 手首 4.0 / グリッパ 1.0）を検討。追従誤差ガードが働くので走行は継続する"
        )


@draccus.wrap()
def main(cfg: JevArmConfig):
    init_logging()
    if cfg.run_dir:
        os.makedirs(cfg.run_dir, exist_ok=True)
        fh = logging.FileHandler(os.path.join(cfg.run_dir, "jev_arm.log"))
        fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        logging.getLogger().addHandler(fh)
    logging.info(pformat(asdict(cfg)))

    # --- 事前チェック（腕を繋ぐ前に落とす）---
    if not cfg.target_offsets_deg:
        raise SystemExit("target_offsets_deg が空。例: --target_offsets_deg='{\"joint_4\": 10.0}'")
    for k in cfg.target_offsets_deg:
        if f"{k}.pos" not in JOINT_KEYS and k not in JOINT_KEYS:
            raise SystemExit(f"target_offsets_deg のキー {k!r} は joint_1..joint_7 のいずれかにすること")
    if cfg.decide_hz <= 0 or cfg.fps <= 0 or cfg.decide_hz > cfg.fps:
        raise SystemExit("0 < decide_hz <= fps にすること")
    if not cfg.no_robot:
        if cfg.robot is None:
            raise SystemExit("--robot.type=openarm_follower ... を指定するか --no_robot=true にすること")
        max_rel = getattr(cfg.robot, "max_relative_target", None)
        if not isinstance(max_rel, (float, dict)):
            raise SystemExit("--robot.max_relative_target を必ず指定すること（小数 or 関節別 dict・安全プロトコル §0）")
        if isinstance(max_rel, dict):
            motors = [k.removesuffix(".pos") for k in STATE_KEYS]
            if set(max_rel) != set(motors):
                raise SystemExit(f"max_relative_target の dict は 8 関節すべて {motors} をキーに持つこと（lerobot 側の制約）")
            max_rel = {k: float(v) for k, v in max_rel.items()}
        if cfg.step_deg > 10.0:
            raise SystemExit("step_deg は 10 度以下にすること（段A の上限）")
        log_torque_caps(cfg.robot, max_rel)
    if os.path.exists(cfg.estop_path):
        raise SystemExit(f"E-stop ファイル {cfg.estop_path} が既に存在する。意図した停止なら削除してから再実行すること")

    catalog = build_action_catalog(cfg.step_deg, cfg.enable_gripper)
    if cfg.decider == "rules":
        decider = RulesDecider(catalog, cfg.tol_deg)
    elif cfg.decider == "jev":
        try:
            decider = JevDecider(catalog, cfg.jev_model, cfg.jev_timeout_s)
        except FatalDeciderError as e:
            raise SystemExit(str(e))
    else:
        raise SystemExit(f"decider={cfg.decider!r} は rules | jev のいずれか")
    logging.info(f"decider = {cfg.decider}  actions = {list(catalog)}")

    # --- 腕 ---
    if cfg.no_robot:
        pose = load_pose_yaml(cfg.initial_pose_path) if cfg.initial_pose_path else {}
        robot = VirtualArm(pose, cfg.no_robot_sag_deg_s, cfg.fps)
        logging.warning("=== NO-ROBOT モード: 仮想腕。通電しない ===")
        if cfg.no_robot_sag_deg_s:
            logging.warning(f"仮想腕に重力負けを模擬: {cfg.no_robot_sag_deg_s} deg/s")
    else:
        if cfg.dry_run:
            logging.warning("=== DRY-RUN モード: 接続（通電・自動整列あり）するが send_action は送らない ===")
        if not cfg.skip_pose_confirm:
            try:
                input(
                    "\n  [接続前チェック] 腕はレスト姿勢か／可動域に人・物は無いか／DC PSU に手が届くか\n"
                    "  接続すると通電し約2.2秒の自動整列で腕が動く。続行 ENTER・中止 Ctrl-C: "
                )
            except (EOFError, KeyboardInterrupt):
                raise SystemExit("\n接続前チェックで中止した（通電していない）")
        robot = make_robot_from_config(cfg.robot)

    runner = SafeRunner(robot)
    runner.install_signal_handlers()

    dec_f = traj_w = traj_f = None
    if cfg.run_dir:
        dec_f = open(os.path.join(cfg.run_dir, "decisions.jsonl"), "a")
        traj_f = open(os.path.join(cfg.run_dir, "traj.csv"), "w", newline="")
        traj_w = csv.writer(traj_f)
        traj_w.writerow(["tick", "t_sec", "decision_i", "action"] + [f"present.{k}" for k in STATE_KEYS] + [f"cmd.{k}" for k in STATE_KEYS])

    # 判断は別スレッドで取り、実行層はブロックしない（先行事例の「同期呼び出しで止まる」を避ける）
    pending: Dict[str, Any] = {"decision": None, "error": None}
    lock = threading.Lock()

    def decide_async(state: dict):
        def _run():
            try:
                d = decider.decide(state)
                with lock:
                    pending["decision"] = d
            except Exception as e:  # FatalDeciderError を含む
                with lock:
                    pending["error"] = e
        threading.Thread(target=_run, daemon=True).start()

    summary = {"decisions": 0, "holds": 0, "lease_misses": 0, "cost_usd": 0.0, "latencies": []}
    start_q = None
    try:
        robot.connect()
        obs = robot.get_observation()
        current = {k: float(obs[k]) for k in STATE_KEYS}
        start_q = dict(current)
        target_q = dict(start_q)
        for k, v in cfg.target_offsets_deg.items():
            key = k if k.endswith(".pos") else f"{k}.pos"
            target_q[key] = start_q[key] + float(v)
        logging.info("開始角: " + ", ".join(f"{k}={v:.2f}" for k, v in start_q.items()))
        logging.info("目標角: " + ", ".join(f"{k}={v:.2f}" for k, v in target_q.items() if target_q[k] != start_q[k]))

        # 意図姿勢。判断（アクション）だけがこれを更新する。補間の起点・hold の送信値・隙間 tick の送信値はすべてこれ。
        # 実測 current に再基準化しない（追従誤差は「消す」のでなく「検知して止める」）。
        q_des = dict(current)
        last_intended = dict(current)  # 前 tick に送った指令（クランプ前・補間値）。追従誤差ガードの比較対象
        last_actions: List[str] = []
        decision_i = 0
        tick = 0
        t_start = time.time()
        exec_plan: List[Dict[str, float]] = []  # 補間済み目標列（実行中の1判断分）
        state: Optional[dict] = None
        cur_action = "hold"
        decide_period = 1.0 / cfg.decide_hz
        n_interp = max(1, int(round(cfg.fps * decide_period)))
        last_decision_t = time.time()
        waiting = False

        while not runner.should_stop:
            tic = time.time()

            # ソフト E-stop（毎 tick）
            if os.path.exists(cfg.estop_path):
                runner.request_stop(f"E-stop file {cfg.estop_path}")
                break

            obs = robot.get_observation()
            current = {k: float(obs[k]) for k in STATE_KEYS}

            # 追従誤差ガード（毎 tick）: 前 tick の指令に対して実測が離れすぎていたら止める
            # （hold が保持になっていない＝重力負け・接触・モータ異常。補間中の意図的な先行分は含まれない）
            # dry-run（実機に送らない）では追従自体が起きないので対象外
            worst_k = max(JOINT_KEYS, key=lambda k: abs(current[k] - last_intended[k]))
            worst_err = current[worst_k] - last_intended[worst_k]
            guard_active = not (cfg.dry_run and not cfg.no_robot)
            if guard_active and abs(worst_err) > cfg.track_err_max_deg:
                runner.request_stop(
                    f"追従誤差 {worst_k} {worst_err:+.2f}° > {cfg.track_err_max_deg}°"
                    f"（実測 {current[worst_k]:.2f} / 指令 {last_intended[worst_k]:.2f} / 意図 {q_des[worst_k]:.2f}）"
                )
                break

            # --- 判断層: 実行計画が空なら次の判断を取りに行く ---
            if not exec_plan:
                with lock:
                    d = pending["decision"]
                    err = pending["error"]
                    pending["decision"] = None
                    pending["error"] = None
                if err is not None:
                    raise err
                if d is None and not waiting:
                    if summary["cost_usd"] >= cfg.budget_usd:
                        runner.request_stop(f"予算 {cfg.budget_usd} USD 到達")
                        break
                    state = {
                        "goal": cfg.goal,
                        "step": decision_i,
                        "q_deg": {k: round(v, 2) for k, v in current.items()},
                        "target_q_deg": {k: round(v, 2) for k, v in target_q.items()},
                        "error_deg": {k.split(".")[0]: round(target_q[k] - current[k], 2) for k in JOINT_KEYS},
                        "tol_deg": cfg.tol_deg,
                        "step_deg": cfg.step_deg,
                        "last_actions": last_actions[-5:],
                    }
                    decide_async(state)
                    waiting = True
                    wait_t0 = time.time()
                elif d is None and waiting:
                    if time.time() - wait_t0 > cfg.lease_s:
                        summary["lease_misses"] += 1
                        logging.warning(f"リース {cfg.lease_s}s 超過 → hold 継続（{summary['lease_misses']} 回目）")
                        wait_t0 = time.time()
                if d is not None:
                    waiting = False
                    decision_i += 1
                    summary["decisions"] += 1
                    summary["cost_usd"] += d.cost_usd
                    summary["latencies"].append(d.latency_s)
                    act = d.action
                    reason = ""
                    if d.error:
                        act, reason = "hold", f"error={d.error}"
                    elif d.source == "jev" and d.confidence < cfg.confidence_min and act != "hold":
                        act, reason = "hold", f"confidence {d.confidence:.2f} < {cfg.confidence_min}"
                    if act == "hold":
                        summary["holds"] += 1
                    logging.info(
                        f"[decision {decision_i}] {d.source} → {d.action} (conf {d.confidence:.2f}, {d.latency_s*1000:.0f}ms"
                        f"{', done_p=%.2f' % d.done_prob if d.done_prob is not None else ''}) → 実行 {act} {reason}"
                    )
                    if dec_f:
                        dec_f.write(json.dumps({
                            "i": decision_i, "t": round(time.time() - t_start, 3), "source": d.source,
                            "chosen": d.action, "executed": act, "confidence": d.confidence,
                            "probabilities": d.probabilities, "latency_s": round(d.latency_s, 4),
                            "input_tokens": d.input_tokens, "cost_usd": d.cost_usd, "done_prob": d.done_prob,
                            "error": d.error, "state_error_deg": state["error_deg"] if state else None,
                        }, ensure_ascii=False) + "\n")
                    last_actions.append(act)
                    cur_action = act
                    if act == "done":
                        logging.info("done を選択 → 正常終了する")
                        break
                    if decision_i >= cfg.max_decisions:
                        logging.info(f"max_decisions={cfg.max_decisions} 到達 → 終了")
                        break
                    # 目標は意図姿勢 q_des に対して定義する（実測 current でなく）
                    goal = apply_action(act, q_des, cfg.step_deg)
                    if act == "hold":
                        exec_plan = [dict(q_des)] * n_interp
                    else:
                        q_from = dict(q_des)
                        exec_plan = [
                            {k: q_from[k] + (goal[k] - q_from[k]) * (j / n_interp) for k in STATE_KEYS}
                            for j in range(1, n_interp + 1)
                        ]
                        q_des = goal  # 計画の終点＝新しい意図姿勢（補間中の隙間 tick も終点を送る）

            # --- 実行層: 1 tick 分の目標を送る（計画が無ければ q_des を送り続ける＝hold）---
            cmd = exec_plan.pop(0) if exec_plan else dict(q_des)
            if traj_w:
                traj_w.writerow([tick, round(time.time() - t_start, 3), decision_i, cur_action]
                                + [round(current[k], 4) for k in STATE_KEYS] + [round(cmd[k], 4) for k in STATE_KEYS])
            if cfg.dry_run and not cfg.no_robot:
                if tick % int(cfg.fps) == 0:
                    logging.info(f"[dry-run] tick={tick} cmd={ {k: round(v, 2) for k, v in cmd.items()} }")
            else:
                # 戻り値（クランプ後の実送信値）は次 tick の基準にしない。基準は常に意図側（q_des / 補間値）
                robot.send_action(cmd)
            last_intended = cmd
            tick += 1

            dt = time.time() - tic
            if dt < 1.0 / cfg.fps:
                time.sleep(1.0 / cfg.fps - dt)

    except FatalDeciderError as e:
        logging.error(f"判断層の致命的エラー → 停止: {e}")
    except Exception:
        logging.exception("実行中に例外 → 停止処理へ")
        raise
    finally:
        try:
            if start_q is not None:
                obs = robot.get_observation()
                logging.info("=== 関節角の変化（開始 → 終了）===")
                for k in STATE_KEYS:
                    logging.info(f"  {k:14} {start_q[k]:8.2f} → {float(obs[k]):8.2f}  ({float(obs[k]) - start_q[k]:+7.2f})")
            lat = summary["latencies"]
            logging.info(
                f"判断 {summary['decisions']} 回・hold {summary['holds']}・リース超過 {summary['lease_misses']}・"
                f"費用 {summary['cost_usd']:.4f} USD・応答中央値 {sorted(lat)[len(lat)//2]*1000:.0f}ms" if lat else
                f"判断 0 回・費用 0 USD"
            )
            if runner.stop_reason:
                logging.info(f"停止理由: {runner.stop_reason}")
        except Exception:
            pass
        for f_ in (dec_f, traj_f):
            if f_:
                try:
                    f_.close()
                except Exception:
                    pass
        runner.shutdown()


if __name__ == "__main__":
    main()
