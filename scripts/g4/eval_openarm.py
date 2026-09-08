#!/usr/bin/env python3
"""
eval_openarm.py — GR00T N1.7 推論 → OpenArm 実機ブリッジ（G4）

`gr00t/eval/real_robot/SO100/eval_so100.py` を雛形に、OpenArm（7関節＋グリッパ＝8次元）用へ
置き換えたもの。差分の要点は以下3つ:

  1. **8次元化**: SO100 は 5関節＋グリッパ＝6次元。OpenArm は joint_1..7 + gripper = 8次元。
     modality.json の分割（single_arm 0:7 / gripper 7:8）と一致させる。
  2. **カメラ1本**: SO100 は front+wrist の2本。OpenArm は chest 単眼のみ。
  3. **安全機構の追加（本ファイルの主目的）**: 雛形は `while True` の無限ループで
     try/finally も signal handler も無く、Ctrl-C で torque disable が走らない。
     安全プロトコル TODO#2「rollout の停止は未定義」の実体がこれ。本実装では
     どの終了経路でも `robot.disconnect()` に到達させる。

安全上の前提（[[openarm-safety-protocol]]）:
  - `OpenArmFollowerConfig.disable_torque_on_disconnect` は既定 True。
    → `disconnect()` に到達しさえすればトルクは落ちる。**到達を保証するのが本実装の役割**。
  - `max_relative_target` は既定 None（クランプ無効）。**必ず明示指定する**（本実装は必須引数扱い）。
  - ただし Damiao の error latch 時はソフト disable が効かない。
    **唯一確実な脱力は power cycle（DC PSU OFF→10秒→ON）**。実行時は電源に手が届く位置で。

使い方（PolicyServer を別プロセスで起動しておくこと）:
    python eval_openarm.py \
        --robot.type=openarm_follower \
        --robot.port=<CAN_IF> \
        --robot.side=right \
        --robot.id=follower_right \
        --robot.cameras='{"chest": {"type": "intelrealsense", "serial_number_or_name": "344422070898", "width": 640, "height": 480, "fps": 30}}' \
        --robot.max_relative_target=3 \
        --lang_instruction="Pick up the bottle and place it in the bin" \
        --max-steps=200 \
        --dry-run=true          # ← 初回は必ず dry-run で軌道を目視してから外す
    # --lang_instruction は必須（収集時の single_task と同じ文。2026-09-08 から既定値なし）
"""

import csv
import io
import logging
import os
import signal
import sys
import time
from dataclasses import asdict, dataclass
from pprint import pformat
from typing import Any, Dict, List

import draccus
import msgpack
import numpy as np
import zmq

from lerobot.robots import (  # noqa: F401
    Robot,
    RobotConfig,
    make_robot_from_config,
    openarm_follower,
)

# カメラ実装は import して初めて draccus の choice registry に登録される。
# これが無いと `--robot.cameras='{"chest": {"type": "intelrealsense", ...}}'` が
# `Couldn't find a choice class for 'intelrealsense'` で落ちる（2026-07-22 実際に踏んだ）。
# lerobot-record が通るのは内部でこれらを import しているため。
import lerobot.cameras.opencv  # noqa: F401
import lerobot.cameras.realsense  # noqa: F401
from lerobot.utils.utils import init_logging, log_say


# =============================================================================
# PolicyClient（gr00t 非依存の最小実装）
# =============================================================================
# Thor では gr00t venv（推論・torch 一式）と lerobot venv（実機制御）が分離しており、
# 相互に import できない。しかし PolicyServer/Client は ZeroMQ REQ/REP + msgpack の
# 単純なプロトコルなので、クライアント側は gr00t を入れずに再実装できる。
# 設計書 §5.3 の「server-client（ZeroMQ）分離」構成そのもので、
# ロボット側を薄いクライアントに保てる（torch を実機側に持ち込まない）。
#
# 対向実装: gr00t/policy/server_client.py の MsgSerializer / PolicyClient
# ndarray は np.save でバイト列にして msgpack に載せる形式（allow_pickle=False）。


class _MsgSerializer:
    """gr00t 側 MsgSerializer と互換のシリアライザ（ndarray 部分のみ）。"""

    @staticmethod
    def to_bytes(data: Any) -> bytes:
        return msgpack.packb(data, default=_MsgSerializer._encode)

    @staticmethod
    def from_bytes(data: bytes) -> Any:
        return msgpack.unpackb(data, object_hook=_MsgSerializer._decode)

    @staticmethod
    def _encode(obj):
        if isinstance(obj, np.ndarray):
            buf = io.BytesIO()
            np.save(buf, obj, allow_pickle=False)
            return {"__ndarray_class__": True, "as_npy": buf.getvalue()}
        return obj

    @staticmethod
    def _decode(obj):
        if isinstance(obj, dict) and "__ndarray_class__" in obj:
            return np.load(io.BytesIO(obj["as_npy"]), allow_pickle=False)
        return obj


class PolicyClient:
    """PolicyServer への薄いクライアント。"""

    def __init__(self, host: str = "localhost", port: int = 5555, timeout_ms: int = 15000,
                 api_token: str | None = None):
        self.host, self.port, self.timeout_ms = host, port, timeout_ms
        self.api_token = api_token
        self.context = zmq.Context()
        self._init_socket()

    def _init_socket(self):
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self.socket.connect(f"tcp://{self.host}:{self.port}")

    def call_endpoint(self, endpoint: str, data: dict | None = None, requires_input: bool = True):
        request: dict = {"endpoint": endpoint}
        if requires_input:
            request["data"] = data
        if self.api_token:
            request["api_token"] = self.api_token
        try:
            self.socket.send(_MsgSerializer.to_bytes(request))
            return _MsgSerializer.from_bytes(self.socket.recv())
        except zmq.error.Again as e:
            # タイムアウト時はソケットを作り直さないと以後の送信が詰まる（REQ/REP の制約）
            self._init_socket()
            raise TimeoutError(
                f"PolicyServer 応答なし（{self.timeout_ms}ms）。サーバ起動と host/port を確認すること。"
            ) from e

    def ping(self) -> bool:
        try:
            self.call_endpoint("ping", requires_input=False)
            return True
        except Exception:
            return False

    def get_action(self, observations: dict):
        """
        サーバとの契約（`gr00t/policy/server_client.py` 実読・2026-07-22）:
          - サーバは `handler.handler(**request["data"])` と **キーワード展開して呼ぶ**ため、
            `data` には観測そのものでなく **`{"observation": obs}`** を入れる
            （`policy.get_action(self, observation, options=None)` に対応）
          - 正常時の戻り値は **`(action, info)` のタプル**
          - 例外時は **`{"error": "..."}`** が返る（成功と同じ経路で来るので明示的に検出する）
        """
        resp = self.call_endpoint("get_action", {"observation": observations})

        if isinstance(resp, dict) and "error" in resp and len(resp) == 1:
            raise RuntimeError(f"PolicyServer 側でエラー: {resp['error']}")

        # (action, info) のタプル / action 単体 のどちらでも受ける
        if isinstance(resp, (list, tuple)) and len(resp) == 2:
            action, info = resp
        else:
            action, info = resp, {}

        if not isinstance(action, dict):
            raise RuntimeError(
                f"PolicyServer の応答が想定外（type={type(action).__name__}）: {str(action)[:200]}"
            )
        return action, info


def recursive_add_extra_dim(obs: Dict) -> Dict:
    """(B, T) 次元を先頭に足す。雛形 eval_so100.py と同一実装。"""
    out = {}
    for k, v in obs.items():
        if isinstance(v, dict):
            out[k] = recursive_add_extra_dim(v)
        elif isinstance(v, np.ndarray):
            out[k] = v[np.newaxis, ...]
        else:
            out[k] = [v]
    return out


class OpenArmAdapter:
    """
    Raw robot observation ↔ GR00T VLA 入出力の変換器。

    SO100Adapter との差分:
      - robot_state_keys が 8 個（joint_1..7 + gripper）
      - camera_keys が chest 単眼
      - state 分割が [:7] / [7:8]（modality.json の single_arm 0:7 / gripper 7:8 と一致）
    """

    # 収集データ meta/info.json の features names と完全一致させること。
    # ここがずれると学習時と推論時で関節の対応が入れ替わり、危険な挙動になる。
    ROBOT_STATE_KEYS = [
        "joint_1.pos",
        "joint_2.pos",
        "joint_3.pos",
        "joint_4.pos",
        "joint_5.pos",
        "joint_6.pos",
        "joint_7.pos",
        "gripper.pos",
    ]
    CAMERA_KEYS = ["chest"]

    def __init__(self, policy_client: PolicyClient):
        self.policy = policy_client

    def obs_to_policy_inputs(self, obs: Dict[str, Any]) -> Dict:
        model_obs = {}
        model_obs["video"] = {k: obs[k] for k in self.CAMERA_KEYS}

        state = np.array([obs[k] for k in self.ROBOT_STATE_KEYS], dtype=np.float32)
        model_obs["state"] = {
            "single_arm": state[:7],  # (7,)
            "gripper": state[7:8],  # (1,)
        }
        model_obs["language"] = {"annotation.human.task_description": obs["lang"]}

        model_obs = recursive_add_extra_dim(model_obs)
        model_obs = recursive_add_extra_dim(model_obs)
        return model_obs

    def decode_action_chunk(self, chunk: Dict, t: int) -> Dict[str, float]:
        """chunk["single_arm"]: (B, T, 7) / chunk["gripper"]: (B, T, 1) → モータ指令 dict"""
        single_arm = chunk["single_arm"][0][t]  # (7,)
        gripper = chunk["gripper"][0][t]  # (1,)
        full = np.concatenate([single_arm, gripper], axis=0)  # (8,)
        return {name: float(full[i]) for i, name in enumerate(self.ROBOT_STATE_KEYS)}

    def get_action(self, obs: Dict) -> List[Dict[str, float]]:
        model_input = self.obs_to_policy_inputs(obs)
        action_chunk, _info = self.policy.get_action(model_input)
        any_key = next(iter(action_chunk.keys()))
        horizon = action_chunk[any_key].shape[1]  # (B, T, D) → T
        return [self.decode_action_chunk(action_chunk, t) for t in range(horizon)]


@dataclass
class EvalConfig:
    robot: RobotConfig
    policy_host: str = "localhost"
    policy_port: int = 5555

    # 1 回の推論から実際に流すステップ数。設計書 §6.4 の暫定値は 16 だが、
    # 初回は短くして「予測が外れても長く走り続けない」ようにする。
    action_horizon: int = 8

    # 収集時の single_task と完全に一致させること（学習時の言語条件と揃える）。
    # 2026-09-08: 既定値（ボトル用の文）を廃止して**必須**にした。既定値に頼ると別タスクの
    # モデルを回しても指示文がボトル用のまま送られる（実測: ボトル 13 本・テープ 13 本の
    # 全 rollout が同一の既定文だった。テープは収集時 "vinyl tape"・推論時 "bottle"）。
    # タスクごとに --lang_instruction="<収集時の single_task と同じ文>" を必ず渡す。
    lang_instruction: str | None = None

    fps: int = 30
    max_steps: int = 200  # 無限ループを避ける（雛形の while True を廃止）

    # true の間は推論と軌道表示だけ行い、send_action は呼ばない。
    # **注意: dry_run=true でも robot.connect() は通電し自動整列で腕が動く**（A-2 参照）。
    # 「dry-run だから実機に何も起きない」ではない。
    dry_run: bool = True

    # B-1: 初回指令が現在角からこの値を超えて乖離していたら中止する。
    # RELATIVE/ABSOLUTE の解釈違いを送信前に機械的に検出するためのゲート。
    first_action_max_dev: float = 20.0

    # A-2 の接続前チェックを飛ばす（自動化時のみ。実機を人が見ていない状況では使わない）
    skip_pose_confirm: bool = False

    # 運転時のカメラ画像を定期保存する（学習分布との突き合わせ用）。
    # 2026-07-22 に「運転時の画像が学習分布から外れていたか」を後から検証できず、
    # 機序の確認が推論に留まった。画像が残っていれば offline で probe にかけられる。
    frame_log_dir: str | None = None
    # N **チャンク** ごとに1枚（既定 1 = 毎チャンク）。
    # 2026-07-22 訂正: 以前は `step % frame_log_every` で判定していたが、この判定は
    # chunk 先頭でしか評価されず、step は必ず action_horizon の倍数になる。
    # そのため実際の保存間隔は lcm(action_horizon, frame_log_every) step で、
    # 既定（horizon=8・every=25）では **200 step に1枚**——1回の実行で画像が
    # ほぼ残らなかった。チャンク数で数えるよう改めた。
    frame_log_every: int = 1

    # 毎ステップの関節角を CSV に記録する（実行どうしを数値で比較するため）。
    # クランプ警告からしか関節角を読めなかった問題への対処——クランプが発動しなくなると
    # 挙動が観測できなくなる（2026-07-22 に実際に発生）。
    trajectory_log: str | None = None

    # 記録のためだけに chunk 内でも観測を取り直す（既定 OFF）。
    #
    # chunk 実行はオープンループで、`robot.get_observation()` は chunk 先頭でしか
    # 呼ばれない。そのため既定のログでは chunk 内の `present.*` が前チャンクの
    # 使い回しになる（2026-07-22 実測: 150 行中 131 行が前行と完全一致）。
    # 画像も同じ地点でしか取れないため、**観測は chunk レート（実測 約1Hz）が上限**で、
    # ロールアウトを LeRobotDataset に変換しても teleop 収集（30fps）と同格にならない。
    #
    # このフラグを立てると、chunk 内の各ステップで記録用に観測を取り直す。
    # **読むだけでモータ指令経路には触れない**（失敗しても実行は継続する）。
    # ⚠ 観測取得は制御周期を消費する。実機で使う前に必ず dry-run で
    #   ステップ間隔が 1/fps に収まっているかを軌道ログで確認すること。
    log_obs_every_step: bool = False

    # 1試行1ディレクトリ。指定すると trajectory_log / frame_log_dir の既定値と
    # logging のファイル出力先をこの下へ集約する（ラッパー run_rollout.sh が切る）。
    # 明示的に trajectory_log / frame_log_dir を渡した場合はそちらを優先する。
    run_dir: str | None = None

    play_sounds: bool = False


class SafeRunner:
    """
    どの終了経路でも robot.disconnect() に到達させるための小さなラッパ。

    雛形 eval_so100.py には try/finally も signal handler も無く、Ctrl-C や例外で
    torque ON のまま抜ける。安全プロトコル §0 に「異常終了すると cleanup が走らず
    torque ON が残留（過熱リスク）・放置禁止」とある状態がこれにあたる。

    **ただしソフト層は保証ではない**（2026-07-21 安全レビューの結論）。
    Damiao の error latch 時は disable が効かず、SIGKILL / OOM / カーネルフリーズでは
    このクラスは動かない。**保証層は物理（DC PSU の遮断＋人間）にしか置けない**。
    本クラスの役割は「人間の介入が必要になる確率と、失敗時のエネルギーを最小化する」こと。
    """

    def __init__(self, robot: Robot):
        self.robot = robot
        self._stopping = False
        self._orig_handlers = {}

    def install_signal_handlers(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            self._orig_handlers[sig] = signal.getsignal(sig)
            signal.signal(sig, self._on_signal)

    def _on_signal(self, signum, _frame):
        # 2 回目の Ctrl-C は即座に落とす（1 回目の後片付けが固まった場合の逃げ道）
        if self._stopping:
            logging.error("2回目のシグナル受信 → 即時終了する。腕の通電を目視で確認すること。")
            sys.exit(130)
        self._stopping = True
        logging.warning(f"シグナル {signum} 受信 → 停止処理に入る（torque disable まで待つ）")

    @property
    def should_stop(self) -> bool:
        return self._stopping

    def shutdown(self):
        """
        必ず呼ばれる後片付け。

        **注意（B-3・2026-07-21 レビュー指摘）**: `robot.disconnect()` に任せるだけでは
        torque disable の失敗を検知できない。`DamiaoMotorsBus.disconnect()` は
        `disable_torque()` の例外を `logger.warning` で握りつぶして続行する実装のため
        （`lerobot/motors/damiao/damiao.py`）、**「disable に失敗した」まさにその場合に
        「disconnect 完了」と表示されてしまう**。
        そこで disconnect の前に `bus.disable_torque()` を直接呼び、その成否を自分で握る。
        """
        disable_ok = False

        # (1) まず torque disable を明示的に試み、成否を自分で把握する
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
        else:
            logging.warning("bus が取得できず torque disable を明示実行できなかった")

        # (2) 接続状態に応じて後片付け（connect() 途中で落ちた場合の二次例外を避ける）
        logging.info("robot.disconnect() を実行する")
        try:
            if getattr(self.robot, "is_connected", False):
                self.robot.disconnect()
            else:
                logging.warning("接続が未完了のまま終了する。腕の脱力を必ず目視確認すること。")
                if bus is not None and getattr(bus, "is_connected", False):
                    bus.disconnect(disable_torque=True)
        except Exception:
            logging.exception(
                "disconnect に失敗した。**直ちに power cycle（DC PSU OFF→10秒→ON）を行うこと。**"
            )
            raise
        finally:
            if disable_ok:
                logging.info(
                    "後片付け完了。**表示に関わらず腕の脱力を物理確認すること**"
                    "（軽く押す。動かなければ power cycle）。"
                )
            else:
                logging.error(
                    "**torque が落ちた確証が無い状態で終了する。腕に触れる前に脱力を確認し、"
                    "疑わしければ power cycle すること。放置は過熱リスクのため厳禁。**"
                )


@draccus.wrap()
def eval(cfg: EvalConfig):
    init_logging()

    # --- run_dir: 1試行1ディレクトリに traj/frame/log を集約する ---
    # ラッパー run_rollout.sh がタイムスタンプ付きで切る。ここが指定されると、
    # 明示されていない trajectory_log / frame_log_dir はこの下を既定にし、
    # logging も run_dir/eval.log にファイル保存する（console の tee とは別系統の保険）。
    # 実機の send_action 経路には触れない——記録の外付けのみ。
    if cfg.run_dir:
        os.makedirs(cfg.run_dir, exist_ok=True)
        if cfg.trajectory_log is None:
            cfg.trajectory_log = os.path.join(cfg.run_dir, "traj.csv")
        if cfg.frame_log_dir is None:
            cfg.frame_log_dir = os.path.join(cfg.run_dir, "frames")
        _fh = logging.FileHandler(os.path.join(cfg.run_dir, "eval.log"))
        _fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        logging.getLogger().addHandler(_fh)
        logging.info(f"run_dir = {cfg.run_dir}（traj/frame/log をここへ集約する）")

    # asdict ログは run_dir 展開の後に置く＝確定した traj/frame パスも記録に残す
    logging.info(pformat(asdict(cfg)))

    # --- 安全設定の事前チェック（実機を繋ぐ前に落とす） ---
    max_rel = getattr(cfg.robot, "max_relative_target", None)
    if max_rel is None:
        raise SystemExit(
            "max_relative_target が未設定。1ステップの移動量が無制限になり危険なので中止する。\n"
            "  --robot.max_relative_target=1.0 のように必ず指定すること（安全プロトコル §0）。"
        )

    # B-2: lerobot の ensure_safe_goal_position は `isinstance(max_relative_target, float)`
    # で分岐する（robots/utils.py L96）。**int を渡すと `raise TypeError` でクラッシュする**（2026-07-22 実コードで確認・素通りではなく fail-safe 側）。実機接続後に無用な例外を踏まないよう、
    # 小数であることをここで強制する（無用な TypeError を実機接続後に踏まない）。
    if not isinstance(max_rel, (float, dict)):
        raise SystemExit(
            f"max_relative_target が float でない（{type(max_rel).__name__}: {max_rel}）。\n"
            "  lerobot 側は isinstance(x, float) で判定するため int だとクランプが効かない。\n"
            "  --robot.max_relative_target=1.0 のように**小数**で指定すること。"
        )

    # 1 step あたりの上限 × fps ＝ 最大角速度。誤った目標へ向かった場合の到達速度になる。
    if isinstance(max_rel, float):
        logging.info(
            f"max_relative_target = {max_rel}（1ステップ上限）→ 最大 約{max_rel * cfg.fps:.0f}/s。"
            "初回実駆動は 1.0 程度まで落とすこと（レビュー B-2）。"
        )
    else:
        logging.info(f"max_relative_target = {max_rel}（関節ごと指定）")

    # B-3: 言語指示の未指定ゲート（2026-09-08）。既定値に頼ると学習時と違う指示文で推論する
    # 事故になる（テープ実機 13 本がボトル用の文のまま走った実例）。実機を繋ぐ前に落とす。
    if not cfg.lang_instruction or not cfg.lang_instruction.strip():
        raise SystemExit(
            "lang_instruction が未指定。学習時（収集時の single_task）と同じ指示文を必ず渡すこと。\n"
            '  例: --lang_instruction="Pick up the vinyl tape and place it in the bin"'
        )
    logging.info(f'lang_instruction = "{cfg.lang_instruction}"（収集時の single_task と一致していること）')

    if cfg.dry_run:
        logging.warning("=== DRY-RUN モード: 推論のみ実行し、実機へは指令を送らない ===")

    # --- 実機を繋ぐ「前」に PolicyServer との疎通を確認する ---
    # 腕を通電させてから通信不良に気づくのを避ける（通電時間を最小化する）。
    policy_client = PolicyClient(host=cfg.policy_host, port=cfg.policy_port)
    if not policy_client.ping():
        raise SystemExit(
            f"PolicyServer ({cfg.policy_host}:{cfg.policy_port}) に到達できない。\n"
            "  gr00t venv 側でサーバを起動してから再実行すること（実機はまだ繋いでいない）。"
        )
    logging.info("PolicyServer 疎通 OK")

    # --- A-2: 接続前の姿勢確認ゲート ---
    # robot.connect() は内部で bus.set_zero_position() を呼び、**接続時点の物理姿勢を
    # 座標系の原点に再設定する**（openarm_follower.py L150）。収集時と違う姿勢で接続すると
    # ①joint_limits が守る領域が物理的にずれ ②モデルの state/action フレームが系統的にずれる。
    # どちらも max_relative_target のクランプでは防げない種類の危険。
    # さらに connect() は enable_torque() → soft_move_to_position() を実行するため、
    # **dry-run であっても通電し、腕が自動整列で実際に動く**（約2.2秒）。
    if not cfg.skip_pose_confirm:
        try:
            input(
                "\n"
                "  [接続前チェック]\n"
                "   - 腕は標準レスト姿勢か（接続時の姿勢が原点として焼き付く）\n"
                "   - 可動域に人・物は無いか\n"
                "   - DC PSU のスイッチに手が届くか（唯一確実な停止手段）\n"
                "  接続すると通電し、約2.2秒の自動整列動作で腕が動く。\n"
                "  続行する場合は ENTER、中止は Ctrl-C: "
            )
        except (EOFError, KeyboardInterrupt):
            raise SystemExit("\n接続前チェックで中止した（実機は通電していない）")

    robot = make_robot_from_config(cfg.robot)
    runner = SafeRunner(robot)

    # A-1: signal handler を connect() より前に設置し、connect() 自体も try 内へ入れる。
    # connect() は torque ON と自動整列（実動作）を伴うため、ここが保護外だと
    # 整列中の Ctrl-C や calibrate() の input() 待ちで torque ON のまま抜けうる。
    runner.install_signal_handlers()

    # 軌道ログ（毎ステップの実測角と指令値）。クランプ警告に頼らず挙動を数値化する。
    traj_f = None
    traj_w = None
    if cfg.trajectory_log:
        traj_f = open(cfg.trajectory_log, "w", newline="")
        traj_w = csv.writer(traj_f)
        keys = OpenArmAdapter.ROBOT_STATE_KEYS
        traj_w.writerow(
            ["step", "chunk_i", "t_sec"]
            + [f"present.{k}" for k in keys]
            + [f"cmd.{k}" for k in keys]
        )
        logging.info(f"軌道ログを記録する: {cfg.trajectory_log}")

    try:
        robot.connect()
        log_say("Robot connected", cfg.play_sounds, blocking=True)

        policy = OpenArmAdapter(policy_client)
        log_say(f'Policy ready: "{cfg.lang_instruction}"', cfg.play_sounds, blocking=True)

        step = 0
        checked_first_action = False
        t_start = time.time()
        first_obs = None
        last_obs = None
        chunk_idx = 0

        def save_frame(source_obs, at_step: int) -> None:
            """運転時フレームを保存する（学習分布との突き合わせを後から可能にする）。

            保存名の番号は軌道ログ CSV の `step` 列と一致させる。
            変換スクリプト（convert_trajlog_to_lerobot.py）はこの対応で画像と行を結ぶ。
            """
            try:
                img = source_obs.get(OpenArmAdapter.CAMERA_KEYS[0])
                if img is not None:
                    os.makedirs(cfg.frame_log_dir, exist_ok=True)
                    np.save(os.path.join(cfg.frame_log_dir, f"frame_{at_step:05d}.npy"), img)
            except Exception:
                logging.warning("フレーム保存に失敗（実行は継続する）", exc_info=False)

        while step < cfg.max_steps and not runner.should_stop:
            obs = robot.get_observation()
            obs["lang"] = cfg.lang_instruction
            if first_obs is None:
                first_obs = {k: float(obs[k]) for k in OpenArmAdapter.ROBOT_STATE_KEYS if k in obs}
            last_obs = {k: float(obs[k]) for k in OpenArmAdapter.ROBOT_STATE_KEYS if k in obs}

            log_this_chunk = bool(cfg.frame_log_dir) and chunk_idx % cfg.frame_log_every == 0
            if log_this_chunk:
                save_frame(obs, step)

            t0 = time.time()
            actions = policy.get_action(obs)
            infer_s = time.time() - t0

            # --- B-1: 初回指令の乖離ゲート（送信前に1度だけ） ---
            # single_arm は RELATIVE 設定だが、実際に差分→絶対角の変換が走るかは
            # PolicyServer 側の use_relative_action フラグ次第。学習時と推論時でこの
            # フラグが食い違うと、差分値が絶対角として送られて腕が大きく動く。
            # サーバ設定を信用せず、**指令が現在角からどれだけ離れているか**で機械的に検出する。
            if not checked_first_action and actions:
                first = actions[0]
                # gripper は判定から除外する。開閉は本質的に大きな変位を伴い（実測で 18 程度）、
                # 座標系のずれとは無関係に閾値へ迫るため。腕の関節だけで判定すれば
                # RELATIVE/ABSOLUTE の取り違えもフレームずれも検出できる
                # （実測: 取り違えが無ければ関節の乖離は 0.03〜0.29 度に収まる）。
                dev_all = {k: abs(first[k] - float(obs[k])) for k in first if k in obs}
                dev = {k: v for k, v in dev_all.items() if not k.startswith("gripper")}
                worst = max(dev.values()) if dev else 0.0
                logging.info(
                    f"初回指令の現在角からの乖離: 最大 {worst:.2f}（閾値 {cfg.first_action_max_dev}・腕関節のみ）"
                    f" / gripper 参考値 {dev_all.get('gripper.pos', 0.0):.2f}"
                )
                if worst > cfg.first_action_max_dev:
                    raise SystemExit(
                        f"初回指令が現在角から {worst:.1f} 乖離している（閾値 {cfg.first_action_max_dev}）。\n"
                        "  RELATIVE/ABSOLUTE の解釈違い、または座標フレームのずれが疑われる。\n"
                        f"  内訳: { {k: round(v, 2) for k, v in dev.items()} }\n"
                        "  中止する（実機へは何も送っていない）。"
                    )
                checked_first_action = True

            for i, action_dict in enumerate(actions[: cfg.action_horizon]):
                if runner.should_stop:
                    logging.warning("停止要求を検知 → chunk 実行を中断する")
                    break

                tic = time.time()

                # --- 記録専用の観測（既定 OFF・log_obs_every_step のコメント参照）---
                # policy に渡す obs には触れない。ここで取り直した値はログにしか使わない。
                log_obs = obs
                if cfg.log_obs_every_step and i > 0:
                    try:
                        log_obs = robot.get_observation()
                    except Exception:
                        logging.warning("記録用の観測取得に失敗（実行は継続する）", exc_info=False)
                    else:
                        if log_this_chunk:
                            save_frame(log_obs, step)

                if traj_w is not None:
                    keys = OpenArmAdapter.ROBOT_STATE_KEYS
                    traj_w.writerow(
                        [step, i, round(time.time() - t_start, 3)]
                        + [round(float(log_obs.get(k, float("nan"))), 4) for k in keys]
                        + [round(float(action_dict[k]), 4) for k in keys]
                    )

                if cfg.dry_run:
                    logging.info(f"[dry-run] step={step} chunk[{i}] {action_dict}")
                else:
                    robot.send_action(action_dict)

                step += 1
                if step >= cfg.max_steps:
                    logging.info(f"max_steps={cfg.max_steps} に到達 → 正常終了する")
                    break

                dt = time.time() - tic
                if dt < 1.0 / cfg.fps:
                    time.sleep(1.0 / cfg.fps - dt)

            chunk_idx += 1
            logging.info(f"step={step}/{cfg.max_steps} 推論 {infer_s*1000:.0f}ms")

    except Exception:
        logging.exception("実行中に例外が発生した → 停止処理へ")
        raise
    finally:
        # --- 実際にどれだけ動いたかの要約（目視では差が分からないため必ず数値で残す） ---
        try:
            if first_obs and last_obs:
                logging.info("=== 関節角の変化（開始 → 終了） ===")
                for k in OpenArmAdapter.ROBOT_STATE_KEYS:
                    if k in first_obs and k in last_obs:
                        d = last_obs[k] - first_obs[k]
                        logging.info(
                            f"  {k:14} {first_obs[k]:8.2f} → {last_obs[k]:8.2f}  ({d:+7.2f})"
                        )
        except Exception:
            pass
        if traj_f is not None:
            try:
                traj_f.close()
                logging.info(f"軌道ログを保存した: {cfg.trajectory_log}")
            except Exception:
                pass

        # 正常終了・Ctrl-C・例外のどの経路でもここを通す（本実装の要点）
        runner.shutdown()


if __name__ == "__main__":
    eval()
