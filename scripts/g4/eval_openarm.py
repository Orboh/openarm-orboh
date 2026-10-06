#!/usr/bin/env python3
"""
eval_openarm.py — GR00T N1.7 推論 → OpenArm 実機ブリッジ（G4）

**両腕経路は 2026-09-21 時点で実機未検証。初回は必ず安全プロトコル §3 の Ctrl-C→torque disable
検証を 1 回踏む（--dry_run=true で connect→両腕整列→Ctrl-C→両腕脱力の目視まで）。**

`gr00t/eval/real_robot/SO100/eval_so100.py` を雛形に、OpenArm（7関節＋グリッパ＝8次元）用へ
置き換えたもの。差分の要点は以下3つ:

  1. **8次元化**: SO100 は 5関節＋グリッパ＝6次元。OpenArm は joint_1..7 + gripper = 8次元。
     modality.json の分割（single_arm 0:7 / gripper 7:8）と一致させる。
     両腕（bi_openarm_follower）は 16 次元（右 8 → 左 8 の順・契約 BIMANUAL_STATE_KEYS）で、
     modality.json の分割は right_arm 0:7 / right_gripper 7:8 / left_arm 8:15 / left_gripper 15:16。
  2. **カメラ1本**: SO100 は front+wrist の2本。OpenArm 単腕は chest 単眼のみ。
     両腕は chest + wrist_right の 2 本（--robot.cameras の挿入順を video キー順にする）。
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

両腕（bi_openarm_follower。max_relative_target は **腕ごと** に 8 キーで渡す。
top-level --robot.max_relative_target は BiOpenArmFollowerConfig に存在しない）:
    python eval_openarm.py --robot.type=bi_openarm_follower \
        --robot.left_arm_config.port=can_fol_l --robot.left_arm_config.side=left \
        --robot.right_arm_config.port=can_fol_r --robot.right_arm_config.side=right \
        --robot.id=follower \
        --robot.cameras='{"chest": {"type": "intelrealsense", "serial_number_or_name": "344422070898", "width": 640, "height": 480, "fps": 30}, "wrist_right": {"type": "intelrealsense", "serial_number_or_name": "352122273417", "width": 640, "height": 480, "fps": 30}}' \
        --robot.left_arm_config.max_relative_target='{joint_1: 2.5, joint_2: 2.5, joint_3: 3.0, joint_4: 5.0, joint_5: 5.0, joint_6: 4.0, joint_7: 5.0, gripper: 15.0}' \
        --robot.right_arm_config.max_relative_target='{joint_1: 2.5, joint_2: 2.5, joint_3: 3.0, joint_4: 5.0, joint_5: 5.0, joint_6: 4.0, joint_7: 5.0, gripper: 15.0}' \
        --dry_run=true --log_obs_every_step=false
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
    # draccus の choice `bi_openarm_follower` は config_bi_openarm_follower.py の
    # register_subclass が import 時に走って初めて登録される（lerobot_record.py も同じ import をしている）。
    bi_openarm_follower,
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

    def get_action(self, observations: dict, options: dict | None = None):
        """
        サーバとの契約（`gr00t/policy/server_client.py` 実読・2026-07-22）:
          - サーバは `handler.handler(**request["data"])` と **キーワード展開して呼ぶ**ため、
            `data` には観測そのものでなく **`{"observation": obs}`** を入れる
            （`policy.get_action(self, observation, options=None)` に対応）
          - 正常時の戻り値は **`(action, info)` のタプル**
          - 例外時は **`{"error": "..."}`** が返る（成功と同じ経路で来るので明示的に検出する）
          - `options` は RTC のときだけ渡す（2026-10-07・`gr00t_rtc_server.py` だけが読む。
            公式の Gr00tPolicy は捨てる）。None のときは従来どおり `options` を送らない。
        """
        data = {"observation": observations}
        if options is not None:
            data["options"] = options
        resp = self.call_endpoint("get_action", data)

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


# =============================================================================
# state/action のレイアウト（単腕 8 次元 / 両腕 16 次元）
# =============================================================================
# 収集データ meta/info.json の features names と完全一致させること。
# ここがずれると学習時と推論時で関節の対応が入れ替わり、危険な挙動になる。
#
# gr00t/openarm_embodiment/openarm_layout.py の LAYOUTS と同一であること
# （別 venv なので import せず複製。検証 3 で突合）。
ARM_MOTOR_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7", "gripper"]

SINGLE_STATE_KEYS = [f"{m}.pos" for m in ARM_MOTOR_NAMES]

# 両腕は右 8 → 左 8 の順（BiOpenArmFollower._motors_ft が right_* を先に並べ、
# lerobot_record.py は robot.observation_features / action_features からそのまま
# dataset features を組むため。teleop 側の left 先順は dataset の並びに影響しない）。
BIMANUAL_STATE_KEYS = [f"right_{m}.pos" for m in ARM_MOTOR_NAMES] + [f"left_{m}.pos" for m in ARM_MOTOR_NAMES]

# modality.json の state/action 分割（(modality_key, start, end)）。
# PolicyServer が返す action チャンクのキー名は modality config の action.modality_keys
# そのものなので、ここが modality.json と食い違うと decode で KeyError になる。
SINGLE_GROUPS = [("single_arm", 0, 7), ("gripper", 7, 8)]
BIMANUAL_GROUPS = [
    ("right_arm", 0, 7),
    ("right_gripper", 7, 8),
    ("left_arm", 8, 15),
    ("left_gripper", 15, 16),
]


class OpenArmAdapter:
    """
    Raw robot observation ↔ GR00T VLA 入出力の変換器。

    SO100Adapter との差分:
      - robot_state_keys が single: 8 個（joint_1..7 + gripper）/ bimanual: 16 個（契約 BIMANUAL_STATE_KEYS・右→左）
      - camera_keys が single: chest 単眼 / bimanual: --robot.cameras の挿入順（既定 chest + wrist_right）
      - state 分割が single: [:7] / [7:8]（modality.json の single_arm 0:7 / gripper 7:8 と一致）
        bimanual: right_arm 0:7 / right_gripper 7:8 / left_arm 8:15 / left_gripper 15:16
    """

    # 単腕の既定（クラス属性は後方互換のため残す。インスタンスは self.robot_state_keys / self.camera_keys を使う）。
    ROBOT_STATE_KEYS = SINGLE_STATE_KEYS
    CAMERA_KEYS = ["chest"]

    def __init__(self, policy_client: PolicyClient, bimanual: bool = False,
                 camera_keys: list[str] | None = None):
        self.policy = policy_client
        self.bimanual = bimanual
        self.robot_state_keys = list(BIMANUAL_STATE_KEYS if bimanual else SINGLE_STATE_KEYS)
        self.groups = list(BIMANUAL_GROUPS if bimanual else SINGLE_GROUPS)
        self.camera_keys = list(camera_keys) if camera_keys is not None else list(self.CAMERA_KEYS)

    def obs_to_policy_inputs(self, obs: Dict[str, Any]) -> Dict:
        model_obs = {}
        model_obs["video"] = {k: obs[k] for k in self.camera_keys}

        state = np.array([obs[k] for k in self.robot_state_keys], dtype=np.float32)
        model_obs["state"] = {key: state[s:e] for key, s, e in self.groups}
        model_obs["language"] = {"annotation.human.task_description": obs["lang"]}

        model_obs = recursive_add_extra_dim(model_obs)
        model_obs = recursive_add_extra_dim(model_obs)
        return model_obs

    def decode_action_chunk(self, chunk: Dict, t: int) -> Dict[str, float]:
        """chunk[<modality_key>]: (B, T, D) を groups の順で concat → モータ指令 dict

        single: single_arm (B,T,7) + gripper (B,T,1) → 8 キー
        bimanual: right_arm (B,T,7) + right_gripper (B,T,1) + left_arm (B,T,7) + left_gripper (B,T,1) → 16 キー
        """
        full = np.concatenate([np.asarray(chunk[key][0][t]).reshape(-1) for key, _, _ in self.groups])
        assert full.shape[0] == len(self.robot_state_keys), (
            f"action 次元 {full.shape[0]} != state キー数 {len(self.robot_state_keys)}（modality.json と不一致）"
        )
        return {name: float(full[i]) for i, name in enumerate(self.robot_state_keys)}

    def get_action(self, obs: Dict) -> List[Dict[str, float]]:
        model_input = self.obs_to_policy_inputs(obs)
        action_chunk, _info = self.policy.get_action(model_input)
        any_key = next(iter(action_chunk.keys()))
        horizon = action_chunk[any_key].shape[1]  # (B, T, D) → T
        return [self.decode_action_chunk(action_chunk, t) for t in range(horizon)]

    def get_chunk(self, obs: Dict, options: dict | None = None):
        """get_action と同じ推論で、チャンクの生の配列（{modality_key: (B, T, D)}）と info も返す。

        RTC で前のチャンクの続きを送るとき、生の配列から modality キーごとに切り出す（2026-10-07）。
        """
        model_input = self.obs_to_policy_inputs(obs)
        action_chunk, info = self.policy.get_action(model_input, options)
        any_key = next(iter(action_chunk.keys()))
        horizon = action_chunk[any_key].shape[1]
        actions = [self.decode_action_chunk(action_chunk, t) for t in range(horizon)]
        raw = {key: np.asarray(action_chunk[key]) for key, _, _ in self.groups}
        return actions, raw, (info if isinstance(info, dict) else {})


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
    # カメラを開くのを「両腕の整列が終わってから」に遅らせる（2026-09-24 板橋 v2）。
    # 右腕をカメラ役にした構成では、右腕が経由地を通って整列する大きな動きで右 wrist の
    # ケーブルが引かれ、USB から一時的に抜ける（整列 8 回中 5 回・腕が止まると 10 秒前後で
    # 自分から再列挙する）。整列中にカメラを開いていると、その脱落で最初の観測が古い画像になり
    # 方策が 1 step も動かずに止まる。true にすると、整列後に RealSense のシリアルが USB 上に
    # 見えるまで camera_wait_s 秒待ってから開く。既定 false（従来どおり connect() 内で開く）。
    defer_camera_connect: bool = False
    camera_wait_s: float = 30.0

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
    # chest 以外のカメラ（手首）も保存する（2026-09-24 板橋 v2）。chest は従来どおり
    # frame_log_dir に .npy（convert_trajlog_to_lerobot.py の入力）、それ以外は同じ実行フォルダの
    # frames_<camera>/frame_<step>.jpg（JPEG q90）。書き込みは別スレッドで行い制御ループを待たせない。
    # 既定 false（従来どおり chest のみ）。
    save_wrist_frames: bool = False

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

    # policy に渡す video キー（カンマ区切り）。None のとき single は ["chest"]（従来どおり）、
    # bimanual は --robot.cameras の挿入順（既定 chest,wrist_right）。
    # modality.json の video キーと一致させること。
    camera_keys: str | None = None

    play_sounds: bool = False

    # ---- 推論の停止を隠す（2026-10-07・設計は Vault の gr00t-async-inference-rtc-design）----
    # inference_mode:
    #   "sync"  : 観測 → 推論を待つ → action_horizon step を流す、の直列（従来）。推論中は指令が止まる。
    #   "async" : 前のチャンクを流しながら裏のスレッドで次を推論する。届いたら「観測の tick から何 step
    #             過ぎたか」(k) を数え、新しいチャンクの k 番目から流す（0〜k-1 は過ぎた時刻の分なので捨てる）。
    #             次の推論は切り替えの直後に始める。action_horizon は使わない（チャンクを使い切る前に切り替わる）。
    #             前のチャンクを使い切っても次が来なければ、最後の指令を保って待つ（underrun として記録）。
    # rtc: Real-Time Chunking。推論のときに、新しいチャンクの冒頭に当たる前のチャンクの指令を
    #   options.rtc で送り、続きとして生成させる。**scripts/g4/gr00t_rtc_server.py のサーバが要る**
    #   （公式の run_gr00t_server.py は options を捨てるので効かない）。
    #   - sync で使う: 前のチャンクの残り（チャンク長 - action_horizon step）を冒頭に置く。frozen は
    #     min(rtc_frozen, 残り)。action_horizon をチャンク長より小さくしないと残りが無い（例: 16 step のモデルで
    #     action_horizon=12、残り 4 step を overlap にする）。frozen を 0 にすると ramp だけで寄せるが、2026-10-07 の
    #     Thor の検証では寄りが弱かった（前のチャンクとの差の中央値 0.54 度が ramp_rate 0.3 で 0.31 度・最大は変わらず）。
    #     既定（rtc_frozen=8）では残りの step をそのまま固定し、その続きを生成させる。
    #   - async で使う: 前のチャンクのうち、観測の tick 以降の指令を最大 rtc_overlap step 送る。
    #     frozen は rtc_frozen（推論の遅れの step 数。届いたときには過ぎている分なので更新させない）。
    # 方式 0（sync・rtc なし）は従来のループをそのまま通る（新しいスケジューラは通らない）。
    inference_mode: str = "sync"
    rtc: bool = False
    rtc_overlap: int = 8
    rtc_frozen: int = 8
    rtc_ramp_rate: float = 5.0


def _arm_only_deviation(dev_all: Dict[str, float]) -> Dict[str, float]:
    """B-1 ゲート用: gripper（単腕 1 本 / 両腕 2 本）を除いた腕関節の乖離だけを返す。

    判定は lerobot_record.py の verify_pose と同じ `"gripper" not in k`。
    """
    return {k: v for k, v in dev_all.items() if "gripper" not in k}


def _check_max_relative_target(robot_cfg, fps: int, bimanual: bool) -> dict:
    """max_relative_target の事前チェック（実機を繋ぐ前に落とす）。

    戻り値は検証・ログ用の view（single: {"max_relative_target": float|dict}、
    bimanual: prefix 付き 16 キー {"left_joint_1": ..., "right_gripper": ...}）。
    **robot には渡さない**——BiOpenArmFollowerConfig に top-level max_relative_target は無く、
    腕側 ensure_safe_goal_position（robots/utils.py L98-100）は goal と完全一致の 8 キー dict を要求し、
    BiOpenArmFollower.send_action は prefix を剥いだ 8 キーを各腕に渡す（L203-213）ため、
    16 キー prefix 付き dict を受け取る経路は存在しない。CLI は腕ごとに 8 キーで渡す。
    """
    def _check_one(max_rel, label: str, example: str) -> None:
        if max_rel is None:
            raise SystemExit(
                f"{label} が未設定。1ステップの移動量が無制限になり危険なので中止する。\n"
                f"  {example} のように必ず指定すること（安全プロトコル §0）。"
            )
        # B-2: lerobot の ensure_safe_goal_position は `isinstance(max_relative_target, float)`
        # で分岐する（robots/utils.py L96）。**int を渡すと `raise TypeError` でクラッシュする**（2026-07-22 実コードで確認・素通りではなく fail-safe 側）。実機接続後に無用な例外を踏まないよう、
        # 小数であることをここで強制する（無用な TypeError を実機接続後に踏まない）。
        if not isinstance(max_rel, (float, dict)):
            raise SystemExit(
                f"{label} が float でない（{type(max_rel).__name__}: {max_rel}）。\n"
                "  lerobot 側は isinstance(x, float) で判定するため int だとクランプが効かない。\n"
                f"  {example} のように**小数**で指定すること。"
            )

    if not bimanual:
        max_rel = getattr(robot_cfg, "max_relative_target", None)
        _check_one(max_rel, "max_relative_target", "--robot.max_relative_target=1.0")
        # 1 step あたりの上限 × fps ＝ 最大角速度。誤った目標へ向かった場合の到達速度になる。
        if isinstance(max_rel, float):
            logging.info(
                f"max_relative_target = {max_rel}（1ステップ上限）→ 最大 約{max_rel * fps:.0f}/s。"
                "初回実駆動は 1.0 程度まで落とすこと（レビュー B-2）。"
            )
        else:
            logging.info(f"max_relative_target = {max_rel}（関節ごと指定）")
        return {"max_relative_target": max_rel}

    # --- bimanual: 腕ごとに検証し、16 キー prefix 付きの view に合成する ---
    example = (
        "--robot.left_arm_config.max_relative_target=1.0 または "
        "--robot.right_arm_config.max_relative_target="
        "{joint_1: 2.5, joint_2: 2.5, joint_3: 3.0, joint_4: 5.0, joint_5: 5.0, joint_6: 4.0, joint_7: 5.0, gripper: 15.0}"
        "（左右それぞれに指定する）"
    )
    view: dict = {}
    for side, arm in (("left", robot_cfg.left_arm_config), ("right", robot_cfg.right_arm_config)):
        max_rel = getattr(arm, "max_relative_target", None)
        _check_one(max_rel, f"--robot.{side}_arm_config.max_relative_target", example)
        if isinstance(max_rel, dict):
            # ensure_safe_goal_position は goal（prefix 剥がし済み 8 キー）と key set の完全一致を要求する。
            if set(max_rel.keys()) != set(ARM_MOTOR_NAMES):
                raise SystemExit(
                    f"--robot.{side}_arm_config.max_relative_target の dict キーが 8 モータと一致しない。\n"
                    f"  期待: {ARM_MOTOR_NAMES}\n  実際: {sorted(max_rel.keys())}\n"
                    "  lerobot の ensure_safe_goal_position は goal と完全一致の key set を要求する（不一致は ValueError）。"
                )
            for k in ARM_MOTOR_NAMES:
                v = max_rel[k]
                if not isinstance(v, float):
                    raise SystemExit(
                        f"--robot.{side}_arm_config.max_relative_target[{k}] が float でない（{type(v).__name__}: {v}）。小数で指定すること。"
                    )
                view[f"{side}_{k}"] = v
            logging.info(f"{side} max_relative_target = {max_rel}（関節ごと指定）")
        else:
            for k in ARM_MOTOR_NAMES:
                view[f"{side}_{k}"] = max_rel
            logging.info(
                f"{side} max_relative_target = {max_rel}（1ステップ上限）→ 最大 約{max_rel * fps:.0f}/s。"
                "初回実駆動は 1.0 程度まで落とすこと（レビュー B-2）。"
            )
    return view


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

    def _buses(self) -> list[tuple[str, Any]]:
        """torque disable を直接叩く bus の一覧。

        単腕 OpenArmFollower は robot.bus 1 本。BiOpenArmFollower には bus 属性が無く
        left_arm.bus / right_arm.bus（実行確認済み）なので腕ごとに列挙する。
        """
        bus = getattr(self.robot, "bus", None)
        if bus is not None:
            return [("arm", bus)]
        out: list[tuple[str, Any]] = []
        for side in ("left", "right"):
            arm = getattr(self.robot, f"{side}_arm", None)
            arm_bus = getattr(arm, "bus", None) if arm is not None else None
            if arm_bus is not None:
                out.append((side, arm_bus))
        return out

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

        # (1) まず torque disable を明示的に試み、成否を自分で把握する（bus ごとに個別にログ・全 bus 成功で OK）
        buses = self._buses()
        if buses:
            results = []
            for name, bus in buses:
                try:
                    bus.disable_torque()
                    results.append(True)
                    logging.info(f"torque disable 成功（{name}）")
                except Exception as e:
                    results.append(False)
                    logging.error(
                        f"**torque disable に失敗した（{name}: {type(e).__name__}: {e}）。"
                        "error latch の可能性がある。直ちに power cycle（DC PSU OFF→10秒→ON）を行うこと。**"
                    )
            disable_ok = all(results)
        else:
            logging.warning("bus が取得できず torque disable を明示実行できなかった")

        # (2) 接続状態に応じて後片付け（connect() 途中で落ちた場合の二次例外を避ける）
        logging.info("robot.disconnect() を実行する")
        try:
            if getattr(self.robot, "is_connected", False):
                self.robot.disconnect()
            else:
                logging.warning("接続が未完了のまま終了する。腕の脱力を必ず目視確認すること。")
                for name, bus in buses:
                    if getattr(bus, "is_connected", False):
                        logging.info(f"未接続のまま bus.disconnect(disable_torque=True)（{name}）")
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


def _write_jpeg(img, path: str) -> None:
    """RGB 画像を JPEG で書く（手首カメラの保存用・別スレッドから呼ぶ）。"""
    import cv2

    cv2.imwrite(path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])


def _stash_cameras(robot) -> list:
    """各腕のカメラを一時的に外し、connect() でカメラを開かせない（defer_camera_connect 用）。

    OpenArmFollower.connect() は self.cameras を順に開き、get_observation() も同じ dict を読む。
    両腕版は左右の腕へ委譲するだけなので、腕ごとの dict を空にして戻せば済む。
    """
    arms = [robot.left_arm, robot.right_arm] if hasattr(robot, "left_arm") else [robot]
    stash = []
    for arm in arms:
        stash.append((arm, dict(arm.cameras)))
        arm.cameras = {}
    return stash


def _realsense_serials() -> set:
    """USB 上に見えている RealSense のシリアル一覧。取得に失敗したら例外をそのまま上げる。"""
    import pyrealsense2 as rs

    return {d.get_info(rs.camera_info.serial_number) for d in rs.context().query_devices()}


def _connect_stashed_cameras(stash: list, wait_s: float) -> None:
    """整列が終わったあとにカメラを開いて腕へ戻す。

    1. **まだどのカメラも開いていない時点で**、必要な RealSense のシリアルが全部 USB 上に
       見えるまで最大 wait_s 秒待つ。開いて映像を流し始めたあとに一覧を取り直すと、取得が
       失敗して見えているカメラまで「無い」扱いになる（2026-09-24 r4 で実際に起きた。
       1 台ずつ開きながら探す初版の誤り）
    2. 揃ったら全カメラを続けて開く。開けたカメラだけを腕へ戻すので、途中で失敗しても
       disconnect() が開いていないカメラを閉じに行かない
    """
    needed = {}
    for _arm, cams in stash:
        for key, cam in cams.items():
            serial = getattr(getattr(cam, "config", None), "serial_number_or_name", None)
            if type(cam).__name__ == "RealSenseCamera" and serial:
                needed[str(serial)] = key
    if needed:
        t0 = time.time()
        first_missing = None
        last_err = None
        while True:
            try:
                present = _realsense_serials()
                last_err = None
            except Exception as e:  # 取得の失敗は握りつぶさずに記録して待ち続ける
                present = set()
                if last_err is None:
                    logging.warning(f"RealSense の一覧取得に失敗した（待ち続ける）: {e}")
                last_err = e
            missing = [s for s in needed if s not in present]
            if first_missing is None:
                first_missing = list(missing)
            if not missing:
                break
            if time.time() - t0 > wait_s:
                names = ", ".join(f"{needed[s]}（{s}）" for s in missing)
                cause = f"。最後の一覧取得エラー: {last_err}" if last_err else ""
                raise ConnectionError(
                    f"カメラ {names} が {wait_s:.0f} 秒待っても USB 上に現れない。ケーブルの抜けを確認すること{cause}"
                )
            time.sleep(0.5)
        if first_missing:
            names = ", ".join(f"{needed[s]}（{s}）" for s in first_missing)
            logging.warning(
                f"カメラ {names} は整列の直後には USB 上に無く、{time.time() - t0:.1f} 秒後に揃った"
                "（整列の動きでケーブルが引かれて一時的に抜けていた可能性）"
            )
    for arm, cams in stash:
        for key, cam in cams.items():
            cam.connect()
            arm.cameras[key] = cam
            logging.info(f"カメラ {key} を整列後に開いた")


def _first_action_gate(first: Dict[str, float], obs: Dict[str, Any], cfg: EvalConfig) -> None:
    """B-1 の初回指令の乖離ゲート（従来ループの同じ判定を関数にしたもの・判定の中身は同じ）。"""
    dev_all = {k: abs(first[k] - float(obs[k])) for k in first if k in obs}
    dev = _arm_only_deviation(dev_all)
    worst = max(dev.values()) if dev else 0.0
    gripper_ref = {k: round(v, 2) for k, v in dev_all.items() if "gripper" in k}
    logging.info(
        f"初回指令の現在角からの乖離: 最大 {worst:.2f}（閾値 {cfg.first_action_max_dev}・腕関節のみ）"
        f" / gripper 参考値 {gripper_ref}"
    )
    if worst > cfg.first_action_max_dev:
        raise SystemExit(
            f"初回指令が現在角から {worst:.1f} 乖離している（閾値 {cfg.first_action_max_dev}）。\n"
            "  RELATIVE/ABSOLUTE の解釈違い、または座標フレームのずれが疑われる。\n"
            f"  内訳: { {k: round(v, 2) for k, v in dev.items()} }\n"
            "  中止する（実機へは何も送っていない）。"
        )


class _Chunk:
    """推論 1 回分のチャンク。origin は観測を取った tick（チャンクの 0 番目がこの tick に当たる）。"""

    __slots__ = ("cid", "origin", "actions", "raw", "infer_ms")

    def __init__(self, cid: int, origin: int, actions: list, raw: dict, infer_ms: float):
        self.cid, self.origin, self.actions, self.raw, self.infer_ms = cid, origin, actions, raw, infer_ms


def _pct(values: list, q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), q)) if values else float("nan")


def _run_scheduled(cfg: EvalConfig, robot, runner, policy: "OpenArmAdapter", state_keys: list,
                   traj_w, save_frame, t_start: float, track: dict) -> None:
    """sync+rtc / async / async+rtc の 3 方式を 30Hz の tick で回す（2026-10-07）。

    - tick は制御周期の通し番号。指令を送れなかった周期（underrun）も 1 つ進める（時刻がずれないように）。
      sync の推論待ちの間は進めない（従来どおり腕が止まっている時間で、待ちの後のチャンクの 0 番目が今に当たる）。
    - いま流すのは cur.actions[tick - cur.origin]。
    - CAN のバスに触るのは常にこのスレッド（観測の取得も send_action も）。裏のスレッドは推論の呼び出しだけを持つ。
      ZMQ の REQ ソケットも裏のスレッドだけが使う（最初の 1 回の推論も pool を通す）。
    """
    from concurrent.futures import ThreadPoolExecutor

    is_async = cfg.inference_mode == "async"
    period = 1.0 / cfg.fps
    arm_keys = [k for k in state_keys if "gripper" not in k]
    stats = {"underrun": 0, "late_drop": 0, "switch_idx": [], "jump": [], "infer_ms": [], "rtc_calls": 0}
    pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="gr00t-infer")

    def fresh_obs() -> Dict[str, Any]:
        o = robot.get_observation()
        o["lang"] = cfg.lang_instruction
        return o

    def infer(obs: Dict[str, Any], options: dict | None):
        t0 = time.time()
        actions, raw, _info = policy.get_chunk(obs, options)
        return actions, raw, (time.time() - t0) * 1000.0

    def rtc_options(cur: _Chunk, idx: int, frozen: int) -> dict | None:
        """cur の idx 番目以降（= 新しいチャンクの 0 番目以降に当たる指令）を options.rtc にする。"""
        if not cfg.rtc:
            return None
        overlap = min(cfg.rtc_overlap, len(cur.actions) - idx)
        if overlap < 1:
            return None
        prev = {k: np.ascontiguousarray(v[:, idx : idx + overlap, :], dtype=np.float32) for k, v in cur.raw.items()}
        stats["rtc_calls"] += 1
        return {"rtc": {"prev_action": prev, "frozen": int(max(0, min(frozen, overlap))),
                        "ramp_rate": float(cfg.rtc_ramp_rate)}}

    def switch_jump(new_actions: list, k: int, cur: _Chunk, old_idx: int) -> None:
        if 0 <= old_idx < len(cur.actions):
            stats["jump"].append(max(abs(new_actions[k][key] - cur.actions[old_idx][key]) for key in arm_keys))

    try:
        obs = fresh_obs()
        track["first_obs"] = {k: float(obs[k]) for k in state_keys if k in obs}
        track["last_obs"] = dict(track["first_obs"])
        actions, raw, ms = pool.submit(infer, obs, None).result()
        stats["infer_ms"].append(ms)
        _first_action_gate(actions[0], obs, cfg)
        horizon = len(actions)
        if cfg.rtc and not is_async and cfg.action_horizon >= horizon:
            raise SystemExit(
                f"sync で rtc を使うには action_horizon（{cfg.action_horizon}）をチャンク長（{horizon}）より小さくする。"
                "前のチャンクの残りが RTC の冒頭になる。中止する（実機へは何も送っていない）。"
            )
        logging.info(
            f"方式 = {cfg.inference_mode}{'+rtc' if cfg.rtc else ''}・チャンク長 {horizon} step・"
            f"{'action_horizon 不使用' if is_async else f'action_horizon {cfg.action_horizon}'}"
            + (f"・rtc overlap≤{cfg.rtc_overlap} frozen≤{cfg.rtc_frozen} "
               f"ramp_rate={cfg.rtc_ramp_rate}" if cfg.rtc else "")
        )
        cur = _Chunk(0, 0, actions, raw, ms)
        next_cid = 1
        pending = None  # (future, origin_tick, cid)
        tick = 0
        obs_fresh = True

        while tick < cfg.max_steps and not runner.should_stop:
            tic = time.time()
            switched = 0

            # (1) async: 届いた推論を取り込む。観測の tick から k step 過ぎているので k 番目から流す
            if pending is not None and pending[0].done():
                fut, origin, cid = pending
                pending = None
                new_actions, new_raw, ms = fut.result()
                stats["infer_ms"].append(ms)
                k = tick - origin
                if k < len(new_actions):
                    switch_jump(new_actions, k, cur, tick - cur.origin)
                    cur = _Chunk(cid, origin, new_actions, new_raw, ms)
                    switched = 1
                    stats["switch_idx"].append(k)
                    logging.info(f"tick={tick} chunk#{cid} 推論 {ms:.0f}ms・{k} step 過ぎた位置から切り替え")
                else:
                    stats["late_drop"] += 1
                    logging.warning(f"tick={tick} chunk#{cid} は推論 {ms:.0f}ms で遅すぎた（{k} step 過ぎ）→ 捨てる")

            idx = tick - cur.origin

            # (2) sync: action_horizon step 流したら止めて推論を待つ（従来と同じ止まり方。rtc なら残りを冒頭に置く）
            if not is_async and idx >= cfg.action_horizon:
                obs = fresh_obs()
                obs_fresh = True
                new_actions, new_raw, ms = pool.submit(infer, obs, rtc_options(cur, idx, cfg.rtc_frozen)).result()
                stats["infer_ms"].append(ms)
                switch_jump(new_actions, 0, cur, idx)
                cur = _Chunk(next_cid, tick, new_actions, new_raw, ms)
                next_cid += 1
                switched = 1
                idx = 0
                logging.info(f"tick={tick} chunk#{cur.cid} 推論 {ms:.0f}ms（sync・腕はこの間止まる）")
                tic = time.time()  # 推論待ちは周期に含めない（従来と同じ）

            # (3) 観測: async で次の推論を始めるとき、または毎 step の記録のとき
            need_submit = is_async and pending is None and tick > cur.origin
            if not obs_fresh and (need_submit or cfg.log_obs_every_step):
                try:
                    obs = fresh_obs()
                    obs_fresh = True
                except Exception:
                    if need_submit:
                        raise
                    logging.warning("記録用の観測取得に失敗（実行は継続する）", exc_info=False)
            if need_submit:
                pending = (pool.submit(infer, obs, rtc_options(cur, idx, cfg.rtc_frozen)), tick, next_cid)
                next_cid += 1
            if obs_fresh:
                track["last_obs"] = {k: float(obs[k]) for k in state_keys if k in obs}
                if cfg.frame_log_dir and cur.cid % cfg.frame_log_every == 0:
                    save_frame(obs, tick)

            # (4) 指令。チャンクを使い切っていたら送らずに最後の指令を保つ（underrun）
            underrun = 0
            if idx < len(cur.actions):
                action_dict = cur.actions[idx]
                if cfg.dry_run:
                    logging.debug(f"[dry-run] tick={tick} chunk#{cur.cid}[{idx}] {action_dict}")
                else:
                    robot.send_action(action_dict)
            else:
                action_dict = cur.actions[-1]
                underrun = 1
                stats["underrun"] += 1

            if traj_w is not None:
                traj_w.writerow(
                    [tick, idx, round(time.time() - t_start, 3)]
                    + [round(float(obs.get(k, float("nan"))), 4) for k in state_keys]
                    + [round(float(action_dict[k]), 4) for k in state_keys]
                    + [cur.cid, cur.origin, switched, round(cur.infer_ms, 1) if switched else "", underrun]
                )

            tick += 1
            obs_fresh = False
            if tick >= cfg.max_steps:
                logging.info(f"max_steps={cfg.max_steps} に到達 → 正常終了する")
                break
            dt = time.time() - tic
            if dt < period:
                time.sleep(period - dt)
    finally:
        pool.shutdown(wait=False, cancel_futures=True)
        n = max(1, tick if "tick" in locals() else 1)
        logging.info(
            f"=== 方式 {cfg.inference_mode}{'+rtc' if cfg.rtc else ''} の要約 === "
            f"推論 {len(stats['infer_ms'])} 回（中央値 {_pct(stats['infer_ms'], 50):.0f}ms・p90 {_pct(stats['infer_ms'], 90):.0f}ms）・"
            f"rtc 付き {stats['rtc_calls']} 回・underrun {stats['underrun']} tick（{100.0 * stats['underrun'] / n:.1f}%）・"
            f"遅すぎて捨てた {stats['late_drop']} 回"
        )
        if stats["switch_idx"]:
            logging.info(
                f"  切り替えた位置 k: 中央値 {_pct(stats['switch_idx'], 50):.0f}・最大 {max(stats['switch_idx'])}"
            )
        if stats["jump"]:
            logging.info(
                f"  切り替えの瞬間の指令の飛び（腕の関節の最大・度）: 中央値 {_pct(stats['jump'], 50):.2f}・"
                f"p99 {_pct(stats['jump'], 99):.2f}・最大 {max(stats['jump']):.2f}"
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

    # --- 単腕 / 両腕のレイアウト決定（安全チェックの前に確定させる）---
    # RobotConfig.type は draccus ChoiceRegistry のプロパティ（make_robot_from_config が使う）。文字列比較で分岐する。
    bimanual = cfg.robot.type == "bi_openarm_follower"
    state_keys = BIMANUAL_STATE_KEYS if bimanual else SINGLE_STATE_KEYS
    if cfg.camera_keys:
        camera_keys = [k.strip() for k in cfg.camera_keys.split(",") if k.strip()]
    elif bimanual:
        # --robot.cameras の挿入順（draccus parse で ['chest', 'wrist_right'] になることを確認済み）
        camera_keys = list(cfg.robot.cameras.keys())
        if not camera_keys:
            raise SystemExit("bimanual は --robot.cameras に chest と wrist_right を渡す（video キーが空）。")
    else:
        camera_keys = list(OpenArmAdapter.CAMERA_KEYS)
    logging.info(
        f"layout = {'bimanual' if bimanual else 'single'}（state {len(state_keys)} 次元）/ camera_keys = {camera_keys}"
    )
    if bimanual:
        logging.warning(
            "=== 両腕経路は 2026-09-21 時点で実機未検証。初回は --dry_run=true で "
            "connect→両腕整列→Ctrl-C→両腕脱力の目視（安全プロトコル §3）を必ず 1 回踏むこと ==="
        )

    # --- 安全設定の事前チェック（実機を繋ぐ前に落とす） ---
    max_rel_view = _check_max_relative_target(cfg.robot, cfg.fps, bimanual)
    if bimanual:
        logging.info(f"max_relative_target(16-key view) = {max_rel_view}")

    # B-3: 言語指示の未指定ゲート（2026-09-08）。既定値に頼ると学習時と違う指示文で推論する
    # 事故になる（テープ実機 13 本がボトル用の文のまま走った実例）。実機を繋ぐ前に落とす。
    if not cfg.lang_instruction or not cfg.lang_instruction.strip():
        raise SystemExit(
            "lang_instruction が未指定。学習時（収集時の single_task）と同じ指示文を必ず渡すこと。\n"
            '  例: --lang_instruction="Pick up the vinyl tape and place it in the bin"'
        )
    logging.info(f'lang_instruction = "{cfg.lang_instruction}"（収集時の single_task と一致していること）')

    # 推論の方式（2026-10-07）。実機を繋ぐ前に落とす。
    if cfg.inference_mode not in ("sync", "async"):
        raise SystemExit(f"inference_mode は sync か async（指定: {cfg.inference_mode!r}）")
    if cfg.rtc:
        if cfg.rtc_overlap < 1 or cfg.rtc_frozen < 0 or cfg.rtc_ramp_rate <= 0:
            raise SystemExit(
                f"rtc の値が不正: overlap={cfg.rtc_overlap}（1 以上）frozen={cfg.rtc_frozen}（0 以上）"
                f" ramp_rate={cfg.rtc_ramp_rate}（正）"
            )
        logging.warning(
            "=== rtc=true: PolicyServer は scripts/g4/gr00t_rtc_server.py で起動していること"
            "（公式の run_gr00t_server.py は options を捨て、RTC が黙って効かない） ==="
        )
    scheduled = cfg.inference_mode == "async" or cfg.rtc
    logging.info(f"推論の方式 = {cfg.inference_mode}{'+rtc' if cfg.rtc else ''}"
                 f"（{'新しいスケジューラ' if scheduled else '従来の直列ループ'}）")

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
    # 両腕は BiOpenArmFollower.connect が left_arm.connect → right_arm.connect の順で各約 2.2 秒動く。
    if bimanual:
        _align_msg = "  接続すると通電し、約2.2秒×2腕（左→右の順に自動整列）で両腕が動く。\n"
    else:
        _align_msg = "  接続すると通電し、約2.2秒の自動整列動作で腕が動く。\n"
    if not cfg.skip_pose_confirm:
        try:
            input(
                "\n"
                "  [接続前チェック]\n"
                "   - 腕が開始姿勢から遠すぎないか（接続時に開始姿勢へ自動整列する）\n"
                "   - 可動域に人・物は無いか\n"
                "   - DC PSU のスイッチに手が届くか（唯一確実な停止手段）\n"
                + _align_msg +
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
        keys = state_keys
        traj_w.writerow(
            ["step", "chunk_i", "t_sec"]
            + [f"present.{k}" for k in keys]
            + [f"cmd.{k}" for k in keys]
            # 新しいスケジューラのときだけ末尾に足す（step = tick・chunk_i = チャンク内の番号）
            + (["chunk_id", "chunk_origin", "switched", "infer_ms", "underrun"] if scheduled else [])
        )
        logging.info(f"軌道ログを記録する: {cfg.trajectory_log}")

    wrist_pool = None
    wrist_dirs = {}
    if cfg.save_wrist_frames and cfg.frame_log_dir:
        from concurrent.futures import ThreadPoolExecutor

        wrist_pool = ThreadPoolExecutor(max_workers=1)
        base = os.path.dirname(os.path.abspath(cfg.frame_log_dir))
        for k in camera_keys[1:]:
            wrist_dirs[k] = os.path.join(base, f"frames_{k}")
            os.makedirs(wrist_dirs[k], exist_ok=True)
        logging.info(f"手首カメラの画像も保存する: {list(wrist_dirs.values())}")

    try:
        cam_stash = _stash_cameras(robot) if cfg.defer_camera_connect else None
        robot.connect()
        if cam_stash is not None:
            logging.info("両腕の整列が終わった。カメラを開く（defer_camera_connect=true）")
            _connect_stashed_cameras(cam_stash, cfg.camera_wait_s)
        log_say("Robot connected", cfg.play_sounds, blocking=True)

        policy = OpenArmAdapter(policy_client, bimanual=bimanual, camera_keys=camera_keys)
        log_say(f'Policy ready: "{cfg.lang_instruction}"', cfg.play_sounds, blocking=True)

        step = 0
        checked_first_action = False
        t_start = time.time()
        first_obs = None
        last_obs = None
        chunk_idx = 0
        track: dict = {}  # 新しいスケジューラが開始・終了の関節角を入れる（finally の要約に使う）

        def save_frame(source_obs, at_step: int) -> None:
            """運転時フレームを保存する（学習分布との突き合わせを後から可能にする）。

            保存名の番号は軌道ログ CSV の `step` 列と一致させる。
            変換スクリプト（convert_trajlog_to_lerobot.py）はこの対応で画像と行を結ぶ。
            """
            try:
                # camera_keys[0]（chest）だけ保存する現行仕様を維持（wrist は保存しない）
                img = source_obs.get(camera_keys[0])
                if img is not None:
                    os.makedirs(cfg.frame_log_dir, exist_ok=True)
                    np.save(os.path.join(cfg.frame_log_dir, f"frame_{at_step:05d}.npy"), img)
                if wrist_pool is not None:
                    for k, d in wrist_dirs.items():
                        wimg = source_obs.get(k)
                        if wimg is not None:
                            wrist_pool.submit(
                                _write_jpeg, np.array(wimg, copy=True), os.path.join(d, f"frame_{at_step:05d}.jpg")
                            )
            except Exception:
                logging.warning("フレーム保存に失敗（実行は継続する）", exc_info=False)

        if scheduled:
            _run_scheduled(cfg, robot, runner, policy, state_keys, traj_w, save_frame, t_start, track)

        while not scheduled and step < cfg.max_steps and not runner.should_stop:
            obs = robot.get_observation()
            obs["lang"] = cfg.lang_instruction
            if first_obs is None:
                first_obs = {k: float(obs[k]) for k in state_keys if k in obs}
            last_obs = {k: float(obs[k]) for k in state_keys if k in obs}

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
                # 判定は lerobot_record.py L614 と同じ `"gripper" not in k`（両腕では
                # right_gripper.pos / left_gripper.pos の gripper 2 本を除外する。
                # startswith("gripper") だと prefix 付きキーを除外できない）。
                dev_all = {k: abs(first[k] - float(obs[k])) for k in first if k in obs}
                dev = _arm_only_deviation(dev_all)
                worst = max(dev.values()) if dev else 0.0
                gripper_ref = {k: round(v, 2) for k, v in dev_all.items() if "gripper" in k}
                logging.info(
                    f"初回指令の現在角からの乖離: 最大 {worst:.2f}（閾値 {cfg.first_action_max_dev}・腕関節のみ）"
                    f" / gripper 参考値 {gripper_ref}"
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
                    keys = state_keys
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
            if scheduled:
                first_obs, last_obs = track.get("first_obs"), track.get("last_obs")
            if first_obs and last_obs:
                logging.info("=== 関節角の変化（開始 → 終了） ===")
                for k in state_keys:
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
        # 腕の脱力を先に済ませてから、手首画像の書き残しを待つ
        if wrist_pool is not None:
            try:
                wrist_pool.shutdown(wait=True)
                for k, d in wrist_dirs.items():
                    logging.info(f"手首カメラ {k} の画像 {len(os.listdir(d))} 枚: {d}")
            except Exception:
                pass


if __name__ == "__main__":
    eval()
