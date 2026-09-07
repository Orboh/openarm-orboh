"""GR00T NEW_EMBODIMENT modality config for OpenArm（8DoF・カメラ構成は OPENARM_CAMERAS で決める）.

launch_finetune.py に --modality_config_path openarm_embodiment/openarm_config.py で渡すと
importlib で動的ロードされ、トップレベルの register_modality_config が NEW_EMBODIMENT に登録する。
二重 import はレジストリの assert で弾かれる点に注意（同一プロセスで1回だけ import）。

カメラ構成（openarm_cameras.py が正本）:
  既定                   : chest,wrist_right
  chest 単眼（50ep 再現） : OPENARM_CAMERAS=chest
  増やす                 : OPENARM_CAMERAS=chest,wrist_right,overhead
学習に使うデータセットの meta/modality.json（gen_modality_json.py で生成）と一致させる。

action rep: single_arm=RELATIVE（N1.7 finetune_new_embodiment.md 推奨・汎化有利）/
gripper=ABSOLUTE（SO100 既定）。収集データは絶対関節角のままで可（delta 化は内部処理）。
正本: 02VLA/gr00t-openarm-pipeline-design.md §3.3。
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from openarm_cameras import cameras_from_env  # noqa: E402

from gr00t.configs.data.embodiment_configs import register_modality_config  # noqa: E402
from gr00t.data.embodiment_tags import EmbodimentTag  # noqa: E402
from gr00t.data.types import (  # noqa: E402
    ActionConfig,
    ActionFormat,
    ActionRepresentation,
    ActionType,
    ModalityConfig,
)

CAMERAS = cameras_from_env()

openarm_config = {
    "video": ModalityConfig(
        delta_indices=[0],
        modality_keys=list(CAMERAS),
    ),
    "state": ModalityConfig(
        delta_indices=[0],
        modality_keys=[
            "single_arm",
            "gripper",
        ],
    ),
    "action": ModalityConfig(
        delta_indices=list(range(0, 16)),  # SO100 踏襲の action horizon（暫定）
        modality_keys=[
            "single_arm",
            "gripper",
        ],
        action_configs=[
            ActionConfig(
                rep=ActionRepresentation.RELATIVE,  # single_arm: N1.7 推奨
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
            ActionConfig(
                rep=ActionRepresentation.ABSOLUTE,  # gripper: SO100 既定
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
        ],
    ),
    "language": ModalityConfig(
        delta_indices=[0],
        modality_keys=["annotation.human.task_description"],
    ),
}

register_modality_config(openarm_config, embodiment_tag=EmbodimentTag.NEW_EMBODIMENT)
print(f"[openarm_config] NEW_EMBODIMENT registered: cameras={CAMERAS}", file=sys.stderr)
