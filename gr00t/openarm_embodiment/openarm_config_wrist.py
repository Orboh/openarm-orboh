"""GR00T NEW_EMBODIMENT modality config for OpenArm (chest+wrist・8DoF).

launch_finetune.py に --modality_config_path openarm_embodiment/openarm_config.py で渡すと
importlib で動的ロードされ、トップレベルの register_modality_config が NEW_EMBODIMENT に登録する。
二重 import はレジストリの assert で弾かれる点に注意（同一プロセスで1回だけ import）。

action rep: single_arm=RELATIVE（N1.7 finetune_new_embodiment.md 推奨・汎化有利）/
gripper=ABSOLUTE（SO100 既定）。収集データは絶対関節角のままで可（delta 化は内部処理）。
正本: 02VLA/gr00t-openarm-pipeline-design.md §3.3。

2026-09-05: wrist_right（D405手首カメラ）を追加。chest単眼データとwrist込みデータは
入力次元が異なる別モデルになる（record_pick_place_openarm.sh WRIST=true の設計判断と同じ）。
"""

from gr00t.configs.data.embodiment_configs import register_modality_config
from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.data.types import (
    ActionConfig,
    ActionFormat,
    ActionRepresentation,
    ActionType,
    ModalityConfig,
)

openarm_config = {
    "video": ModalityConfig(
        delta_indices=[0],
        modality_keys=["chest", "wrist_right"],
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
