"""GR00T NEW_EMBODIMENT modality config for OpenArm（単腕8DoF/両腕16DoF・カメラ構成は OPENARM_CAMERAS で決める）.

launch_finetune.py に --modality_config_path openarm_embodiment/openarm_config.py で渡すと
importlib で動的ロードされ、トップレベルの register_modality_config が NEW_EMBODIMENT に登録する。
二重 import はレジストリの assert で弾かれる点に注意（同一プロセスで1回だけ import）。

レイアウト（openarm_layout.py が正本）は環境変数 OPENARM_EMBODIMENT（single|bimanual）で決める:
  既定（単腕8DoF）: 何も付けない（joint_1..7 + gripper）
  両腕16DoF        : OPENARM_EMBODIMENT=bimanual
                     （right_joint_1..7+right_gripper 0:7/7:8、left_joint_1..7+left_gripper 8:15/15:16。
                      並びの正本は lerobot 側 BiOpenArmFollower._motors_ft＝right 先・left 後）
学習に使うデータセットの meta/modality.json（gen_modality_json.py で生成。OPENARM_EMBODIMENT を渡すと
判定結果との一致を検証する）と一致させる（不一致は check_dataloader.py が検出する）。

カメラ構成（openarm_cameras.py が正本）:
  既定                   : chest,wrist_right
  chest 単眼（50ep 再現） : OPENARM_CAMERAS=chest
  増やす                 : OPENARM_CAMERAS=chest,wrist_right,overhead
学習に使うデータセットの meta/modality.json（gen_modality_json.py で生成）と一致させる。

action rep: 腕（single_arm / right_arm / left_arm）=RELATIVE（N1.7 finetune_new_embodiment.md 推奨・
汎化有利）/ グリッパ（gripper / right_gripper / left_gripper）=ABSOLUTE（SO100 既定）。
収集データは絶対関節角のままで可（delta 化は内部処理）。state_key は指定しない（action_key と
同名の state を RelativeActionLoader が要求するため。gr00t/data/stats.py）。
正本: 02VLA/gr00t-openarm-pipeline-design.md §3.3。
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from openarm_cameras import cameras_from_env  # noqa: E402
from openarm_layout import action_rep, layout_from_env, modality_keys, state_names  # noqa: E402

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
LAYOUT = layout_from_env()
KEYS = modality_keys(LAYOUT)
STATE_NAMES = state_names(LAYOUT)

openarm_config = {
    "video": ModalityConfig(
        delta_indices=[0],
        modality_keys=list(CAMERAS),
    ),
    "state": ModalityConfig(
        delta_indices=[0],
        modality_keys=list(KEYS),
    ),
    "action": ModalityConfig(
        delta_indices=list(range(0, 16)),  # SO100 踏襲の action horizon（暫定）
        modality_keys=list(KEYS),
        action_configs=[
            ActionConfig(
                rep=ActionRepresentation[action_rep(k)],  # 腕=RELATIVE / グリッパ=ABSOLUTE
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            )
            for k in KEYS
        ],
    ),
    "language": ModalityConfig(
        delta_indices=[0],
        modality_keys=["annotation.human.task_description"],
    ),
}

register_modality_config(openarm_config, embodiment_tag=EmbodimentTag.NEW_EMBODIMENT)
print(f"[openarm_config] NEW_EMBODIMENT registered: layout={LAYOUT} keys={KEYS} cameras={CAMERAS}", file=sys.stderr)
