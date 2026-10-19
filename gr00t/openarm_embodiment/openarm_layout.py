"""OpenArm の状態/行動レイアウト（単腕8DoF / 両腕16DoF）を1か所で決める（gr00t 非依存・単体でテスト可）。

構成の決め方は環境変数 OPENARM_EMBODIMENT（single|bimanual）。未指定なら DEFAULT_LAYOUT="single"。

  single   : joint_1..7 + gripper（7 関節＋グリッパ1＝8次元。従来どおり）
  bimanual : right_joint_1..7 + right_gripper + left_joint_1..7 + left_gripper（16次元）

並びの正本は lerobot 側 BiOpenArmFollower._motors_ft（bi_openarm_follower.py L124-133、
right_* を先・left_* を後に並べる）と lerobot_record.py（L526-537、dataset_features を
robot.action_features / robot.observation_features から組む）。teleop（BiOpenArmLeader）側の
left 先の並びは dataset 側には反映されない（build_dataset_frame が dataset.features の names
で値を引くため。feature_utils.py L117-121）。

action rep: 腕（*_arm）は RELATIVE、グリッパ（*_gripper）は ABSOLUTE。
"""

from __future__ import annotations

import json
from pathlib import Path

ENV_VAR = "OPENARM_EMBODIMENT"
DEFAULT_LAYOUT = "single"

_ARM = [f"joint_{i}.pos" for i in range(1, 8)]

LAYOUTS = {
    "single": {
        "state_names": _ARM + ["gripper.pos"],
        "groups": [("single_arm", 0, 7), ("gripper", 7, 8)],
    },
    "bimanual": {
        "state_names": (
            [f"right_{n}" for n in _ARM]
            + ["right_gripper.pos"]
            + [f"left_{n}" for n in _ARM]
            + ["left_gripper.pos"]
        ),
        "groups": [
            ("right_arm", 0, 7),
            ("right_gripper", 7, 8),
            ("left_arm", 8, 15),
            ("left_gripper", 15, 16),
        ],
    },
}


def parse_layout(raw: str) -> str:
    if raw not in LAYOUTS:
        raise ValueError(f"{ENV_VAR} は {sorted(LAYOUTS)} のいずれか。渡されたのは: {raw!r}")
    return raw


def layout_from_env() -> str:
    import os

    return parse_layout(os.environ.get(ENV_VAR, DEFAULT_LAYOUT))


def state_names(layout: str) -> list[str]:
    return list(LAYOUTS[parse_layout(layout)]["state_names"])


def modality_keys(layout: str) -> list[str]:
    """groups の並び順そのまま（dict 挿入順として使う）。"""
    return [g[0] for g in LAYOUTS[parse_layout(layout)]["groups"]]


def group_slices(layout: str) -> dict[str, tuple[int, int]]:
    return {name: (start, end) for name, start, end in LAYOUTS[parse_layout(layout)]["groups"]}


def action_rep(key: str) -> str:
    """key（modality key）が "gripper" で終われば ABSOLUTE、それ以外（腕）は RELATIVE。"""
    return "ABSOLUTE" if key.endswith("gripper") else "RELATIVE"


def build_state_action(layout: str) -> dict:
    """groups 順の {"state": {...}, "action": {...}}（modality.json の state/action 断片と同形）。"""
    slices = group_slices(layout)
    keys = modality_keys(layout)
    section = {k: {"start": slices[k][0], "end": slices[k][1]} for k in keys}
    return {"state": dict(section), "action": dict(section)}


def detect_layout(dataset_root: str | Path) -> str:
    """meta/info.json の observation.state / action の names からレイアウトを当てる。

    両方が同じ既知レイアウトの state_names と完全一致しないと ValueError（names を含む）。
    """
    info_path = Path(dataset_root) / "meta" / "info.json"
    if not info_path.is_file():
        raise FileNotFoundError(f"meta/info.json が無い: {info_path}")
    features = json.loads(info_path.read_text(encoding="utf-8")).get("features", {})
    state_feat = features.get("observation.state", {})
    action_feat = features.get("action", {})
    state_names_ds = list(state_feat.get("names", []))
    action_names_ds = list(action_feat.get("names", []))

    matched = None
    for layout, spec in LAYOUTS.items():
        if state_names_ds == list(spec["state_names"]):
            matched = layout
            break
    if matched is None:
        raise ValueError(
            "observation.state の names がどのレイアウトとも一致しない: "
            f"state.names={state_names_ds}"
        )
    if action_names_ds != state_names_ds:
        raise ValueError(
            f"observation.state と action の names が不一致: state={state_names_ds} action={action_names_ds}"
        )
    return matched
