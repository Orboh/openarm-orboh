"""OpenArm のカメラ構成を1か所で決める（gr00t 非依存・単体でテスト可）。

カメラ名は LeRobot の `observation.images.<name>` の <name>（= modality.json の video キー）。
構成の決め方は環境変数 OPENARM_CAMERAS（カンマ区切り）。未指定なら DEFAULT_CAMERAS。

  既定（2026-09-05 以降の標準）: chest,wrist_right
  chest 単眼（50ep v2 の再現）  : OPENARM_CAMERAS=chest
  3カメラ以上                   : OPENARM_CAMERAS=chest,wrist_right,overhead  のように足すだけ

カメラ構成が違えば入力次元が違う「別モデル」になる。学習に使うデータセットの
meta/modality.json と必ず一致させる（不一致は check_dataloader.py が検出する）。
"""

from __future__ import annotations

import json
import os
from pathlib import Path

ENV_VAR = "OPENARM_CAMERAS"
DEFAULT_CAMERAS = "chest,wrist_right"
IMAGE_PREFIX = "observation.images."


def parse_cameras(raw: str) -> list[str]:
    cams = [c.strip() for c in raw.split(",") if c.strip()]
    if not cams:
        raise ValueError(f"{ENV_VAR} が空。例: {ENV_VAR}={DEFAULT_CAMERAS}")
    dup = {c for c in cams if cams.count(c) > 1}
    if dup:
        raise ValueError(f"{ENV_VAR} にカメラ名の重複: {sorted(dup)}")
    return cams


def cameras_from_env() -> list[str]:
    return parse_cameras(os.environ.get(ENV_VAR, DEFAULT_CAMERAS))


def detect_cameras(dataset_root: str | Path) -> list[str]:
    """変換済みデータセットの meta/info.json から observation.images.* を拾う（並びは info.json 順）。"""
    info = Path(dataset_root) / "meta" / "info.json"
    if not info.is_file():
        raise FileNotFoundError(f"meta/info.json が無い: {info}")
    features = json.loads(info.read_text(encoding="utf-8")).get("features", {})
    cams = [k[len(IMAGE_PREFIX):] for k in features if k.startswith(IMAGE_PREFIX)]
    if not cams:
        raise ValueError(f"{info} に {IMAGE_PREFIX}* が1つも無い")
    return cams


def build_modality(cameras: list[str]) -> dict:
    """GR00T の meta/modality.json（単腕 8DoF・関節7＋グリッパ1）を組み立てる。"""
    return {
        "state": {
            "single_arm": {"start": 0, "end": 7},
            "gripper": {"start": 7, "end": 8},
        },
        "action": {
            "single_arm": {"start": 0, "end": 7},
            "gripper": {"start": 7, "end": 8},
        },
        "video": {c: {"original_key": f"{IMAGE_PREFIX}{c}"} for c in cameras},
        "annotation": {"human.task_description": {"original_key": "task_index"}},
    }
