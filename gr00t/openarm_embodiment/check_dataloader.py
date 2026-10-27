"""G1 §3.5 dataloader 疎通確認（モデル抜き・ShardedSingleStepDataset 直叩き）。

usage: uv run --no-sync python openarm_embodiment/check_dataloader.py <dataset_path>
       OPENARM_CAMERAS=chest uv run --no-sync python ... <dataset_path>   # chest 単眼データのとき
       OPENARM_EMBODIMENT=bimanual uv run --no-sync python ... <dataset_path>  # 両腕16DoFデータのとき

config 側（OPENARM_CAMERAS / OPENARM_EMBODIMENT）とデータセット側（meta/modality.json、
meta/info.json の observation.state names）が食い違っていたら、dataloader を作る前に止める。
"""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import openarm_config  # noqa: F401  register_modality_config(NEW_EMBODIMENT)・カメラは OPENARM_CAMERAS／レイアウトは OPENARM_EMBODIMENT
import openarm_layout
from gr00t.configs.data.embodiment_configs import MODALITY_CONFIGS
from gr00t.data.dataset.sharded_single_step_dataset import (
    ShardedSingleStepDataset,
    extract_step_data,
)
from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.data.stats import generate_rel_stats, generate_stats

DS = sys.argv[1]
tag = EmbodimentTag.NEW_EMBODIMENT

# [0] config とデータセットのカメラ構成が一致しているか（違えば別モデル。ここで止める）
_mod = os.path.join(DS, "meta", "modality.json")
if not os.path.isfile(_mod):
    sys.exit(f"[FAIL] {_mod} が無い。gen_modality_json.py で生成する")
_mod_data = json.load(open(_mod, encoding="utf-8"))
_ds_cams = list(_mod_data.get("video", {}).keys())
if _ds_cams != list(openarm_config.CAMERAS):
    sys.exit(
        f"[FAIL] カメラ構成の不一致: config={openarm_config.CAMERAS} / dataset={_ds_cams}\n"
        f"       OPENARM_CAMERAS={','.join(_ds_cams)} を付けて実行するか、gen_modality_json.py でデータセット側を作り直す"
    )
print("[0] cameras match:", _ds_cams)

# [0b] config（OPENARM_EMBODIMENT）とデータセット側（modality.json のキー順・start/end、
#      info.json の observation.state names）の embodiment（単腕/両腕）が一致しているか
_ds_state_keys = list(_mod_data.get("state", {}).keys())
_ds_action_keys = list(_mod_data.get("action", {}).keys())
_expected_slices = openarm_layout.group_slices(openarm_config.LAYOUT)
_ds_slices_state = {k: (v["start"], v["end"]) for k, v in _mod_data.get("state", {}).items()}
_ds_slices_action = {k: (v["start"], v["end"]) for k, v in _mod_data.get("action", {}).items()}
if (
    _ds_state_keys != openarm_config.KEYS
    or _ds_action_keys != openarm_config.KEYS
    or _ds_slices_state != _expected_slices
    or _ds_slices_action != _expected_slices
):
    try:
        _detected = openarm_layout.detect_layout(DS)
    except Exception as e:  # noqa: BLE001  自動判定に失敗してもメッセージは出す
        _detected = f"<detect_layout failed: {e}>"
    sys.exit(
        f"[FAIL] embodiment 不一致: config={openarm_config.LAYOUT}/{openarm_config.KEYS} "
        f"dataset state={_ds_state_keys} action={_ds_action_keys}\n"
        f"       OPENARM_EMBODIMENT={_detected} を付けて実行するか、gen_modality_json.py でデータセット側を作り直す"
    )
_info = os.path.join(DS, "meta", "info.json")
if os.path.isfile(_info):
    _state_names_ds = (
        json.load(open(_info, encoding="utf-8")).get("features", {}).get("observation.state", {}).get("names", [])
    )
    if list(_state_names_ds) and list(_state_names_ds) != openarm_config.STATE_NAMES:
        sys.exit(
            f"[FAIL] observation.state の names が config と不一致: config={openarm_config.STATE_NAMES} "
            f"dataset={_state_names_ds}"
        )
print("[0b] embodiment match:", openarm_config.LAYOUT, openarm_config.KEYS)

mc = MODALITY_CONFIGS[tag.value]
print("[1] registered modality keys:", list(mc.keys()))

print("[2] generate_stats + generate_rel_stats (rel stats exercises modality.json via RELATIVE な腕キー)")
generate_stats(DS)
generate_rel_stats(DS, tag)

print("[3] build ShardedSingleStepDataset")
ds = ShardedSingleStepDataset(
    dataset_path=DS,
    embodiment_tag=tag,
    modality_configs=mc,
    video_backend="ffmpeg",  # CLI（AV1対応の static-ffmpeg v8.0 = ffmpeg+ffprobe を PATH に）。torchcodec は FFmpeg 共有ライブラリ不在で不可
    shard_size=4096,
    episode_sampling_rate=1.0,
    seed=42,
    allow_padding=True,
)
print("    num shards (len(ds)) =", len(ds))

print("[4] load episode 0 (AV1 video + state/action) and extract step 0 via modality config")
episode_data = ds.episode_loader[0]  # decodes AV1 video + reads parquet
print("    episode_data type:", type(episode_data).__name__)
step = extract_step_data(episode_data, 0, ds.modality_configs, ds.embodiment_tag, ds.allow_padding)
print("    VLAStepData type:", type(step).__name__)


def _shape(v):
    import numpy as _np

    if isinstance(v, _np.ndarray):
        return f"ndarray{v.shape} {v.dtype}"
    if isinstance(v, dict):
        return {k: _shape(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return f"{type(v).__name__}(len={len(v)})" + (
            f" first={_shape(v[0])}" if v else ""
        )
    return f"{type(v).__name__}"


fields = getattr(step, "__dataclass_fields__", None)
if fields:
    for f in fields:
        print(f"      {f}: {_shape(getattr(step, f))}")
else:
    print("      repr:", step)

print("DATALOADER_PASS")
