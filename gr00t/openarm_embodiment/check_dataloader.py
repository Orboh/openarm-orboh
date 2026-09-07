"""G1 §3.5 dataloader 疎通確認（モデル抜き・ShardedSingleStepDataset 直叩き）。

usage: uv run --no-sync python openarm_embodiment/check_dataloader.py <dataset_path>
       OPENARM_CAMERAS=chest uv run --no-sync python ... <dataset_path>   # chest 単眼データのとき

config 側（OPENARM_CAMERAS）とデータセット側（meta/modality.json の video キー）のカメラが
食い違っていたら、dataloader を作る前に止める。
"""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import openarm_config  # noqa: F401  register_modality_config(NEW_EMBODIMENT)・カメラは OPENARM_CAMERAS
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
_ds_cams = list(json.load(open(_mod, encoding="utf-8")).get("video", {}).keys())
if _ds_cams != list(openarm_config.CAMERAS):
    sys.exit(
        f"[FAIL] カメラ構成の不一致: config={openarm_config.CAMERAS} / dataset={_ds_cams}\n"
        f"       OPENARM_CAMERAS={','.join(_ds_cams)} を付けて実行するか、gen_modality_json.py でデータセット側を作り直す"
    )
print("[0] cameras match:", _ds_cams)
mc = MODALITY_CONFIGS[tag.value]
print("[1] registered modality keys:", list(mc.keys()))

print("[2] generate_stats + generate_rel_stats (rel stats exercises modality.json via RELATIVE single_arm)")
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
