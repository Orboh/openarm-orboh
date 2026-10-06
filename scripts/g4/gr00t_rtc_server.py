#!/usr/bin/env python3
"""
gr00t_rtc_server.py — Real-Time Chunking（RTC）に対応した GR00T N1.7 の PolicyServer（2026-10-07）

GR00T N1.7 の action head には RTC の部品がある（`gr00t/model/gr00t_n1d7/gr00t_n1d7.py`
`get_action_with_features`: 入力に `action` があれば、前のチャンクの末尾 `rtc_overlap_steps` を
新しいチャンクの冒頭に入れ、そのうち `rtc_frozen_steps` は更新せず、残りは指数の ramp で混ぜる）。
ただし公式リリース（2026-04-17）と上流 main（2026-08-20）のどちらでも、`Gr00tPolicy._get_action` が
`options` を捨てていて、サーバ経由では使えない（上流の getting_started/real_world_deployment.md
「RTC status (experimental)」）。

このスクリプトは Isaac-GR00T の作業ツリーを書き換えずに、`Gr00tPolicy` を継承して `_get_action` だけを
上書きする。`options["rtc"]` が無い呼び出しは元の `_get_action` をそのまま通る（従来のサーバと同じ挙動）。

options["rtc"] の契約（クライアント = scripts/g4/eval_openarm.py）:
    {
      "prev_action": {<action modality key>: ndarray (B, overlap, D)},  # 物理値の絶対角
      "frozen": int,        # 0 <= frozen <= overlap
      "ramp_rate": float,   # > 0
    }
  - prev_action の j 番目は、新しいチャンクの j 番目（= 今回の観測の時刻から j step 後）に当たる
    前のチャンクの指令。チャンク上の位置合わせはクライアントが済ませて送る。
  - overlap は prev_action の長さ。チャンク長（modality config の action の delta_indices の数）以下。

変換: 腕は観測の時点の状態からの相対値（ActionRepresentation.RELATIVE）で学習しているので、前のチャンクは
**今回の観測の状態を基準にした相対値** に直してから正規化する（processor の state_action_processor.apply_action。
decode_action が使う unapply_action の逆）。学習時の collate と同じく、modality キーの順に結合し、0 で
max_action_dim・max_action_horizon まで詰める。GR00T の RTC は prev[:, H-overlap:H] を新しいチャンクの
[0:overlap] に入れる（H = options["action_horizon"]）ので、prev_action をその位置に置く。

起動（Thor・tmux policy の中。rollout_bi_takeei.sh server-rtc が呼ぶ）:
    cd ~/projects/Isaac-GR00T-n1.7
    .venv/bin/python ~/projects/openarm-orboh/scripts/g4/gr00t_rtc_server.py \\
        --model-path ~/models/gr00t-bi-takeei-sort-v1 --embodiment-tag NEW_EMBODIMENT \\
        --host 127.0.0.1 --port 5555
"""

import os
import sys
import time
from dataclasses import dataclass
from typing import Any

# gr00t が venv に入っていない場合に備え、Isaac-GR00T の作業ツリーを import 経路に足す
# （既定は ~/projects/Isaac-GR00T-n1.7。GR00T_ROOT で変えられる）。
_GR00T_ROOT = os.environ.get("GR00T_ROOT", os.path.expanduser("~/projects/Isaac-GR00T-n1.7"))
if os.path.isdir(_GR00T_ROOT) and _GR00T_ROOT not in sys.path:
    sys.path.insert(0, _GR00T_ROOT)

import numpy as np  # noqa: E402
import torch  # noqa: E402
import tyro  # noqa: E402

from gr00t.data.embodiment_tags import EmbodimentTag  # noqa: E402
from gr00t.data.types import MessageType  # noqa: E402
from gr00t.policy.gr00t_policy import Gr00tPolicy, _rec_to_dtype  # noqa: E402
from gr00t.policy.server_client import PolicyServer  # noqa: E402


class Gr00tRtcPolicy(Gr00tPolicy):
    """options["rtc"] があるときだけ、前のチャンクを入力の `action` に入れて RTC で推論する。"""

    def _rtc_action_tensor(self, prev_action: dict, states: list, overlap: int, horizon: int) -> torch.Tensor:
        """前のチャンク（物理値の絶対角）を、モデルの入力の `action`（B, max_horizon, max_action_dim）に直す。"""
        tag = self.embodiment_tag.value
        sap = self.processor.state_action_processor
        keys = self.modality_configs["action"].modality_keys
        max_dim = self.processor.max_action_dim
        max_h = self.processor.max_action_horizon
        out = []
        for b, raw_state in enumerate(states):
            # 相対値の正規化の統計値はチャンクの位置（0〜H-1 番目）ごとにあり、apply_action は長さ H の行動を
            # 前提にする（短いと boolean index の形が合わずに落ちる・2026-10-07 に Thor で確認）。前のチャンクの続きは
            # 新しいチャンクの 0〜overlap-1 番目に入るので、長さ H の先頭に置き（残りは最後の行の繰り返し）、
            # その位置の統計値で正規化してから先頭 overlap 行を取り出す。
            seg = {}
            for k in keys:
                v = np.asarray(prev_action[k][b], dtype=np.float32)  # (overlap, D)
                seg[k] = np.concatenate([v, np.repeat(v[-1:], horizon - overlap, axis=0)], axis=0)  # (H, D)
            norm = sap.apply_action(seg, tag, state=raw_state)  # 今回の状態を基準にした相対値 → 正規化
            cat = np.concatenate([np.asarray(norm[k], dtype=np.float32)[:overlap] for k in keys], axis=-1)
            buf = np.zeros((max_h, max_dim), dtype=np.float32)
            buf[horizon - overlap : horizon, : cat.shape[1]] = cat
            out.append(buf)
        return torch.from_numpy(np.stack(out, axis=0))

    def _get_action(self, observation: dict[str, Any], options: dict[str, Any] | None = None):
        rtc = (options or {}).get("rtc")
        if not rtc:
            return super()._get_action(observation, options)

        t0 = time.time()
        keys = self.modality_configs["action"].modality_keys
        horizon = len(self.modality_configs["action"].delta_indices)
        prev_action = rtc["prev_action"]
        missing = [k for k in keys if k not in prev_action]
        if missing:
            raise ValueError(f"rtc.prev_action に action のキーが無い: {missing}（必要: {keys}）")
        overlap = int(np.asarray(prev_action[keys[0]]).shape[1])
        frozen = int(rtc.get("frozen", 0))
        ramp_rate = float(rtc.get("ramp_rate", 5.0))
        if not (1 <= overlap <= horizon):
            raise ValueError(f"rtc の overlap {overlap} はチャンク長 {horizon} 以下・1 以上にする")
        if not (0 <= frozen <= overlap):
            raise ValueError(f"rtc の frozen {frozen} は 0 以上・overlap {overlap} 以下にする")
        if ramp_rate <= 0:
            raise ValueError(f"rtc の ramp_rate {ramp_rate} は正にする")

        # 以下は Gr00tPolicy._get_action（リリース 23ace64・gr00t_policy.py L390〜421）と同じ手順に、
        # 入力の `action` と options の受け渡しだけを足したもの。
        unbatched_observations = self._unbatch_observation(observation)
        processed_inputs = []
        states = []
        for obs in unbatched_observations:
            vla_step_data = self._to_vla_step_data(obs)
            states.append(vla_step_data.states)
            messages = [{"type": MessageType.EPISODE_STEP.value, "content": vla_step_data}]
            processed_inputs.append(self.processor(messages))

        collated_inputs = self.collate_fn(processed_inputs)
        collated_inputs["inputs"]["action"] = self._rtc_action_tensor(prev_action, states, overlap, horizon)
        collated_inputs = _rec_to_dtype(collated_inputs, dtype=torch.bfloat16)

        rtc_options = {
            "action_horizon": horizon,
            "rtc_overlap_steps": overlap,
            "rtc_frozen_steps": frozen,
            "rtc_ramp_rate": ramp_rate,
        }
        with torch.inference_mode():
            model_pred = self.model.get_action(**collated_inputs, options=rtc_options)
        normalized_action = model_pred["action_pred"].float()

        batched_states = {}
        for k in self.modality_configs["state"].modality_keys:
            batched_states[k] = np.stack([s[k] for s in states], axis=0)
        unnormalized_action = self.processor.decode_action(
            normalized_action.cpu().numpy(), self.embodiment_tag, batched_states
        )
        casted_action = {key: value.astype(np.float32) for key, value in unnormalized_action.items()}
        info = {"rtc_overlap": overlap, "rtc_frozen": frozen, "server_ms": round((time.time() - t0) * 1000, 1)}
        return casted_action, info


@dataclass
class RtcServerConfig:
    model_path: str
    """学習済みモデルのフォルダ"""
    embodiment_tag: str = "new_embodiment"
    device: str = "cuda"
    host: str = "127.0.0.1"
    port: int = 5555
    strict: bool = True


def main(cfg: RtcServerConfig) -> None:
    tag = EmbodimentTag.resolve(cfg.embodiment_tag)
    if cfg.model_path.startswith("/") and not os.path.exists(cfg.model_path):
        raise FileNotFoundError(f"モデルが無い: {cfg.model_path}")
    print(f"[rtc-server] gr00t_root={_GR00T_ROOT} model={cfg.model_path} tag={tag} device={cfg.device}")
    policy = Gr00tRtcPolicy(embodiment_tag=tag, model_path=cfg.model_path, device=cfg.device, strict=cfg.strict)
    horizon = len(policy.modality_configs["action"].delta_indices)
    print(f"[rtc-server] チャンク長 {horizon} step・action keys {policy.modality_configs['action'].modality_keys}")
    server = PolicyServer(policy=policy, host=cfg.host, port=cfg.port)
    print(f"[rtc-server] ready — {cfg.host}:{cfg.port}（options.rtc が無い呼び出しは従来と同じ推論）")
    try:
        server.run()
    except KeyboardInterrupt:
        print("\n[rtc-server] 停止")


if __name__ == "__main__":
    main(tyro.cli(RtcServerConfig))
