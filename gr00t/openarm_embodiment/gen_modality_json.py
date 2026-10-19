"""変換済み（v2）データセットの meta/info.json からカメラ・レイアウトを検出し、meta/modality.json を書く。

手で modality.json をコピーする手順の置き換え。カメラが増減しても info.json に追従する。
レイアウト（単腕8DoF/両腕16DoF）は observation.state の names から自動判定する
（OPENARM_EMBODIMENT を指定した場合は判定結果との一致を検証するだけに使う）。

usage:
  python gen_modality_json.py <dataset_root>                        # info.json から自動判定
  OPENARM_CAMERAS=chest python gen_modality_json.py <root>          # カメラを部分集合に絞る（無い名前は落とす）
  OPENARM_EMBODIMENT=bimanual python gen_modality_json.py <root>    # 想定レイアウトを明示（不一致なら FAIL）
  python gen_modality_json.py <root> --dry-run                      # 書かずに表示だけ
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from openarm_cameras import ENV_VAR, build_modality, detect_cameras, parse_cameras  # noqa: E402
from openarm_layout import ENV_VAR as LAYOUT_ENV_VAR  # noqa: E402
from openarm_layout import detect_layout, state_names  # noqa: E402


def main(argv: list[str]) -> int:
    args = [a for a in argv if not a.startswith("--")]
    dry = "--dry-run" in argv
    if len(args) != 1:
        print(__doc__, file=sys.stderr)
        return 2
    root = Path(args[0]).expanduser()

    detected_layout = detect_layout(root)
    want_layout = os.environ.get(LAYOUT_ENV_VAR)
    if want_layout and want_layout != detected_layout:
        print(
            f"[FAIL] {LAYOUT_ENV_VAR}={want_layout} だがデータセットは {detected_layout}"
            f"（observation.state の names: {state_names(detected_layout)}）",
            file=sys.stderr,
        )
        return 1
    layout = detected_layout

    available = detect_cameras(root)
    raw = os.environ.get(ENV_VAR)
    if raw:
        cameras = parse_cameras(raw)
        missing = [c for c in cameras if c not in available]
        if missing:
            print(f"[FAIL] {ENV_VAR} のカメラがデータセットに無い: {missing}（データセットにあるのは {available}）", file=sys.stderr)
            return 1
    else:
        cameras = available

    modality = build_modality(cameras, layout)
    text = json.dumps(modality, ensure_ascii=False, indent=2) + "\n"
    dst = root / "meta" / "modality.json"

    print(f"[layout] dataset={layout} state_dim={16 if layout == 'bimanual' else 8} keys={list(modality['state'].keys())}")
    print(f"[cameras] dataset={available} -> modality.json={cameras}")
    if dry:
        print(text, end="")
        return 0
    if dst.exists() and dst.read_text(encoding="utf-8") != text:
        print(f"[NOTE] 既存の {dst} を上書きする（内容が違う）")
    dst.write_text(text, encoding="utf-8")
    print(f"[OK] wrote {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
