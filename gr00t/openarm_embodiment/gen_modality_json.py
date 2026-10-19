"""変換済み（v2）データセットの meta/info.json からカメラを検出し、meta/modality.json を書く。

手で modality.json をコピーする手順の置き換え。カメラが増減しても info.json に追従する。

usage:
  python gen_modality_json.py <dataset_root>                   # info.json のカメラを全部使う
  OPENARM_CAMERAS=chest python gen_modality_json.py <root>     # 部分集合に絞る（無い名前は落とす）
  python gen_modality_json.py <root> --dry-run                 # 書かずに表示だけ
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from openarm_cameras import ENV_VAR, build_modality, detect_cameras, parse_cameras  # noqa: E402


def main(argv: list[str]) -> int:
    args = [a for a in argv if not a.startswith("--")]
    dry = "--dry-run" in argv
    if len(args) != 1:
        print(__doc__, file=sys.stderr)
        return 2
    root = Path(args[0]).expanduser()

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

    modality = build_modality(cameras)
    text = json.dumps(modality, ensure_ascii=False, indent=2) + "\n"
    dst = root / "meta" / "modality.json"

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
