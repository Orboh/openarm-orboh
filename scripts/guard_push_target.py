#!/usr/bin/env python3
"""統合したデータセットを push する前に、押し先が安全か確かめる。

設計正本: orboh-company-vault
  07Member's-free-space/Kento/02-Tech-Plan/02VLA/collection-recording-manual.md
  (§中止条件 / 実装バックログ5)

守っているのは1点:

  **push 先の HF リポジトリが既に存在するなら止める。**

`lerobot-edit-dataset merge` の `--new_repo_id` は自動採番されず、`push_to_hub`
は `create_repo(exist_ok=True)` と `upload_folder`(削除パターン無し)で既存リポジトリ
に重ね書きする。つまり同じ名前へ2度目を押すと、**古い chunk が誰からも参照されない
まま残る**——ファイルは増えるのにエラーは出ず、データセットとしては静かに壊れる。
名前が衝突したら版を上げるのが正しく、機械が止めないと気づけない類の事故なので
ここで落とす。

ローカル収集先の上書きはこのスクリプトの担当ではない(upstream が
`dataset_metadata.py` の `mkdir(exist_ok=False)` で既に禁じており、`stamp_repo_id()`
が Python 内で名前を変えるので shell 側の事前チェックは恒久的に空振りする)。

使い方:
  guard_push_target.py --repo-id Orboh/openarm_bottle_pnp_chest_v2
  guard_push_target.py --repo-id ... --stamp-manifest ~/collections/<run>   # HF 身元を manifest へ
  guard_push_target.py --repo-id ... --allow-existing                       # 重ね書きを承知で通す
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

# 命名規則: openarm_<task>_<cams>_v<N>。カメラ構成を名前に含めるのは、単眼と
# 多カメラでは入力次元が変わり別モデルになるため、混ざると学習が壊れるから。
NAME_PATTERN = re.compile(r"^openarm_[a-z0-9]+(?:_[a-z0-9]+)*_v\d+$")


def out(msg: str) -> None:
    print(f"[guard] {msg}")


def err(msg: str) -> None:
    print(f"[guard] {msg}", file=sys.stderr)


def check_naming(repo_id: str) -> bool:
    """命名規則に合っているか。合わなくても止めない(警告のみ)。"""
    name = repo_id.split("/", 1)[1] if "/" in repo_id else repo_id
    if NAME_PATTERN.match(name):
        out(f"命名規則 OK: {name}")
        return True
    err(
        f"警告: 命名規則から外れている: {name}\n"
        "[guard]   期待する形: openarm_<task>_<cams>_v<N>"
        "（例 openarm_bottle_pnp_chest_v2）\n"
        "[guard]   カメラ構成を名前に入れるのは、単眼と多カメラで入力次元が変わり"
        "別モデルになるため。"
    )
    return False


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--repo-id", required=True, help="push 先（例 Orboh/openarm_bottle_pnp_chest_v2）")
    parser.add_argument(
        "--allow-existing",
        action="store_true",
        help="既存リポジトリへの重ね書きを承知で通す（orphan chunk が残る）",
    )
    parser.add_argument(
        "--stamp-manifest",
        nargs="*",
        default=[],
        metavar="RUN_DIR",
        help="確認できた HF 身元と push 先を、指定した run の manifest.json に書く",
    )
    args = parser.parse_args(argv)

    try:
        from huggingface_hub import HfApi
        from huggingface_hub.errors import HfHubHTTPError
    except ImportError:
        err("huggingface_hub が入っていない。lerobot の venv で走らせること。")
        return 1

    api = HfApi()
    check_naming(args.repo_id)

    # 身元の確認。トークンが無いまま 50ep 録り終えてから 403 で止まるのが最悪の
    # 順序なので、押す前にここで落とす。
    try:
        me = api.whoami()
    except Exception as exc:
        err(
            f"HF の身元を確認できない（{exc}）。トークンが無いか無効。\n"
            "[guard]   `huggingface-cli login` か HF_TOKEN を設定してから再実行する。"
        )
        return 1
    identity = me.get("name") or me.get("email") or "unknown"
    auth = (me.get("auth") or {}).get("accessToken") or {}
    token_name = auth.get("displayName") or "(名前なし)"
    token_role = auth.get("role") or "(role 不明)"
    out(f"HF 身元: {identity} / トークン: {token_name} / role: {token_role}")

    # 本題。存在したら止める。
    try:
        exists = api.repo_exists(args.repo_id, repo_type="dataset")
    except HfHubHTTPError as exc:
        err(f"push 先の存在を確認できない（{exc}）。ネットワークか権限の問題。")
        return 1

    if exists:
        if not args.allow_existing:
            err(
                f"停止: push 先が既に存在する: {args.repo_id}\n"
                "[guard]   push_to_hub は create_repo(exist_ok=True) + upload_folder"
                "（削除パターン無し）で重ね書きするので、\n"
                "[guard]   古い chunk が誰からも参照されないまま残る（orphan chunk）。"
                "エラーは出ないがデータセットは静かに壊れる。\n"
                "[guard]   正しい対処は版を上げること（..._v2 → ..._v3）。\n"
                "[guard]   重ね書きを承知で進めるなら --allow-existing。"
            )
            return 1
        err(f"警告: 既存の {args.repo_id} へ重ね書きする（--allow-existing）。orphan chunk が残る。")
    else:
        out(f"push 先は未作成: {args.repo_id}（重ね書きの心配なし）")

    # トークンがそのリポジトリへ書けるかは、実際に押すまで確定しない。
    # 細粒度トークンはリポジトリ単位でスコープされるため、whoami では分からない。
    out(
        "注意: このリポジトリへ書けるかは実際の push でしか確定しない"
        "（細粒度トークンはリポジトリ単位スコープ）。"
    )
    out("      本番の前に小さなテスト push を1回通しておくこと。")

    for run_dir in args.stamp_manifest:
        path = Path(run_dir).expanduser() / "manifest.json"
        try:
            manifest = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            err(f"警告: manifest を更新できない（{exc}）: {path}")
            continue
        artifacts = manifest.setdefault("artifacts", {})
        artifacts["hf_identity"] = {"name": identity, "token": token_name, "role": token_role}
        artifacts["push_target"] = args.repo_id
        path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        out(f"manifest に HF 身元と push 先を記録: {path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
