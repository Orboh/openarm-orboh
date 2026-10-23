#!/bin/bash
# GR00T N1.7 FT の準備を EC2 上で一気に行う（ubuntu ユーザーで実行・GPU 不要）。
#   clone → LFS の wheel 取得 → uv sync → openarm_embodiment 配置・patches 適用 → v3→v2 変換
#   → modality.json 生成 → norm stats → dataloader 疎通
# 置き場所は既定でローカル NVMe（/opt/dlami/nvme）。止めると消えるので、起動のたびに流し直す前提。
# 2026-10-21 に orboh-sim（g5.8xlarge）で全段通過を確認済み。
#
# 使い方（PC から）: scripts/ec2/ssm_run.sh start scripts/ec2/gr00t_n17_prepare.sh
#   必須 env: DS_ID（HF のデータセット ID・v3.0）
#   任意 env: OPENARM_CAMERAS（既定 chest,wrist_right）、OPENARM_EMBODIMENT（既定 bimanual）、
#             WORK（既定 /opt/dlami/nvme/gr00t）、GR00T_REF（既定 51d4c89）
# openarm_embodiment と patches（gr00t/patches/*.patch）は ssm_run.sh が $WORK/emb.b64 に置く。
set -eo pipefail

: "${DS_ID:?DS_ID（例: Kota0612/bi_openarm_takeei_wood_l1_chest_wristr_wristl_v1）を指定する}"
WORK=${WORK:-/opt/dlami/nvme/gr00t}
GR00T_REF=${GR00T_REF:-51d4c89}   # 2026-08-20 main。video_backend 引数が無い世代（check_dataloader は両対応）
G=$WORK/Isaac-GR00T-n17
DS=$WORK/hf/lerobot/$DS_ID
export OPENARM_EMBODIMENT=${OPENARM_EMBODIMENT:-bimanual}
export OPENARM_CAMERAS=${OPENARM_CAMERAS:-chest,wrist_right}
# uv のキャッシュと HF のキャッシュも NVMe に置く（ルートの EBS は他用途で埋まっている）
export UV_CACHE_DIR=$WORK/uv-cache HF_HOME=$WORK/hf PATH=$HOME/.local/bin:$PATH
HF_TOKEN="$(cat "$HOME/.cache/huggingface/token")"
export HF_TOKEN   # set -x より前に置く（ログに平文で残さない）

step() { echo "=== [$(date -u +%H:%M:%S)] $*"; }
set -x

step "1/7 clone ($GR00T_REF)"
mkdir -p "$WORK"
[ -d "$G/.git" ] || GIT_LFS_SKIP_SMUDGE=1 git clone -q https://github.com/NVIDIA/Isaac-GR00T "$G"
cd "$G"
# -f: 前回当てた patches を捨ててから当て直す（openarm_embodiment は未追跡なので消えない）
git checkout -q -f "$GR00T_REF"
# wheel（torchcodec など）は LFS 管理。SKIP_SMUDGE のままだとポインタファイルになり uv sync が zip エラーで落ちる
git lfs pull --include "*.whl"

step "2/7 openarm_embodiment・patches"
rm -rf openarm_embodiment patches
base64 -d "$WORK/emb.b64" | tar xzf -
ls openarm_embodiment
# seed 引数など upstream に無い変更（Orboh の fork が無いので patch で持つ）
for p in patches/*.patch; do git apply "$p" && echo "applied: $p"; done

step "3/7 uv sync"
GIT_LFS_SKIP_SMUDGE=1 uv sync --python 3.12 2>&1 | tail -3
.venv/bin/python -c 'import torch; print("torch", torch.__version__, "cuda", torch.cuda.is_available())'

step "4/7 v3→v2 変換"
if [ -f "$DS/meta/episodes.jsonl" ]; then
  echo "変換済みなのでスキップ: $DS"
else
  (cd scripts/lerobot_conversion \
    && { [ -d .venv ] || uv venv -q --python 3.10; } \
    && GIT_LFS_SKIP_SMUDGE=1 uv pip install -q -e . \
    && .venv/bin/python convert_v3_to_v2.py --repo-id "$DS_ID" 2>&1 | grep -v 'it/s' | tail -5)
fi
echo CONVERT_EXIT=0

step "5/7 modality.json"
.venv/bin/python openarm_embodiment/gen_modality_json.py "$DS"

step "6/7 norm stats"
# v3 由来の stats.json があると再計算を飛ばすので退避する
[ -f "$DS/meta/stats.json" ] && [ ! -f "$DS/meta/relative_stats.json" ] && mv "$DS/meta/stats.json" "$DS/meta/stats.json.v3-orig"
.venv/bin/python gr00t/data/stats.py --dataset-path "$DS" --embodiment-tag NEW_EMBODIMENT \
  --modality-config-path openarm_embodiment/openarm_config.py 2>&1 | grep -v 'it/s' | tail -5

step "7/7 dataloader 疎通"
.venv/bin/python openarm_embodiment/check_dataloader.py "$DS" 2>&1 | grep -v 'it/s' | tail -20

df -h / "$WORK" | tail -2
echo PREPARE_DONE
