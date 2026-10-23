#!/bin/bash
# GR00T N1.7 FT を EC2 上で走らせる（ubuntu ユーザーで実行・gr00t_n17_prepare.sh の後）。
# 学習引数は 08-06 実走（bimanual-pipeline-runbook §5-3）と同じ。ディスク watchdog と VRAM ピーク記録つき。
# checkpoint を SAVES 本（既定 8）残し、CKPT_DEST を指定すると保存のたびに HF か S3 へ上げる
# （2026-10-21 の学習は最後に 1 回だけ上げる作りで、上がらないまま停止して NVMe ごと失った）。
#
# 使い方（PC から）: scripts/ec2/ssm_run.sh start scripts/ec2/gr00t_n17_train.sh
#   必須 env: DS_ID、RUN_NAME（出力ディレクトリ名）
#   任意 env: STEPS（既定 10000）、BATCH（既定 32）、WORKERS（既定 4）、
#             OPENARM_CAMERAS / OPENARM_EMBODIMENT（prepare と同じ値にする）、
#             HF_MODEL_REPO（指定すると学習後にモデルを上げる・model/private で作成済みのこと）、
#             CKPT_DEST（指定すると checkpoint-* を保存のたびに上げる。hf:<repo>（model/private で作成済み）か
#               s3://<bucket>/<prefix>（$RUN_NAME/ の下に置く）。1 本 約 12.6GB × SAVES 本になる。
#               HF の無料枠は private の容量に上限があるので、8 本残すなら s3://orboh-models/gr00t-ckpt を推奨）、
#             SAVES（残す checkpoint 数・既定 8。save_steps = STEPS / SAVES）、SEED（既定 42）、
#             USE_WANDB（既定 1。ubuntu の ~/.netrc に api.wandb.ai が無いと起動前に止める）、
#             WANDB_PROJECT（既定 gr00t-openarm）
# 前提: nvidia/Cosmos-Reason2-2B（gated）にトークンのアカウントがアクセスできること。
#       N1.7 は全チェックポイントがこれを backbone として読む。
set -eo pipefail

: "${DS_ID:?DS_ID を指定する}" "${RUN_NAME:?RUN_NAME（例: ft_bi_takeei_wood_l1_v1）を指定する}"
WORK=${WORK:-/opt/dlami/nvme/gr00t}
G=$WORK/Isaac-GR00T-n17
DS=$WORK/hf/lerobot/$DS_ID
OUT=$WORK/out/$RUN_NAME
LOG=$WORK/logs
STEPS=${STEPS:-10000}
BATCH=${BATCH:-32}
WORKERS=${WORKERS:-4}
SAVES=${SAVES:-8}
SEED=${SEED:-42}
USE_WANDB=${USE_WANDB:-1}
WANDB_PROJECT=${WANDB_PROJECT:-gr00t-openarm}
SAVE_STEPS=$((STEPS / SAVES))
MIN_FREE_GB=15
export OPENARM_EMBODIMENT=${OPENARM_EMBODIMENT:-bimanual}
export OPENARM_CAMERAS=${OPENARM_CAMERAS:-chest,wrist_right}
export UV_CACHE_DIR=$WORK/uv-cache HF_HOME=$WORK/hf PATH=$HOME/.local/bin:$PATH
HF_TOKEN="$(cat "$HOME/.cache/huggingface/token")"
export HF_TOKEN   # set -x より前に置く

[ $((SAVE_STEPS * SAVES)) -eq "$STEPS" ] || { echo "STEPS=$STEPS が SAVES=$SAVES で割り切れない"; exit 1; }
wandb_args=()
if [ "$USE_WANDB" = 1 ]; then
  # 3 時間回してから曲線が無いと分かるのを防ぐ
  grep -q api.wandb.ai "$HOME/.netrc" 2>/dev/null || { echo "W&B の認証が無い（~/.netrc）。USE_WANDB=0 で無効化できる"; exit 1; }
  # run 名は $OUT の名前（= RUN_NAME）になる。--experiment_name を渡すと $OUT/$RUN_NAME/ に一段深く出てしまうので渡さない
  wandb_args=(--use_wandb --wandb_project "$WANDB_PROJECT")
fi

mkdir -p "$OUT" "$LOG"
cd "$G"

# watchdog: 空きが MIN_FREE_GB を切ったら学習を止める・VRAM ピークを記録する
watchdog() {
  local peak=0 used avail
  while sleep 30; do
    used=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | sort -n | tail -1)
    avail=$(df -BG --output=avail "$WORK" | tail -1 | tr -dc 0-9)
    [ "$used" -gt "$peak" ] && peak=$used
    echo "$(date -u +%FT%TZ) vram=${used}MiB peak=${peak}MiB avail=${avail}G" >> "$LOG/watchdog.log"
    if [ "$avail" -lt "$MIN_FREE_GB" ]; then
      echo "$(date -u +%FT%TZ) 空き ${avail}G < ${MIN_FREE_GB}G のため学習を止める" >> "$LOG/watchdog.log"
      pkill -f launch_finetune.py
    fi
  done
}
watchdog &
WD=$!

# 保存が終わった checkpoint を HF に上げる。CheckpointFormatCallback が processor と experiment_cfg を
# コピーし終えてから（＝ experiment_cfg/ がある）上げる。HF Trainer の push_to_hub はこのコピーより
# 前に上げてしまい、単体で読めない checkpoint になるので使わない。
push_one() {
  case "$CKPT_DEST" in
    hf:*) hf upload "${CKPT_DEST#hf:}" "$1" "$2" --repo-type model --commit-message "$RUN_NAME $2" ;;
    s3://*) aws s3 sync "$1" "${CKPT_DEST%/}/$RUN_NAME/$2" --only-show-errors ;;
    *) echo "CKPT_DEST は hf:<repo> か s3://... : $CKPT_DEST"; return 1 ;;
  esac
}
push_ckpts() {
  local d n
  for d in "$OUT"/checkpoint-*; do
    [ -d "$d/experiment_cfg" ] || continue
    n=$(basename "$d")
    [ -f "$LOG/pushed.$RUN_NAME.$n" ] && continue
    # 初めて見えた周回では上げない（processor のコピーが続いているかもしれない）。最終周は ${1:-} = now
    if [ ! -f "$LOG/seen.$RUN_NAME.$n" ] && [ "${1:-}" != now ]; then touch "$LOG/seen.$RUN_NAME.$n"; continue; fi
    if push_one "$d" "$n" >> "$LOG/push_ckpts.log" 2>&1; then
      touch "$LOG/pushed.$RUN_NAME.$n"
      echo "$(date -u +%FT%TZ) pushed $n" >> "$LOG/push_ckpts.log"
    else
      echo "$(date -u +%FT%TZ) push 失敗 $n（次の周回で再試行）" >> "$LOG/push_ckpts.log"
    fi
  done
}
PUSHER=
if [ -n "${CKPT_DEST:-}" ]; then
  ( while sleep 60; do push_ckpts; done ) &
  PUSHER=$!
fi
trap 'kill $WD $PUSHER 2>/dev/null' EXIT

set -x
.venv/bin/python gr00t/experiment/launch_finetune.py \
  --base_model_path nvidia/GR00T-N1.7-3B \
  --dataset_path "$DS" \
  --embodiment_tag NEW_EMBODIMENT \
  --modality_config_path openarm_embodiment/openarm_config.py \
  --num_gpus 1 --output_dir "$OUT" \
  --save_steps "$SAVE_STEPS" --save_total_limit "$SAVES" --save_only_model --max_steps "$STEPS" \
  --warmup_ratio 0.05 --weight_decay 1e-5 --learning_rate 1e-4 \
  --global_batch_size "$BATCH" \
  --color_jitter_params brightness 0.3 contrast 0.4 saturation 0.5 hue 0.08 \
  --dataloader_num_workers "$WORKERS" \
  --seed "$SEED" "${wandb_args[@]}"
echo TRAIN_EXIT=$?
tail -1 "$LOG/watchdog.log"
ls -d "$OUT"/checkpoint-* | tee "$LOG/checkpoints.$RUN_NAME.txt"

if [ -n "$PUSHER" ]; then
  kill "$PUSHER" 2>/dev/null || true
  push_ckpts now
  n_local=$(ls -d "$OUT"/checkpoint-* | wc -l)
  n_pushed=$(ls "$LOG"/pushed."$RUN_NAME".checkpoint-* 2>/dev/null | wc -l)
  echo "CKPT_PUSHED=$n_pushed/$n_local"
  [ "$n_pushed" -eq "$n_local" ] || { echo "CKPT_PUSH_INCOMPLETE（NVMe 上に残っている。止める前に上げ直す）"; exit 1; }
fi

if [ -n "${HF_MODEL_REPO:-}" ]; then
  # checkpoint-*（optimizer 込み・24GB）は上げない。--exclude は 2 回書くと前が捨てられるので 1 回だけ
  hf upload "$HF_MODEL_REPO" "$OUT" . --repo-type model --exclude "checkpoint-*"
  # loss の記録（trainer_state.json）は最後の checkpoint にしか無いので、モデルと一緒に上げる
  hf upload "$HF_MODEL_REPO" "$OUT/checkpoint-$STEPS/trainer_state.json" trainer_state.json --repo-type model
  hf models info "$HF_MODEL_REPO" | grep -o '"rfilename": "[^"]*safetensors"' || true
  echo UPLOAD_DONE
fi
echo DONE_MARKER
