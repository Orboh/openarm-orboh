#!/bin/bash
# 学習で残した checkpoint-* ごとに GR00T 同梱の open_loop_eval.py を回し、誤差の表と軌跡の図を 1 フォルダにまとめる
# （ubuntu ユーザーで実行・gr00t_n17_train.sh の後）。評価そのものは open_loop_eval.py のまま、ここは回して集めるだけ。
#
# 使い方（PC から）: scripts/ec2/ssm_run.sh start scripts/ec2/gr00t_n17_openloop.sh
#   必須 env: DS_ID（検証用の手本が入ったデータセット・v2 変換と modality.json 済み）、RUN_NAME、
#             TRAJ_IDS（検証に使う手本の episode 番号・カンマ区切り。例: 3,17,42）
#   任意 env: CKPTS（既定は $OUT/checkpoint-* 全部。CKPT_DEST から落とした置き場を渡してもよい）、
#             EVAL_STEPS（1 本あたりの最大ステップ・既定 400）、OPENARM_CAMERAS / OPENARM_EMBODIMENT（学習と同じ値）
# 出力: $WORK/eval/$RUN_NAME/summary.tsv（checkpoint × 手本の MSE/MAE）と checkpoint-*/traj_*.jpeg
# 注意: 学習に使った手本で回すと誤差は小さく出て選定の役に立たない。学習から外した手本を使う。
set -eo pipefail

: "${DS_ID:?DS_ID を指定する}" "${RUN_NAME:?RUN_NAME を指定する}" "${TRAJ_IDS:?TRAJ_IDS（例: 3,17,42）を指定する}"
WORK=${WORK:-/opt/dlami/nvme/gr00t}
G=$WORK/Isaac-GR00T-n17
DS=$WORK/hf/lerobot/$DS_ID
OUT=$WORK/out/$RUN_NAME
EVAL=$WORK/eval/$RUN_NAME
EVAL_STEPS=${EVAL_STEPS:-400}
CKPTS=${CKPTS:-$OUT}
export OPENARM_EMBODIMENT=${OPENARM_EMBODIMENT:-bimanual}
export OPENARM_CAMERAS=${OPENARM_CAMERAS:-chest,wrist_right}
export UV_CACHE_DIR=$WORK/uv-cache HF_HOME=$WORK/hf PATH=$HOME/.local/bin:$PATH

mkdir -p "$EVAL"
cd "$G"
read -ra traj <<< "${TRAJ_IDS//,/ }"
printf 'checkpoint\ttraj\tmse\tmae\n' > "$EVAL/summary.tsv"

# checkpoint-<step> を step の数値順に回す
for ck in $(ls -d "$CKPTS"/checkpoint-* | sort -t- -k2 -n); do
  n=$(basename "$ck")
  echo "=== [$(date -u +%H:%M:%S)] $n"
  mkdir -p "$EVAL/$n"
  # --save_plot_path を渡すと手本ごとの図が同じ名前で上書きされるので、既定の置き場に出させてから移す
  rm -rf /tmp/open_loop_eval
  .venv/bin/python gr00t/eval/open_loop_eval.py \
    --model_path "$ck" --dataset_path "$DS" --embodiment_tag new_embodiment \
    --traj_ids "${traj[@]}" --steps "$EVAL_STEPS" 2>&1 | tee "$EVAL/$n/eval.log" | grep -E "MSE|MAE" || true
  mv /tmp/open_loop_eval/*.jpeg "$EVAL/$n/" 2>/dev/null || true
  # open_loop_eval.py のログ行「MSE for trajectory 3: 0.01, MAE: 0.05」から表を作る
  sed -nE "s/.*MSE for trajectory ([0-9]+): ([^,]+), MAE: (.*)/$n\t\1\t\2\t\3/p" "$EVAL/$n/eval.log" >> "$EVAL/summary.tsv"
done

echo "=== checkpoint ごとの平均（MSE の小さい順）"
awk -F'\t' 'NR>1 {s[$1]+=$3; a[$1]+=$4; c[$1]++} END {for (k in s) printf "%s\t%.6f\t%.6f\n", k, s[k]/c[k], a[k]/c[k]}' \
  "$EVAL/summary.tsv" | sort -t$'\t' -k2 -g | tee "$EVAL/summary_mean.tsv"
echo "出力: $EVAL"
echo DONE_MARKER
