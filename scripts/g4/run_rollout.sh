#!/usr/bin/env bash
# GR00T×OpenArm ロールアウトの記録ラッパー（Thor 上で実行）。
#
# 1試行 = 1ディレクトリ（~/rollouts/<UTC>-<tag>/）に、条件・コマンド・環境・端末出力・
# 生データを集約する。記録がバラける（bash_history / g3-logs / g4-artifacts / tmp）問題と、
# 端末出力が一切ファイル化されない問題への恒久対処。
#
# 使い方:
#   run_rollout.sh <tag> -- <eval_openarm.py への引数...>
# 例:
#   run_rollout.sh bottle-scene-matched -- \
#     --robot.type=openarm_follower --robot.port=can_fol_r \
#     --robot.side=right --robot.id=follower_right \
#     --robot.cameras='{"chest": {"type":"intelrealsense","serial_number_or_name":"344422070898","width":640,"height":480,"fps":30}}' \
#     --robot.max_relative_target=1.0 --policy_host=127.0.0.1 --policy_port=5555 \
#     --dry_run=false --fps=10 --max_steps=150 --action_horizon=8 --log_obs_every_step=true
#
# tag は英数ハイフンのみに正規化する（Vault のファイル名規則に合わせる）。
# 実機の send_action 経路には一切触れない——記録の外付けのみ。
# 生成物のうち軽量メタ（manifest/command/console/notes）は Mac から pull_rollout.sh で
# Vault へ回収する。重い生データ（traj.csv/frames）は HF へ（convert_trajlog_to_lerobot.py）。
set -euo pipefail

VENV="$HOME/projects/lerobot/.venv/bin/python"
HF="$HOME/projects/lerobot/.venv/bin/hf"
EVAL="$HOME/projects/openarm-orboh/scripts/g4/eval_openarm.py"
GR00T="$HOME/projects/Isaac-GR00T-n1.7"
MODEL="$HOME/models/gr00t-bottle-v1"

# --- 引数 ---
TAG="${1:-untagged}"; shift || true
if [[ "${1:-}" == "--" ]]; then shift; fi
TAG=$(printf '%s' "$TAG" | tr -c 'A-Za-z0-9-' '-' | sed 's/-\{2,\}/-/g; s/^-//; s/-$//')
[[ -z "$TAG" ]] && TAG="untagged"

# --- 記録フラグ guard（2026-07-24・2026-07-22 破綻=記録フラグ付け忘れの恒久対処）---
# --log_obs_every_step を明示しない実行を弾く。true は毎ステップ観測を取り直して制御周期を
# 食うため無条件強制はしない——毎ステップ記録(true・事前に dry-run で t_sec≈1/fps を確認)か
# 1Hz記録(false)かを実行者に明示させ、付け忘れ事故を構造的に防ぐ。
_has_obs_flag=false
for _a in "$@"; do case "$_a" in --log_obs_every_step*) _has_obs_flag=true ;; esac; done
if [[ "$_has_obs_flag" == false ]]; then
  echo "ERROR: --log_obs_every_step を明示してください（既定 false・付け忘れが 2026-07-22 破綻の主因）。" >&2
  echo "  毎ステップ角/frames を残す → --log_obs_every_step=true（事前に --dry_run=true で t_sec が 1/fps に収まるか確認）" >&2
  echo "  1Hz(chunk先頭)記録で可     → --log_obs_every_step=false" >&2
  exit 2
fi

TS=$(date -u +%Y%m%dT%H%M%SZ)
RUN_DIR="$HOME/rollouts/${TS}-${TAG}"
mkdir -p "$RUN_DIR"

# --- command.sh: 再実行できる形でコマンド全文を残す ---
{
  echo "#!/usr/bin/env bash"
  echo "# 記録 $TS / tag=$TAG"
  echo "# 再実行時は --run_dir を外すか別名にすること（上書き防止）"
  printf '%q ' "$VENV" "$EVAL"
  printf '%q ' "$@"
  printf '%q\n' "--run_dir=$RUN_DIR"
} > "$RUN_DIR/command.sh"
chmod +x "$RUN_DIR/command.sh"

# --- manifest.json: 環境スナップショット（条件は command.sh が正・ここは環境に集中）---
git_head()  { git -C "$1" rev-parse --short HEAD 2>/dev/null || echo "n/a"; }
git_branch(){ git -C "$1" branch --show-current 2>/dev/null || echo "detached"; }
git_dirty() { git -C "$1" status --porcelain 2>/dev/null | wc -l | tr -d ' '; }
MODEL_SHA=$(sha256sum "$MODEL/config.json" 2>/dev/null | cut -c1-16 || echo "n/a")
HF_USER=$("$HF" auth whoami 2>/dev/null | grep -oP 'user:\s*\K\S+' || echo "n/a")
PYV=$("$VENV" --version 2>&1 || echo "n/a")

jq -n \
  --arg ts "$TS" --arg tag "$TAG" --arg host "$(hostname)" \
  --arg oa_h "$(git_head  ~/projects/openarm-orboh)" --arg oa_b "$(git_branch ~/projects/openarm-orboh)" --arg oa_d "$(git_dirty ~/projects/openarm-orboh)" \
  --arg lr_h "$(git_head  ~/projects/lerobot)"       --arg lr_b "$(git_branch ~/projects/lerobot)" \
  --arg gr_h "$(git_head  "$GR00T")"                 --arg gr_b "$(git_branch "$GR00T")" \
  --arg model "$MODEL" --arg model_sha "$MODEL_SHA" \
  --arg emb "$GR00T/openarm_embodiment/openarm_config.py" \
  --arg py "$PYV" --arg hf "$HF_USER" \
  '{timestamp:$ts, tag:$tag, host:$host,
    git:{openarm_orboh:{head:$oa_h,branch:$oa_b,uncommitted_files:($oa_d|tonumber)},
         lerobot:{head:$lr_h,branch:$lr_b},
         isaac_gr00t_n17:{head:$gr_h,branch:$gr_b}},
    model:{path:$model, config_sha16:$model_sha},
    embodiment_config:$emb, python:$py, hf_user:$hf}' \
  > "$RUN_DIR/manifest.json"

# --- notes.md 雛形（人が後で目視所見を書く）---
cat > "$RUN_DIR/notes.md" <<EOF
# rollout $TS ($TAG)

## 目視所見（人が記入）
-

## 判定
-
EOF

echo "== run_dir: $RUN_DIR =="
echo "== manifest =="; cat "$RUN_DIR/manifest.json"; echo
echo "== eval_openarm を起動（console.log に tee）=="

# --- 本体を tee で呼ぶ。eval_openarm の exit code を保持する ---
set +e
"$VENV" "$EVAL" "$@" --run_dir="$RUN_DIR" 2>&1 | tee "$RUN_DIR/console.log"
rc=${PIPESTATUS[0]}
set -e
echo "== eval_openarm 終了 rc=$rc ==" | tee -a "$RUN_DIR/console.log" >/dev/null

echo
echo "記録完了: $RUN_DIR"
echo "  条件・コマンド → command.sh / manifest.json"
echo "  端末出力       → console.log / eval.log"
echo "  生データ       → traj.csv / frames/"
echo "  Mac から Vault へ回収:  pull_rollout.sh ${TS}-${TAG}"
echo "  生データを HF へ:        convert_trajlog_to_lerobot.py --trajectory-log $RUN_DIR/traj.csv --frame-log-dir $RUN_DIR/frames ..."
exit "$rc"
