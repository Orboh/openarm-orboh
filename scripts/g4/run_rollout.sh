#!/usr/bin/env bash
# GR00T×OpenArm ロールアウトの記録ラッパー（Thor 上で実行）。
#
# ---- 推論ランナーは2本ある。取り違えないこと --------------------------------
#   このファイル scripts/g4/run_rollout.sh
#     → 自前の g4/eval_openarm.py を起動。GR00T(外部 PolicyServer)用。実走済み。
#   scripts/run_rollout_lerobot.sh
#     → lerobot 標準の `lerobot-rollout` を起動。ACT 等の lerobot ポリシー用。
# -----------------------------------------------------------------------------
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
#     --lang_instruction="Pick up the bottle and place it in the bin" \
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
MODEL="${MODEL:-$HOME/models/gr00t-bottle-v2}"

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

# --- 言語指示 guard（2026-09-08）---
# eval_openarm.py の既定値（ボトル用の文）に頼ると、別タスクのモデルでも学習時と違う指示文が
# 送られる（実測: ボトル 13 本・テープ 13 本の全 rollout が同一の既定文。テープは収集時
# "vinyl tape"・推論時 "bottle"）。収集時の single_task と同じ文をタスクごとに明示させ、
# 使った文を manifest.json に残す。
_lang=""
for _a in "$@"; do case "$_a" in --lang_instruction=*) _lang="${_a#*=}" ;; esac; done
if [[ -z "$_lang" ]]; then
  echo "ERROR: --lang_instruction=\"<収集時の single_task と同じ文>\" を明示してください（2026-09-08 から必須）。" >&2
  echo "  例: --lang_instruction=\"Pick up the vinyl tape and place it in the bin\"" >&2
  exit 2
fi

# --- 開始姿勢 YAML の自動配線（収集時と同じ構えから始めるため）---------------
# eval_openarm.py は draccus で RobotConfig を組むので --robot.initial_pose_path を
# そのまま受ける。接続時整列そのものは connect() 内の align_on_connect(既定 true)で
# 既に効いているが、**キャプチャ済み ready 姿勢の YAML は渡さないと使われない**。
# 渡さないと公式既定(肘 π/5=36°)に落ち、収集側 record_pick_place_openarm.sh が
# YAML を渡していた場合に**推論だけ別の構えから始まる**。YAML が無い今は両方とも
# 公式既定で一致しているので実害は出ていないが、キャプチャした瞬間に静かにずれる。
# --log_obs_every_step のような明示強制にはしない: 渡すか渡さないかにトレードオフが
# 無く、あるなら常に渡すのが正しいため(自動付与し、付けた事実を console.log に残す)。
_side=""; _has_pose=false
for _a in "$@"; do
  case "$_a" in
    --robot.side=*)              _side="${_a#*=}" ;;
    --robot.initial_pose_path=*) _has_pose=true ;;
  esac
done
POSE_ARGS=()
if [[ "$_has_pose" == false && -n "$_side" ]]; then
  _pose="$HOME/projects/openarm-orboh/config/initial_pose_${_side}.yaml"
  if [[ -f "$_pose" ]]; then
    POSE_ARGS+=(--robot.initial_pose_path="$_pose")
    echo "[rollout] 開始姿勢 YAML を自動付与: $_pose"
  else
    echo "[rollout] 開始姿勢 YAML なし → 公式既定(肘36°)で整列。収集側も同条件であること: $_pose"
  fi
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
  # 自動付与した開始姿勢も残す。ここに書かないと再実行時に構えが変わる
  [ ${#POSE_ARGS[@]} -gt 0 ] && printf '%q ' "${POSE_ARGS[@]}"
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
  --arg lang "$_lang" \
  --arg py "$PYV" --arg hf "$HF_USER" \
  '{timestamp:$ts, tag:$tag, host:$host,
    git:{openarm_orboh:{head:$oa_h,branch:$oa_b,uncommitted_files:($oa_d|tonumber)},
         lerobot:{head:$lr_h,branch:$lr_b},
         isaac_gr00t_n17:{head:$gr_h,branch:$gr_b}},
    model:{path:$model, config_sha16:$model_sha},
    lang_instruction:$lang,
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
"$VENV" "$EVAL" "$@" ${POSE_ARGS[@]+"${POSE_ARGS[@]}"} --run_dir="$RUN_DIR" 2>&1 | tee "$RUN_DIR/console.log"
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
