#!/bin/bash
# PC から EC2 に SSM でスクリプトを送り、ubuntu ユーザーのバックグラウンドで走らせる。
# 対話の start-session を使わない（Claude Code からも同じ手順で動かせる）。
#
# 使い方:
#   scripts/ec2/ssm_run.sh start <script.sh>   # 送って nohup 起動（openarm_embodiment と patches も一緒に送る）
#   scripts/ec2/ssm_run.sh tail  <script.sh>   # 走っているか + ログ末尾
#   scripts/ec2/ssm_run.sh exec  <script.sh>   # root でその場実行して出力を待つ（短い確認用）
#   start 時は環境変数 DS_ID / RUN_NAME / OPENARM_CAMERAS / OPENARM_EMBODIMENT / STEPS / BATCH /
#   WORKERS / HF_MODEL_REPO / CKPT_DEST / SAVES / SEED / USE_WANDB / WANDB_PROJECT / GR00T_REF /
#   CKPTS / TRAJ_IDS / EVAL_STEPS を引き継ぐ（指定したものだけ）。
# 既定: AWS_PROFILE=kota0（共用ユーザー spot）、IID=orboh-sim
set -euo pipefail

export AWS_PROFILE=${AWS_PROFILE:-kota0}
IID=${IID:-i-0db430a2eff6cd78c}
WORK=${WORK:-/opt/dlami/nvme/gr00t}
REPO=$(cd "$(dirname "$0")/../.." && pwd)
MODE=${1:?start|tail|exec}
SCRIPT=${2:?スクリプトのパス}
NAME=$(basename "$SCRIPT" .sh)
PASS_VARS="DS_ID RUN_NAME OPENARM_CAMERAS OPENARM_EMBODIMENT STEPS BATCH WORKERS HF_MODEL_REPO GR00T_REF
  CKPT_DEST SAVES SEED USE_WANDB WANDB_PROJECT CKPTS TRAJ_IDS EVAL_STEPS"

# root のスクリプト本文を送り、終わるまで待って出力を表示する
send() {
  local body=$1 cid st
  cid=$(aws ssm send-command --instance-ids "$IID" --document-name AWS-RunShellScript \
    --parameters "{\"commands\":[\"echo $(base64 -w0 <<<"$body") | base64 -d | bash\"],\"executionTimeout\":[\"3600\"]}" \
    --query Command.CommandId --output text)
  while :; do
    st=$(aws ssm get-command-invocation --command-id "$cid" --instance-id "$IID" --query Status --output text 2>/dev/null || true)
    case "$st" in Pending|InProgress|Delayed|"") sleep 5 ;; *) break ;; esac
  done
  aws ssm get-command-invocation --command-id "$cid" --instance-id "$IID" \
    --query '[Status,StandardOutputContent,StandardErrorContent]' --output text
  [ "$st" = Success ]
}

case "$MODE" in
  start)
    envs=""
    for v in $PASS_VARS; do [ -n "${!v:-}" ] && envs+="export $v=$(printf %q "${!v}"); "; done
    emb=$(tar czf - --exclude=__pycache__ -C "$REPO/gr00t" openarm_embodiment patches | base64 -w0)
    send "set -e
mkdir -p $WORK/logs && chown -R ubuntu:ubuntu $WORK
echo $emb > $WORK/emb.b64
echo $(base64 -w0 "$SCRIPT") | base64 -d > $WORK/$NAME.sh
echo '$envs' > $WORK/$NAME.env
chown ubuntu:ubuntu $WORK/emb.b64 $WORK/$NAME.sh $WORK/$NAME.env
sudo -u ubuntu -H bash -c 'source $WORK/$NAME.env; nohup bash $WORK/$NAME.sh > $WORK/logs/$NAME.log 2>&1 &'
sleep 2; echo started: $WORK/logs/$NAME.log"
    ;;
  tail)
    send "pgrep -u ubuntu -f $WORK/$NAME.sh >/dev/null && echo RUNNING || echo STOPPED
tr '\r' '\n' < $WORK/logs/$NAME.log | grep -vE 'it/s\]?\$|^\s*\$' | tail -${LINES_TAIL:-30}
[ -f $WORK/logs/watchdog.log ] && tail -1 $WORK/logs/watchdog.log || true"
    ;;
  exec)
    send "$(cat "$SCRIPT")"
    ;;
  *) echo "unknown mode: $MODE" >&2; exit 2 ;;
esac
