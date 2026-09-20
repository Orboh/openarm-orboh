#!/usr/bin/env bash
# openarm-orboh を入口にした「1回の clone」から、テレオペ+収集ができる状態まで
# 面倒を見るブートストラップ。兄弟リポ(lerobot / XRoboToolkit-Orin-Video-Sender)の
# clone・lerobot venv 構築・sender ビルド・scripts/setup.sh をまとめて実行する。
#
# なぜ要るか: 従来の README は openarm-orboh 自身の submodule/apt/udev しかカバーせず、
# 遠隔テレオペ・収集の実行系(run_teleop_remote_*.sh 等)が前提とする兄弟リポは
# 手動で $HOME/projects/ 配下に揃っている前提で書かれていた。「clone一発」では
# ゴールの状態に届かなかった穴を塞ぐのがこのスクリプト。
#
# 使い方:
#   bash scripts/bootstrap.sh                 # フル(Thor=follower実機側相当。3リポ+lerobot venv+senderビルド+setup.sh)
#   bash scripts/bootstrap.sh --operator       # 操作側PC用(sender の clone/build を省略。lerobot+openarm-orboh のみ)
#   bash scripts/bootstrap.sh --check          # 読み取り専用の状態レポートのみ(一切変更しない・fetch もしない)
#   bash scripts/bootstrap.sh --skip-setup     # scripts/setup.sh(apt/teleopビルド/udev・sudo要)を飛ばす
#   bash scripts/bootstrap.sh --shallow        # 兄弟リポを --depth 1 で clone(fresh-clone 検証専用。本番機では使わない)
#   PROJECTS_DIR=/path bash scripts/bootstrap.sh   # リポの基底を上書き(既定 $HOME/projects。テスト用途専用)
#
# 冪等性: 何度実行しても安全。clone はディレクトリ不在時のみ、venv 構築は .venv 不在時のみ、
# sender ビルドはバイナリ不在時のみ行う。既存リポには fetch と状態報告だけを行い、
# checkout / reset / pull / make clean / 削除は一切しない(喪失ゼロ優先)。dirty なリポは
# 警告して触らない。
#
# 終了コード: 0=全緑 / 1=WARN あり(このスクリプト自身が壊れた場合を除き、途中で
# 止めるのではなく最後まで進めて WARN として報告する「preflight」思想)。
set -u

# ---- 設定ブロック(兄弟リポの URL・ブランチ pin。変更はここ1箇所) --------------
OPENARM_BRANCH="${OPENARM_BRANCH:-main}"
LEROBOT_URL="${LEROBOT_URL:-https://github.com/Orboh/lerobot.git}"
LEROBOT_BRANCH="${LEROBOT_BRANCH:-feat/openarm-remote-teleop}"
SENDER_URL="${SENDER_URL:-https://github.com/Orboh/XRoboToolkit-Orin-Video-Sender.git}"
SENDER_BRANCH="${SENDER_BRANCH:-vector-x86}"
PROJECTS_DIR="${PROJECTS_DIR:-$HOME/projects}"

# ---- 引数 ---------------------------------------------------------------------
OPERATOR_MODE=false
CHECK_ONLY=false
SKIP_SETUP=false
SHALLOW=false
usage() {
  sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}
for arg in "$@"; do
  case "$arg" in
    --operator)   OPERATOR_MODE=true ;;
    --check)      CHECK_ONLY=true ;;
    --skip-setup) SKIP_SETUP=true ;;
    --shallow)    SHALLOW=true ;;
    -h|--help)    usage; exit 0 ;;
    *) echo "[bootstrap] 不明な引数: $arg" >&2; usage; exit 2 ;;
  esac
done

WARN_COUNT=0
SUMMARY=()
info() { printf '[bootstrap] %s\n' "$*"; }
ok()   { printf '[bootstrap] OK: %s\n' "$*"; }
warn() { printf '[bootstrap] WARN: %s\n' "$*" >&2; WARN_COUNT=$((WARN_COUNT + 1)); }
section() { printf '\n[bootstrap] ---- %s ----\n' "$*"; }

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ "$SHALLOW" = true ]; then
  warn "--shallow はテスト専用。本番機では付けずに再実行し、フル履歴で clone すること(shallow だと sync_status.sh の behind/ahead 判定が狂う)"
fi
if [ "$PROJECTS_DIR" != "$HOME/projects" ]; then
  warn "PROJECTS_DIR が既定(\$HOME/projects)と違う ($PROJECTS_DIR)。run_teleop_*.sh / run_leader_agent_*.sh 等の実行スクリプト群は \$HOME/projects を決め打ちしているため、この構成のままではテレオペ・収集そのものは動かない(テスト専用の上書き)"
fi
if [ "$REPO" != "$PROJECTS_DIR/openarm-orboh" ]; then
  warn "このスクリプトの実体位置 ($REPO) が $PROJECTS_DIR/openarm-orboh と一致しない。実行スクリプト群のハードコードパス契約が満たされない可能性がある"
fi

# ---- 0. 前提コマンド -----------------------------------------------------------
section "0/6 前提コマンド確認"
command -v git >/dev/null 2>&1 || warn "git が無い。導入してから再実行: sudo apt install -y git"
command -v python3 >/dev/null 2>&1 || warn "python3 が無い。導入してから再実行: sudo apt install -y python3 python3-venv"
HAVE_UV=false
if command -v uv >/dev/null 2>&1; then
  HAVE_UV=true
  ok "uv 検出 -> lerobot venv 構築に使う"
else
  info "uv 未検出 -> python3 -m venv にフォールバック"
fi

# ---- 1. openarm-orboh 自身 ------------------------------------------------------
section "1/6 openarm-orboh(このリポ)"
SELF_BRANCH="$(git -C "$REPO" branch --show-current 2>/dev/null || echo '?')"
if [ "$SELF_BRANCH" != "$OPENARM_BRANCH" ]; then
  warn "openarm-orboh: 現在ブランチ '$SELF_BRANCH' が想定 '$OPENARM_BRANCH' と違う。checkout はしない(手で: git -C \"$REPO\" checkout $OPENARM_BRANCH)"
else
  ok "openarm-orboh: branch=$SELF_BRANCH"
fi
if [ "$CHECK_ONLY" = true ]; then
  info "openarm-orboh: --check につき submodule 更新はしない"
else
  if git -C "$REPO" submodule update --init --recursive; then
    ok "openarm-orboh: submodule(openarm_teleop/openarm_description) OK"
  else
    warn "openarm-orboh: submodule 取得に失敗した"
  fi
fi
if [ -n "$(git -C "$REPO" status --porcelain 2>/dev/null)" ]; then
  warn "openarm-orboh: 未コミットの変更がある(触らない)"
fi
SUMMARY+=("openarm-orboh: path=$REPO branch=$SELF_BRANCH(想定 $OPENARM_BRANCH)")

# ---- 兄弟リポ共通処理 -----------------------------------------------------------
# 既存リポには fetch + 状態報告のみ。checkout/reset/pull は絶対にしない(喪失ゼロ)。
declare -A REPO_STATUS
ensure_repo() {
  local name="$1" url="$2" branch="$3"
  local dir="$PROJECTS_DIR/$name"
  local tmp="$PROJECTS_DIR/${name}.partial"

  if [ ! -d "$dir" ]; then
    if [ -d "$tmp" ]; then
      warn "$name: 前回の clone が中断した残骸がある ($tmp)。削除してから再実行: rm -rf \"$tmp\""
      REPO_STATUS[$name]=WARN
      return
    fi
    if [ "$CHECK_ONLY" = true ]; then
      info "$name: 見つからない ($dir)。--check につき clone しない"
      REPO_STATUS[$name]=WARN
      return
    fi
    info "$name: clone する ($url @ $branch)"
    local clone_args=(git clone -b "$branch" "$url" "$tmp")
    [ "$SHALLOW" = true ] && clone_args=(git clone --depth 1 -b "$branch" "$url" "$tmp")
    if "${clone_args[@]}"; then
      mv "$tmp" "$dir"
      ok "$name: clone 完了 -> $dir"
      REPO_STATUS[$name]=OK
    else
      warn "$name: clone に失敗した(ネットワーク断・ブランチ不在など)。$tmp を rm -rf してから再実行する"
      rm -rf "$tmp" 2>/dev/null || true
      REPO_STATUS[$name]=WARN
    fi
    return
  fi

  if [ ! -d "$dir/.git" ]; then
    warn "$name: $dir はあるが git リポジトリではない。手で確認すること"
    REPO_STATUS[$name]=WARN
    return
  fi

  local origin
  origin="$(git -C "$dir" remote get-url origin 2>/dev/null || echo '')"
  if [ "$origin" != "$url" ]; then
    warn "$name: origin が想定と違う ($origin != $url)。upstream 直 clone の可能性(push 事故のもと)。直すなら: git -C \"$dir\" remote set-url origin \"$url\""
  fi

  if [ "$CHECK_ONLY" = true ]; then
    info "$name: --check につき fetch はしない(前回 fetch 時点の情報)"
  else
    if ! timeout 20 git -C "$dir" fetch origin "$branch" -q 2>/dev/null; then
      warn "$name: fetch 失敗(ネットワーク断、またはブランチ '$branch' が remote に無い→設定ブロックを確認)"
    fi
  fi

  local dirty
  dirty="$(git -C "$dir" status --porcelain 2>/dev/null)"
  if [ -n "$dirty" ]; then
    warn "$name: 未コミットの変更がある。触らない(手で確認: git -C \"$dir\" status)"
    REPO_STATUS[$name]=WARN
    return
  fi

  local cur head
  cur="$(git -C "$dir" branch --show-current 2>/dev/null)"
  head="$(git -C "$dir" rev-parse --short HEAD 2>/dev/null)"
  if [ "$cur" != "$branch" ]; then
    warn "$name: 現在ブランチ '$cur' が想定 '$branch' と違う。checkout はしない(手で: git -C \"$dir\" checkout $branch)"
    REPO_STATUS[$name]=WARN
    return
  fi

  local behind ahead
  read -r behind ahead <<<"$(git -C "$dir" rev-list --left-right --count "origin/$branch...HEAD" 2>/dev/null || echo "0 0")"
  if [ "${behind:-0}" != "0" ] || [ "${ahead:-0}" != "0" ]; then
    warn "$name: origin/$branch と ${behind:-0}件遅れ/${ahead:-0}件進み。pull/push は手で(例: git -C \"$dir\" pull --ff-only)"
    REPO_STATUS[$name]=WARN
  else
    ok "$name: $branch ($head) 同期済み"
    REPO_STATUS[$name]=OK
  fi
}

# ---- 2. lerobot(両モード必須) --------------------------------------------------
section "2/6 lerobot(fork)"
ensure_repo lerobot "$LEROBOT_URL" "$LEROBOT_BRANCH"
LEROBOT_DIR="$PROJECTS_DIR/lerobot"
SUMMARY+=("lerobot: path=$LEROBOT_DIR branch想定=$LEROBOT_BRANCH -> ${REPO_STATUS[lerobot]:-?}")

# ---- 3. lerobot venv ------------------------------------------------------------
section "3/6 lerobot venv"
LEROBOT_VENV_STATUS="SKIP"
if [ -d "$LEROBOT_DIR" ]; then
  VENV="$LEROBOT_DIR/.venv"
  if [ -d "$VENV" ]; then
    if [ -x "$VENV/bin/python" ] && "$VENV/bin/python" -c 'import lerobot, pinocchio, zmq' >/dev/null 2>&1 \
        && { [ -x "$VENV/bin/lerobot-teleoperate" ] || [ -x "$VENV/bin/lerobot-record" ]; }; then
      ok "lerobot venv: 既存 ($VENV)。import(lerobot/pinocchio/zmq) と lerobot コマンドの実在を確認"
      LEROBOT_VENV_STATUS="OK"
    else
      warn "lerobot venv: 既存 ($VENV) だが検証失敗(import または lerobot-teleoperate/-record が無い)。再構築するなら削除してから再実行: rm -rf \"$VENV\""
      LEROBOT_VENV_STATUS="WARN"
    fi
  elif [ "$CHECK_ONLY" = true ]; then
    info "lerobot venv: --check につき未構築(構築はしない)"
    LEROBOT_VENV_STATUS="WARN"
  elif ! command -v python3 >/dev/null 2>&1; then
    warn "lerobot venv: python3 が無いので構築をスキップ"
    LEROBOT_VENV_STATUS="WARN"
  elif ! python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)'; then
    warn "lerobot venv: python3 が 3.10 未満。venv 構築をスキップ(3.10+ が必要)"
    LEROBOT_VENV_STATUS="WARN"
  else
    info "lerobot venv: 新規構築する ($VENV)"
    pip_install() {
      if [ "$HAVE_UV" = true ]; then
        (cd "$LEROBOT_DIR" && uv pip install --python "$VENV/bin/python" "$@")
      else
        "$VENV/bin/pip" install "$@"
      fi
    }
    if [ "$HAVE_UV" = true ]; then
      (cd "$LEROBOT_DIR" && uv venv "$VENV")
    else
      python3 -m venv "$VENV"
    fi
    if [ -x "$VENV/bin/python" ] && pip_install -e "$LEROBOT_DIR[damiao,intelrealsense]" pin pyzmq; then
      ok "lerobot venv: 構築完了(damiao + intelrealsense + pin + pyzmq)"
      LEROBOT_VENV_STATUS="OK"
    elif [ -x "$VENV/bin/python" ] && pip_install -e "$LEROBOT_DIR[damiao]" pin pyzmq; then
      warn "lerobot venv: intelrealsense(pyrealsense2) 抜きで構築した。このホストでカメラ収録するなら手で追加: (uv or pip) install pyrealsense2"
      LEROBOT_VENV_STATUS="WARN"
    else
      warn "lerobot venv: 構築に失敗した(damiao 抜きでも失敗)。手で確認: cd $LEROBOT_DIR && (uv venv .venv または python3 -m venv .venv)"
      LEROBOT_VENV_STATUS="WARN"
    fi
  fi
else
  info "lerobot venv: lerobot 自体が無いのでスキップ"
  LEROBOT_VENV_STATUS="WARN"
fi
SUMMARY+=("lerobot venv: path=$LEROBOT_DIR/.venv -> $LEROBOT_VENV_STATUS")

# ---- 4. sender(--operator ならスキップ) ------------------------------------------
section "4/6 XRoboToolkit-Orin-Video-Sender"
SENDER_STATUS="SKIP"
if [ "$OPERATOR_MODE" = true ]; then
  info "sender: --operator につき対象外(操作側PCに映像 sender は不要)"
else
  ensure_repo XRoboToolkit-Orin-Video-Sender "$SENDER_URL" "$SENDER_BRANCH"
  SENDER_DIR="$PROJECTS_DIR/XRoboToolkit-Orin-Video-Sender"
  if [ -d "$SENDER_DIR" ]; then
    BIN="$SENDER_DIR/OrinVideoSender"
    if [ -x "$BIN" ]; then
      ok "sender: 既存バイナリを尊重する ($BIN)。make clean はしない"
      SENDER_STATUS="OK"
    elif [ "$CHECK_ONLY" = true ]; then
      info "sender: --check につきビルドしない"
      SENDER_STATUS="WARN"
    else
      MISSING=()
      for pc in gstreamer-1.0 gstreamer-app-1.0 gstreamer-plugins-base-1.0 libzmq; do
        pkg-config --exists "$pc" 2>/dev/null || MISSING+=("$pc")
      done
      if [ "${#MISSING[@]}" -gt 0 ]; then
        warn "sender: 依存不足(${MISSING[*]})。ビルドしない。導入コマンド(推測・要確認):"
        warn "  sudo apt install -y build-essential pkg-config git libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-tools libzmq3-dev v4l-utils"
        SENDER_STATUS="WARN"
      elif (cd "$SENDER_DIR" && make); then
        ok "sender: make 完了 -> $BIN"
        SENDER_STATUS="OK"
      else
        warn "sender: make に失敗した。$SENDER_DIR で手動確認すること"
        SENDER_STATUS="WARN"
      fi
    fi
  else
    warn "sender: リポジトリが無いのでビルドをスキップ"
    SENDER_STATUS="WARN"
  fi
  SUMMARY+=("sender: path=${SENDER_DIR:-$PROJECTS_DIR/XRoboToolkit-Orin-Video-Sender} branch想定=$SENDER_BRANCH repo=${REPO_STATUS[XRoboToolkit-Orin-Video-Sender]:-?} build=$SENDER_STATUS")
fi

# ---- 5. scripts/setup.sh(apt/openarm_teleopビルド/udev。sudo要) -------------------
section "5/6 scripts/setup.sh"
if [ "$CHECK_ONLY" = true ]; then
  info "setup.sh: --check につきスキップ"
elif [ "$SKIP_SETUP" = true ]; then
  info "setup.sh: --skip-setup につきスキップ(必要になったら単体で: bash scripts/setup.sh)"
else
  info "setup.sh を実行する(apt 依存導入・openarm_teleop ビルド・udev ルール導入。sudo のパスワード入力が複数回出る)"
  if bash "$REPO/scripts/setup.sh"; then
    ok "setup.sh 完了"
  else
    warn "setup.sh が失敗した。個別に再実行できる: bash scripts/setup.sh"
  fi
fi

# ---- 6. 検証サマリ ---------------------------------------------------------------
section "6/6 検証サマリ"
for line in "${SUMMARY[@]}"; do
  echo "  $line"
done
echo
if [ "$WARN_COUNT" -gt 0 ]; then
  echo "[bootstrap] WARN ${WARN_COUNT} 件。上の WARN 行を読み、案内されたコマンドで手で直す(このスクリプトは checkout/reset/pull/削除/apt を自動実行しない)。" >&2
else
  echo "[bootstrap] 全緑。次は下の『次にやること』チェックリストへ。"
fi

cat <<'EOF'

[bootstrap] ---- 次にやること(このスクリプトが自動化しない人手作業) ----
  1. PEAK アダプタを PC に直挿し(USBハブ経由は電流/認識問題でNG)。
     新品(役割タグ未設定)アダプタだけ焼く: sudo bash scripts/burn_channel_id.sh <iface> leader|follower
  2. CAN を固定名にして健全性を確認: sudo bash scripts/canup.sh
  3. 較正(どちらか):
     - 既存機がある: leader_*.json を scp でコピー
       scp thor:~/.cache/huggingface/lerobot/calibration/teleoperators/openarm_leader/leader_right.json \
           thor:~/.cache/huggingface/lerobot/calibration/teleoperators/openarm_leader/leader_left.json \
           ~/.cache/huggingface/lerobot/calibration/teleoperators/openarm_leader/
     - 新規機体: bash scripts/calibrate_openarm.sh <left|right>
  4. Hugging Face: 保存先リポジトリを Web UI で作成(Owner=Orboh)し、トークンにそのリポの
     write 権限を付与してから小さなテスト push で確認する(新規リポ名だと403になる罠がある)。
  5. PICO: 開発者モード有効化・Safeguard オフ・XRoboToolkit APK 導入、アプリで
     Remote Vision -> Listen -> ThorのIP:13579 -> Confirm。
  6. ネットワーク: 操作側PCとThorを同一5GHz Wi-Fi/LANに接続し、
     sudo ufw allow 13579/tcp(必要なら)。可能なら DHCP 予約でIP固定。
  7. カメラ製造番号(D435i/D405のシリアル)がシーンプロファイル(scenes/*.yaml)の
     値と一致するか目視確認。
  8. 安全ゲート: フォロワー周囲の可動域クリア・非常停止(電源スイッチ)への到達確認・
     グリッパーに異物が挟まっていないことを確認。
EOF

[ "$WARN_COUNT" -gt 0 ] && exit 1
exit 0
