<!-- 2026-09-21 起草（Vector・Claude workflow）。実機未検証の手順書。実行して食い違いが出たら本文を直す。正本の設計は Vault 02VLA/gr00t-openarm/gr00t-openarm-pipeline-design.md。 -->

# 両腕 OpenArm × GR00T 手順書（収集 → v3→v2 変換 → EC2 FT → Thor rollout）

起案日 2026-09-21。対象は Vector で編集した feat/bimanual-collection（openarm-orboh）と feat/openarm-remote-teleop（lerobot fork・BiOpenArmLeader.soft_move_to 追加）の実機投入。両腕経路は全段で実機未検証なので、各段の「初回だけやる検証」を飛ばさない。以下、絶対パスは Thor と EC2 のものも `~` を各マシンの HOME として読む。

---

## 0. 前提の確認（着手前に 5 分）

1. Vector の作業ツリーは未コミット。この手順書は「人が commit と push を行う」ことを前提にする（workflow は commit/push しない）。
2. lerobot fork の remote は `origin` = huggingface 本家、`orboh` = Orboh/lerobot。push 先は必ず `orboh`（`git -C ~/projects/lerobot remote -v` で実測済み）。
3. Thor 側の較正 4 本（`~/.cache/huggingface/lerobot/calibration/robots/openarm_follower/follower_{left,right}.json`・`teleoperators/openarm_leader/leader_{left,right}.json`）は揃っている（左は 2026-09-21 に bump-to-stop で再較正済み、右は 08-04 版）。BiOpenArmFollower/BiOpenArmLeader は `--robot.id=follower` / `--teleop.id=leader` から `follower_left` 等の名前を組み、内側の単腕クラスが `robots/openarm_follower/` 配下を読むので、bi 専用の較正ディレクトリは要らない（lerobot `robots/robot.py` L49-53・`bi_openarm_follower.py` L59-61）。
4. 左右の開始姿勢 YAML は Thor 上で「右 9/5 キャプチャ（J1 -69.38 / J7 80.75）とその鏡像の左」の対になっている。Vector の `config/initial_pose_right.yaml` は旧キャプチャ（J1 -19.89）で対になっていないので、Vector で `record_bimanual_openarm.sh` を起動すると鏡像検算（`POSE_MIRROR_TOL_DEG` 既定 2°）で止まる。これは Thor では起きない（Thor の右 YAML は 9/5 版が未コミットで置かれている）。

根拠: devlog 2026-09-21 追記（`vla-track-devlog.md` 進行ログ 2026-09-18〜19 末尾）、`scripts/record_bimanual_openarm.sh` の鏡像検算ブロック、pipeline-design §1.0「clone 以外に要るもの」。

---

## 1. Thor への配置手順（Kota の未コミット変更を上書きしない）

### 1.1 Vector 側（人が行う）

1. openarm-orboh をコミットする。対象は次の 11 ファイル（`git -C ~/projects/openarm-orboh status --short` の実測。`.claude/` は含めない）:
   `config/initial_pose_left.yaml`、`scripts/collect_manifest.py`、`scripts/record_bimanual_openarm.sh`（新規）、`gr00t/README.md`、`gr00t/openarm_embodiment/openarm_layout.py`（新規）、`gr00t/openarm_embodiment/openarm_cameras.py`、`gr00t/openarm_embodiment/openarm_config.py`、`gr00t/openarm_embodiment/gen_modality_json.py`、`gr00t/openarm_embodiment/check_dataloader.py`、`scripts/g4/eval_openarm.py`、`scripts/g4/run_rollout.sh`。
   `gr00t/openarm_embodiment/modality.json` は Vector では未変更なので含まれない（含まれていたら止まる。Thor 側の同名ファイルは Kota の wrist 版で未コミット）。
2. `git -C ~/projects/openarm-orboh push origin feat/bimanual-collection`。
3. lerobot をコミットする。対象は `src/lerobot/teleoperators/bi_openarm_leader/bi_openarm_leader.py` の 1 本だけ（`git diff --stat` 実測 34 行追加）。
4. `git -C ~/projects/lerobot push orboh feat/openarm-remote-teleop`。`origin` には絶対に push しない。誤爆防止として `git -C ~/projects/lerobot config remote.origin.pushurl no_push` を先に入れておくと安全（既存の pre-push secret-scan とは別の保険）。
5. PR はどちらも Kota マージ。PR は open で残す（自分でマージしない）。

### 1.2 Thor 側（`ssh thor`・状態を変える操作なので 1 つずつ確認しながら）

6. まず Thor の汚れを実測する。`git -C ~/projects/openarm-orboh status --short` と `git -C ~/projects/lerobot status --short` を出し、Kota の未コミット（既知: `gr00t/openarm_embodiment/modality.json`、`config/initial_pose_right.yaml`、lerobot の `src/lerobot/scripts/lerobot_record.py`、ほかに `scenes/tape-desk-v1.yaml` があり得る）を控える。**これらは今回の配置対象と重ならない**ことが上書きしない根拠になる（1.1 の 12 ファイルと突き合わせる）。
7. 万一、配置対象と Thor の汚れが重なるファイルがあれば（例: Kota が `openarm_config.py` に wrist 変更を残している場合）、checkout の前に退避する: `cp <file> <file>.thor-bak-20260921` を取り、配置後に `diff <file>.thor-bak-20260921 <file>` で Kota の意図を確認して手で統合する。退避なしに `git checkout -- <file>` すると Kota の変更が消える。
8. openarm-orboh は「ファイル単位の取り込み」にする（Thor のブランチは変えない。Thor は main @8005d1a 系のまま）:
   `git -C ~/projects/openarm-orboh fetch origin feat/bimanual-collection`
   `git -C ~/projects/openarm-orboh checkout origin/feat/bimanual-collection -- config/initial_pose_left.yaml scripts/collect_manifest.py scripts/record_bimanual_openarm.sh gr00t/README.md gr00t/openarm_embodiment/openarm_layout.py gr00t/openarm_embodiment/openarm_cameras.py gr00t/openarm_embodiment/openarm_config.py gr00t/openarm_embodiment/gen_modality_json.py gr00t/openarm_embodiment/check_dataloader.py scripts/g4/eval_openarm.py scripts/g4/run_rollout.sh`
   `config/initial_pose_left.yaml` は 09-21 に Thor でも同内容に更新済みなので実質差分ゼロのはず。`git diff --stat` で確認する。
9. Thor の Isaac-GR00T-n1.7 側は symlink（2026-09-05 に `install_gr00t_embodiment.sh` で `~/projects/Isaac-GR00T-n1.7/openarm_embodiment` を repo の `gr00t/openarm_embodiment` へ向けた）なので、8 の checkout だけで GR00T 側にも新ファイルが見える。`ls -l ~/projects/Isaac-GR00T-n1.7/openarm_embodiment` が symlink であることを確認する。symlink でなく実体ディレクトリなら `GR00T_ROOT=~/projects/Isaac-GR00T-n1.7 bash scripts/install_gr00t_embodiment.sh` を走らせ、`[SKIP]` が出たら `diff -r` で中身を見てから `FORCE=1`（既存実体は `.bak-<UTC>` に退避される）。
10. lerobot も同じくファイル単位で取り込む:
    `git -C ~/projects/lerobot fetch orboh feat/openarm-remote-teleop`
    `git -C ~/projects/lerobot checkout orboh/feat/openarm-remote-teleop -- src/lerobot/teleoperators/bi_openarm_leader/bi_openarm_leader.py`
    Thor の `lerobot_record.py` の汚れはこのコマンドに触れられない。venv は editable install なので再インストール不要。Thor の HEAD が Vector と同じ `927b387b` なら代替として `git merge --ff-only orboh/feat/openarm-remote-teleop` でもよい（汚れているファイルをマージが触らないので通る）。
11. 配置後の自己検証（実機不要）:
    `bash -n ~/projects/openarm-orboh/scripts/record_bimanual_openarm.sh`
    `~/projects/lerobot/.venv/bin/python -m py_compile ~/projects/openarm-orboh/scripts/g4/eval_openarm.py ~/projects/lerobot/src/lerobot/teleoperators/bi_openarm_leader/bi_openarm_leader.py`
    `~/projects/lerobot/.venv/bin/python -c "from lerobot.teleoperators.bi_openarm_leader import BiOpenArmLeader; print(hasattr(BiOpenArmLeader,'soft_move_to'))"` が True。
    `~/projects/openarm-orboh/scripts/check_repo_sync.sh` は record 起動時に自動で走り同期負債を警告する（`SKIP_SYNC_CHECK=true` で黙らせられるが初回は見る）。

根拠: `git status`/`git remote -v` 実測（Vector・2026-09-21）、pipeline-design §1.0（clone 手順⑤ symlink・Thor と Vector の HEAD）、devlog 現在地「openarm_embodiment/ 設定一式 ✅ 解消」（Thor 側は symlink・旧実体 `.bak-20260905T062703Z`）、`scripts/install_gr00t_embodiment.sh`（SKIP/FORCE 挙動）。

---

## 2. 収集の起動（Thor・`ssh -t thor` の素の端末で。`ssh -X` にしない）

### 2.1 物理と CAN の前提（毎セッション）

1. フォロワー箱（22222222）とリーダー箱（44444444・倉庫箱を 09-21 に焼き直し済み）を両方接続し、`sudo bash ~/projects/openarm-orboh/scripts/canup.sh` で `can_fol_l / can_fol_r / can_lead_l / can_lead_r` の 4 本が UP・ERROR-ACTIVE。
2. `~/projects/lerobot/.venv/bin/python ~/projects/openarm-orboh/scripts/openarm_motorcheck.py` で 4 腕 32 モータ応答。**全行 0.0000 で [OK] が出るのは偽 OK**（未給電・無応答）。`drops [none]` を確認するまで動かさない。
3. カメラ: `rs-enumerate-devices | grep -i serial` で chest D435i `344422070898` と wrist D405 `352122273417`（板橋個体）が見える。Thor の元の USB-C は使わない（ホスト非対応）。
4. シーン YAML: 板橋用を新設する。`cp scenes/_TEMPLATE.yaml scenes/itabashi-desk-v1.yaml`（名前は例。英数ハイフン）を作り、実機の前で TODO を全部埋める。TODO が残ると起動が止まる（dry run だけ `ALLOW_TODO_SCENE=true` で通せるが本番では使わない。Kota の tape は todo_count=40 のまま録られて条件が辿れなくなった）。
5. 板橋の机高さ・かご位置は 09-19 の devlog に「アーム机約 710・昇降表示 89・相対 +180mm」の実測があるので出発点にする。

### 2.2 dry run（2 本・条件を残さない）

```
source ~/projects/lerobot/.venv/bin/activate
cd ~/projects/openarm-orboh
env -u DISPLAY -u WAYLAND_DISPLAY \
  OPERATOR=Kento SCENE=none \
  TASK="Pick up the okra with both arms and place it in the bin" \
  TASK_SLUG=okra \
  REPO_ID=Orboh/bi_openarm_okra_chest_wristr_test_$(date +%s) \
  NUM_EPISODES=2 EPISODE_TIME_S=30 DISPLAY_DATA=false \
  bash scripts/record_bimanual_openarm.sh bi-okra-dry
```

見るもの:
6. 起動バナー `[record-bi] follower=L:can_fol_l R:can_fol_r leader=L:can_lead_l R:can_lead_r` と `cameras=... chest ... wrist_right ...`、`obs=chest_wristr`。
7. 起動ログに `No display detected: reading recording control keys from stdin` が出る（出なければ DISPLAY が生えている）。
8. 接続時整列で両腕が自走する（各腕約 2.2 秒）。手を離しておく。
9. 1 本目終了後に `a` を押して、**leader 左→右（各 2.0 秒）→ follower 両腕（1.5 秒）の順に開始姿勢へ戻る**ことを目視する。これが unit B（`BiOpenArmLeader.soft_move_to`）の初回実機検証。戻し後の検算 3° を超えると console.log と `start-pose-bimanual.jsonl` に出る。
10. 初回だけ `開始姿勢 '.../initial_pose_left.yaml' が前回の収録時と変わっている` で止まりうる（09-21 に左 YAML を鏡像で作り直したため過去 manifest の sha と割れる）。理由がそれだと確認できたら `ALLOW_POSE_DRIFT=true` を 1 回だけ足す。右 YAML は 9/5 版で Kota の run と同じ sha なので出ないはず。出たら原因を確認してから。
11. 終了後に `~/collections/<UTC>-bi-okra-dry/` に `console.log / manifest.json / command.sh / start-pose-bimanual.jsonl` があり、`jq '.control_variables.side, .control_variables.cameras, .calibration | length' manifest.json` が `bimanual` / `chest, wrist_right` / `4` を返す。
12. データ側: `python - <<'EOF'` で `meta/info.json` の `features["observation.state"]["names"]` が契約 16 キー（`right_joint_1.pos` … `right_gripper.pos` … `left_joint_1.pos` … `left_gripper.pos`）の順、`features["action"]["names"]` も同一、`observation.images.chest` と `observation.images.wrist_right` が 480x640x3、fps 30 であることを確認する。同じ検査を `python gr00t/openarm_embodiment/gen_modality_json.py ~/.cache/huggingface/lerobot/Orboh/<stamped_repo> --dry-run` に `OPENARM_EMBODIMENT=bimanual` を付けて走らせても代替できる（gr00t 非依存・v3 の info.json でも読める。判定と env が食い違えば `[FAIL]`）。
13. test データセットは `~/.cache/huggingface/lerobot/Orboh/` から消してよい（捨て名運用）。

### 2.3 受入ゲート（本番 10 本を 1 セッション録って止める）

```
env -u DISPLAY -u WAYLAND_DISPLAY \
  OPERATOR=Kento SCENE=itabashi-desk-v1 \
  TASK="<2.2 と同じ文字列>" TASK_SLUG=okra \
  NUM_EPISODES=10 EPISODE_TIME_S=30 DISPLAY_DATA=false \
  bash scripts/record_bimanual_openarm.sh bi-okra-v1-s1
```

`REPO_ID` は省略すると `Orboh/bi_openarm_okra_chest_wristr_v1` が組まれ、lerobot が `_YYYYMMDD_HHMMSS` を付けたディレクトリを作る（manifest resolve が実 repo_id を追記する）。セッションごとに別スタンプになるので最後に merge する（2.4）。

受入で見る項目（データ管理標準 §5-4 の「ロード・可視化・時刻整合・FOV・台帳」を両腕向けに具体化）:
14. **ロードと次元**: 2.2 の 12 と同じ検査を 10 本のデータセットに対して。`total_episodes` 10・`total_tasks` 1・TASK 文字列が一字一句同じ。
15. **length と動画区間の整合**（08-06 に 3 本失った罠）: 各 ep について `(video to - from) * 30 == length` を `meta/episodes/` から検算する。不一致の ep はデータと動画が別テイクを指しているので、混ぜる前に `lerobot-edit-dataset --operation.type delete_episodes` で落とす。運用側の予防は「20 秒（今回は 30 秒）の打ち切りに達させない・達したら ← で捨てず保存する」。
16. **FOV と画質**（両カメラ）: `ffmpeg -i videos/observation.images.wrist_right/chunk-000/file-000.mp4 -frames:v 1 /tmp/wr.png` と chest で 1 枚ずつ抜き、wrist_right に把持対象とグリッパが映っているか、chest に両腕とかごが入るか目視する。Thor には AV1 が読める ffmpeg がある（変換 env が残存）。
17. **開始姿勢**: `start-pose-bimanual.jsonl` の逸脱が 3° 以内で、左右とも記録されている（左だけ・右だけになっていないこと）。
18. **両腕の値が動いている**: `data/chunk-000/file-000.parquet` を読み、`observation.state` の 0:8 と 8:16 の両方に分散があること（片腕が CAN 落ちで固定値になっていると学習が壊れる。09-19 の遠位 4 モータ片道通信の事例）。
19. **台帳**: ep 番号・成否・尺・異常を `manifest.json` の human_fields か `notes.md` に記入（位置統制するならセル列）。判定者は Kento。
20. **Vault 回収**: Vector から `bash ~/projects/openarm-orboh/scripts/pull_run.sh <UTC>-bi-okra-v1-s1`（回収先は `02VLA/gr00t-openarm/collections/`）。
21. ゲート PASS 後に s2〜s5 を 10 本ずつ録る（タグを `-s2` … に変える）。ALLOW 系フラグは付けない（付けたくなる警告が出たら止まって原因を見る）。

### 2.4 統合（収録セッション中には走らせない・Thor 上）

22. 失敗 ep を各セッションから `delete_episodes` で落としてから、`lerobot-edit-dataset --operation.type merge --operation.repo_ids "[Orboh/bi_openarm_okra_chest_wristr_v1_<s1>,...,<s5>]" --new_repo_id Orboh/bi_openarm_okra_chest_wristr_v1`。merge は `--repo_id` を取らず `--new_repo_id` 必須。出力は `images/` を持たない。
23. 統合後にもう一度 14・15 の検査（50 本・16 次元・length 整合）。

根拠: `scripts/record_bimanual_openarm.sh`（env 名・既定値・ゲート・バナー文言・stamp 解決は同ファイルと `scripts/collect_manifest.py` `_find_stamped`）、pipeline-design §2.5 gotcha 2〜4（`env -u DISPLAY`・stdin 直読み・貼り付け事故）と §2.7（確定コマンド型・`EPISODE_ADVANCE=manual`・検算 3°）、`2026-08-06-bottle-v2-s1-handoff.md` §7.10〜7.12（length/動画整合・merge の作法・`NUM_EPISODES=1` を使わない）、data-management-standard §4（捨て名 `_test_<unixtime>`）§5-4/5-5、devlog 2026-09-18〜19（板橋の机高さ・USB-C・偽 OK・drops）。

---

## 3. HF push（Thor から・Orboh org・private）

1. **人手の 2 操作**（トークンは org に対して read しか持たないので、マシンを変えても回避できない）:
   a. https://huggingface.co/new-dataset で Owner を **Orboh** に切り替え、`bi_openarm_okra_chest_wristr_v1` を private で作る（IME を半角にして手打ち）。
   b. https://huggingface.co/settings/tokens で Thor のトークン `thor-lerobot-uproad` の Repository permissions にその dataset を追加し read + write。トークン値は変わらないので Thor 側のファイルは触らない。
2. 途中 push（約 25 ep ＝ s1・s2 が揃った時点で 1 回）: 統合前でも s1 と s2 を merge した中間データセットを同じ repo に push してよい。`push_to_hub` は既存 repo に重ね書きするので、**push 前に HF 側の中身を実測**する:
   ```
   python ~/projects/openarm-orboh/scripts/guard_push_target.py --repo-id Orboh/bi_openarm_okra_chest_wristr_v1 --allow-existing
   python -c "
   import os
   from lerobot.datasets.lerobot_dataset import LeRobotDataset
   r=os.path.expanduser('~/.cache/huggingface/lerobot/Orboh/bi_openarm_okra_chest_wristr_v1')
   ds=LeRobotDataset('Orboh/bi_openarm_okra_chest_wristr_v1', root=r)
   ds.push_to_hub(private=True)
   "
   ```
   `guard_push_target.py` は「push 先が既に存在する」で止まるのが既定。器を人手で作った直後は `.gitattributes` だけなので `--allow-existing` で通す。2 回目以降（途中 push の重ね書き）は旧 chunk の orphan 化を承知の上で通す。
3. 完了後（50 本統合版）: 同じコマンドで push。`push_to_hub` が最後に `v3.0` タグを delete → create するので、タグの貼り直しは不要。手で打つなら `hf` CLI で `v3.0` を全量アップロード後に 1 回だけ（途中 push ごとには打たない）。
4. HF 側で検証: `python -c "from huggingface_hub import HfApi; print(HfApi().dataset_info('Orboh/bi_openarm_okra_chest_wristr_v1', revision='v3.0').sha)"` が通り、`meta/info.json`（revision v3.0）の `total_episodes` 50・`observation.state` が `[16] float32`・video 2 本。
5. EC2 の `ec2-lerobot-read` は org `Orboh` 全体に `repo.content.read` を持つので、新 dataset の read に追加作業は要らない（08-06 に実測で決着）。

根拠: `2026-08-06-bottle-v2-s1-handoff.md` §7.13（2 手作業・guard・push コマンド・タグ）、pipeline-design §2.6・§4.6〜4.7（トークンのスコープ実測・whoami-v2 の 1 行）、data-management-standard §5-2/5-3/5-7。

---

## 4. v3→v2 変換と両腕 modality.json（EC2 上で m5.xlarge のまま。Thor ローカルでは行わない）

1. EC2 を起動して SSM Online を待つ（駆動は `send-command` の base64 包み。対話 `start-session` は Claude Code から張れない）。準備は全部 m5.xlarge（$0.192/h）で済ませ、GPU 型へ上げるのは §5 の本走直前だけ。
2. 新 embodiment を EC2 に届ける。EC2 には openarm-orboh の clone が無く、`/home/ubuntu/Isaac-GR00T-n17/openarm_embodiment/` は 7 月の実体ディレクトリ（単腕 chest 版）。Vector で `tar czf - --exclude=__pycache__ -C ~/projects/openarm-orboh/gr00t openarm_embodiment | base64 | tr -d '\n'`（実測 9.6KB）を作り、send-command のスクリプトに埋めて EC2 側で
   `cd /home/ubuntu/Isaac-GR00T-n17 && mv openarm_embodiment openarm_embodiment.bak-20260921 && echo <B64> | base64 -d | tar xzf - && chown -R ubuntu:ubuntu openarm_embodiment && ls -l openarm_embodiment`
   とする。旧ディレクトリは消さず退避（chest 単眼モデルの再現に要る `modality.json` スナップショットが入っている）。
3. 変換 env を再構築する（08-06 に 6.2GB 回収済みなので無い前提）:
   `cd /home/ubuntu/Isaac-GR00T-n17/scripts/lerobot_conversion && uv venv --python 3.10 && GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .`
4. 変換（HF から直接 pull・`--root` 省略で `HF_LEROBOT_HOME` 既定）:
   `export HF_HOME=/home/ubuntu/.cache/huggingface; export HF_TOKEN="$(cat /home/ubuntu/.g2-hf-token)"`（`set -x` より前に置く）
   `.venv/bin/python convert_v3_to_v2.py --repo-id Orboh/bi_openarm_okra_chest_wristr_v1`
   出力は `/home/ubuntu/.cache/huggingface/lerobot/Orboh/bi_openarm_okra_chest_wristr_v1/`（v2.1・元 v3.0 は同階層 `_v3.0` に退避）。`CONVERT_EXIT=0` をログに出す。
5. AV1 対策: 変換は `ffmpeg -c copy` で切り出すだけなので AV1 のまま。EC2 の `apt ffmpeg 4.4.2` は `libdav1d` 同梱で読める。`ffmpeg -decoders | grep av1` に `libdav1d` が出ることを毎回確認（機体が復元される前提なので）。
6. modality.json を生成する（手コピーしない・gr00t 非依存なのでどの python でも可）:
   `cd /home/ubuntu/Isaac-GR00T-n17 && OPENARM_EMBODIMENT=bimanual .venv/bin/python openarm_embodiment/gen_modality_json.py /home/ubuntu/.cache/huggingface/lerobot/Orboh/bi_openarm_okra_chest_wristr_v1`
   `cat meta/modality.json` が契約どおり（state/action とも right_arm 0:7 / right_gripper 7:8 / left_arm 8:15 / left_gripper 15:16、video chest と wrist_right、annotation task_index）であること。env を付けているのは判定結果との不一致を `[FAIL]` にするため。
7. norm stats（GPU 不要・`generate_rel_stats` が modality.json を実ロードするので疎通確認を兼ねる）:
   `OPENARM_EMBODIMENT=bimanual OPENARM_CAMERAS=chest,wrist_right .venv/bin/python gr00t/data/stats.py --dataset-path /home/ubuntu/.cache/huggingface/lerobot/Orboh/bi_openarm_okra_chest_wristr_v1 --embodiment-tag NEW_EMBODIMENT --modality-config-path openarm_embodiment/openarm_config.py`
   新規データセットなので既存 `meta/stats.json` は無いはずだが、あれば削除してから（既存があると再計算をスキップする）。`relative_stats.json` に `right_arm` と `left_arm` の 2 キーが出る。
8. dataloader 疎通:
   `OPENARM_EMBODIMENT=bimanual OPENARM_CAMERAS=chest,wrist_right .venv/bin/python openarm_embodiment/check_dataloader.py /home/ubuntu/.cache/huggingface/lerobot/Orboh/bi_openarm_okra_chest_wristr_v1`
   期待: `[0b]` embodiment 一致、`states.right_arm (1,7)` / `right_gripper (1,1)` / `left_arm (1,7)` / `left_gripper (1,1)`、`actions.*` が (16,7)/(16,1)、`images.chest` と `images.wrist_right` が (480,640,3)。
9. 変換 env と uv キャッシュを回収して空きを作る（`uv cache clean`・`rm -rf scripts/lerobot_conversion/.venv`）。`df -h /` で **40GiB 以上**の空きを確認してから §5 へ。

根拠: pipeline-design §3.1（`convert_v3_to_v2.py` の引数・in-place・やらないこと）、§3.5 警告（AV1・ffmpeg）、§4.3（EC2 上で直接 pull）、§4.5（LFS wheel・トークン分離・ディスク運用）、§4.7 実行順序 2〜6 と stats コマンド、`gr00t/README.md`「両腕（bimanual）データを回す」、`gr00t/openarm_embodiment/gen_modality_json.py`・`check_dataloader.py` の usage、`ec2-g2-driver-scripts.md`（send-command 作法・preflight）。

---

## 5. EC2 FT（`i-0f47cdfb06939170e`・us-east-1a）

0. **bridge 確認を 1 行**: 触る前に `machine-bridge.md` の Inbox に Mac が EC2 を使っている記載が無いことを見て、`Vector → Mac` に `[未読] 2026-09-XX — EC2 i-0f47cdfb06939170e を Vector が FT で占有中・終了まで型変更/stop をしない → 本手順書` を 1 行書く（2 台から同時に駆動すると学習中の stop で成果を失う）。
1. 型変更（同 AZ・同ボリュームなので stop → modify → start）:
   `aws ec2 stop-instances --instance-ids $IID`（stopped を待つ）
   `aws ec2 modify-instance-attribute --instance-id $IID --instance-type g6e.4xlarge`
   `aws ec2 start-instances --instance-ids $IID`
   `InsufficientInstanceCapacity` なら g6e.4xlarge / 2xlarge / 8xlarge / 12xlarge を 1 周して 20 秒待って 2 周目（08-06・08-24 とも 2 周目で確保）。1 周目で諦めない。
2. 起動後の preflight（root の send-command）: `systemctl is-active gpu-idle-autostop.timer`（アイドル自動停止）・`nvidia-smi`（L40S 46GB）・`df -h /`（40GiB 以上）・`ls -l /home/ubuntu/Isaac-GR00T-n17/openarm_embodiment/`（§4-2 の新版）・`ffmpeg -decoders | grep av1`。
3. `train-launch.sh` を差し替えて起動する。差分は `DS`・`OUT`・`STEPS` と **env 2 つ**だけ:
   ```
   DS=/home/ubuntu/.cache/huggingface/lerobot/Orboh/bi_openarm_okra_chest_wristr_v1
   OUT=/home/ubuntu/g2-out/ft_bi_okra_v1
   STEPS=10000
   ```
   生成する `g2-ft.sh` の `export HF_TOKEN=...` の直後・`set -x` より前に
   ```
   export OPENARM_EMBODIMENT=bimanual
   export OPENARM_CAMERAS=chest,wrist_right
   ```
   を足す。`launch_finetune.py` は `--modality_config_path openarm_embodiment/openarm_config.py` を import して config を組むので、env は同じプロセスに乗っていなければ単腕 8 次元として登録され、dataloader が 16 次元と食い違って落ちる（check_dataloader はこの付け忘れを止めるが、FT 起動そのものは別プロセス）。
   学習引数（08-06 実走と同じ）: `--base_model_path nvidia/GR00T-N1.7-3B --dataset_path $DS --embodiment_tag NEW_EMBODIMENT --modality_config_path openarm_embodiment/openarm_config.py --num_gpus 1 --output_dir $OUT --save_steps $STEPS --save_total_limit 1 --max_steps $STEPS --warmup_ratio 0.05 --weight_decay 1e-5 --learning_rate 1e-4 --global_batch_size 32 --color_jitter_params brightness 0.3 contrast 0.4 saturation 0.5 hue 0.08 --dataloader_num_workers 4`。
   ディスク watchdog（空き 15GiB で kill・VRAM ピーク記録）は同スクリプトが同時起動する。
4. step とバッチの目安: 50ep 級は `max_steps=10000`・`global_batch_size=32` を維持（bottle 50ep の実績と同条件で比較できる）。両腕 16 次元は state/action 側の計算量にほぼ効かない一方、動画が 2 本になるので dataloader 側が重くなる。`dataloader_num_workers 4` で 1.7 step/s を下回るようなら 8 に上げる（g6e.4xlarge は 16 vCPU）。VRAM は bottle 実測 40.0/46.1GB で余裕 6GB。2 カメラで VRAM が増えるのは画像トークン分だけなので OOM の可能性は低いが、watchdog の `peak=` を必ず読む。最小の配管検証は `--max_steps 5` でなく §4-7/8 の stats と check_dataloader で済ませる（`max_steps` が小さくても終了時 ckpt 24GB を必ず書く）。
5. 概算コスト: g6e.4xlarge $3.0042/h。bottle 50ep が 1h37（1.72 step/s）だったので、2 カメラで 1.3〜1.6 倍を見込み **2〜2.6 時間、$6〜8**。m5.xlarge での準備（変換・stats・疎通）2〜3 時間で $0.5 前後。合計 10 ドル未満。$30k クレジット（残 $27.9k）は **2026-09-30 失効**で以後は $100k 枠（2028-03 失効）が自動充当されるため、9 月中に回すことに実費上の意味はあるが、9/30 を過ぎても請求は立たない。
6. 走行中の監視: `progress.sh` 相当（`tail /home/ubuntu/g2-logs/ft.log`・`watchdog.log`）で loss と `avail=`・`peak=` を追う。終了判定は `TRAIN_EXIT=0` と `DONE_MARKER`。
7. モデルの HF 配布（成果を EBS に置き去りにしない・共有機体はまた消される前提）:
   a. 人手 2 操作: hf.co で `Orboh/gr00t-bi-openarm-okra-chest-wristr-v1` を **model / private** で作成（Owner を Orboh に）、`ec2-lerobot-read` の Repository permissions に **model 側**を追加して Write。
   b. EC2 で `cp -al` によるハードリンクのステージを作り（`config.json`・`model.safetensors.index.json`・`training_args.bin`・`model-0000*.safetensors`・`experiment_cfg/`・`processor/` の 14 ファイル）、`hf upload Orboh/gr00t-bi-openarm-okra-chest-wristr-v1 $G . --repo-type model`。`--exclude` は 2 回書くと前が捨てられるので使わない。`checkpoint-*` は上げない。
   c. HF 上に safetensors 3 本が揃ったことを実測してから `checkpoint-10000`（24GB）を削除する。
8. ログ回収: `/home/ubuntu/g2-logs/{convert,stats,ft,watchdog}.log` を send-command の stdout 経由で Vector に回収し、`train_runtime`・`train_loss`・step/s・VRAM ピークを devlog に記録する。ログにトークンが平文で残っていないか `grep hf_` で確認する。
9. 停止: 型を戻してから stop（`modify-instance-attribute --instance-type m5.xlarge` は stopped 状態で）。アイドル自動停止が 30 分で止めるが明示的に止める。bridge の行を `[完了]` に更新する。

根拠: pipeline-design §4.1（学習引数）・§4.2（型変更・単価 $3.0042/h・実測 1h37・VRAM 40.0GB・ckpt 24GB）・§4.4（アイドル自動停止）・§4.5（実行順序・必須事項・ディスク）・§4.7（容量不足・stats 先行・watchdog・`hf upload --exclude` の罠・ハードリンク・削除ガード）、`ec2-g2-driver-scripts.md`（2 台同時駆動禁止と bridge・`train-launch.sh`・停止）、`aws-credits.md`（残額・失効日・$100k 自動充当）、`Isaac-GR00T` `launch_finetune.load_modality_config`（parent を sys.path に足して import_module＝env は同プロセス）。

---

## 6. Thor rollout（両腕・初回は dry run で Ctrl-C → 脱力を目視）

1. モデル取得（HF → Thor は約 10MB/s・12.6GB に約 20 分。先に落としておく）:
   `hf download Orboh/gr00t-bi-openarm-okra-chest-wristr-v1 --local-dir ~/models/gr00t-bi-okra-v1`。`diff <(ls ~/models/gr00t-bottle-v2) <(ls ~/models/gr00t-bi-okra-v1)` でファイル構成が一致。
2. embodiment の一致確認: Thor の `~/projects/Isaac-GR00T-n1.7/openarm_embodiment/openarm_config.py` が §1 で配置した版であること。学習時（EC2）と推論時（Thor）で modality が食い違うと実機で無意味な出力になる。`OPENARM_EMBODIMENT=bimanual ~/projects/Isaac-GR00T-n1.7/.venv/bin/python -c "import sys; sys.path.insert(0,'$HOME/projects/Isaac-GR00T-n1.7/openarm_embodiment'); import openarm_config as c; print(c.LAYOUT, list(c.KEYS))"` が `bimanual` と 4 キー。
3. CAN を上げ（`canup.sh`・4 本のうちフォロワー 2 本が要る）、motorcheck で右左 16 モータ応答、`drops [none]`。
4. PolicyServer（gr00t venv・tmux）。**env をサーバ側プロセスに付ける**（config はここで import される）:
   ```
   export HF_TOKEN=$(cat ~/.cache/huggingface/token); export HF_HOME=$HOME/.cache/huggingface
   R=~/projects/Isaac-GR00T-n1.7; cd $R
   OPENARM_EMBODIMENT=bimanual OPENARM_CAMERAS=chest,wrist_right \
   $R/.venv/bin/python gr00t/eval/run_gr00t_server.py \
     --model-path ~/models/gr00t-bi-okra-v1 --embodiment-tag NEW_EMBODIMENT \
     --modality-config-path $R/openarm_embodiment/openarm_config.py --host 127.0.0.1 --port 5555
   ```
5. 両腕を基準姿勢に置き、DC PSU に手が届く位置に立つ。接続時整列で **左→右の順に各腕約 2.2 秒自走**する。
6. dry run（lerobot venv・対話端末・ENTER が要る）:
   ```
   cd ~/projects/openarm-orboh/scripts/g4
   MODEL=$HOME/models/gr00t-bi-okra-v1 bash run_rollout.sh bi-okra-dryrun -- \
     --robot.type=bi_openarm_follower \
     --robot.left_arm_config.port=can_fol_l --robot.left_arm_config.side=left \
     --robot.right_arm_config.port=can_fol_r --robot.right_arm_config.side=right \
     --robot.id=follower \
     --robot.cameras='{"chest": {"type":"intelrealsense","serial_number_or_name":"344422070898","width":640,"height":480,"fps":30}, "wrist_right": {"type":"intelrealsense","serial_number_or_name":"352122273417","width":640,"height":480,"fps":30}}' \
     --robot.left_arm_config.max_relative_target='{joint_1: 2.5, joint_2: 2.5, joint_3: 3.0, joint_4: 5.0, joint_5: 5.0, joint_6: 4.0, joint_7: 5.0, gripper: 15.0}' \
     --robot.right_arm_config.max_relative_target='{joint_1: 2.5, joint_2: 2.5, joint_3: 3.0, joint_4: 5.0, joint_5: 5.0, joint_6: 4.0, joint_7: 5.0, gripper: 15.0}' \
     --lang_instruction="<収集の TASK と一字一句同じ>" \
     --policy_host=127.0.0.1 --policy_port=5555 \
     --dry_run=true --fps=30 --max_steps=64 --action_horizon=16 --log_obs_every_step=true
   ```
   要点: `max_relative_target` は **腕ごとに 8 キーの dict**（top-level は BiOpenArmFollowerConfig に無い・7 キーや None は起動時 SystemExit）。初期値は bottle v2 デモの 1 step 変化量 P99.9 実測（joint1/2=2.5, joint3=3.0, joint4/5/7=5.0, joint6=4.0, gripper=15.0）を左右同値で置き、両腕 okra デモの `data/*.parquet` から同じ P99.9 を取り直して置き換える（スカラー一律 1.0 はデモ実速度の 1/4〜1/12 で軌道を壊した）。この値は MIT 位置制御では復元トルク上限も兼ねるので、小さすぎると hold が重力で垂れる。`--robot.cameras` の挿入順（chest, wrist_right）が video キー順になるので modality.json と同じ順で書く。`--lang_instruction` は既定文がボトル用なので必ず渡す（09-19 に 4 本とも既定文で回した穴）。開始姿勢 YAML は run_rollout.sh が左右それぞれ `config/initial_pose_{left,right}.yaml` を自動付与し `command.sh` に残す。`--log_obs_every_step` は明示必須（無指定は exit 2）。
7. dry run で見るもの: (a) `layout = bimanual（state 16 次元）/ camera_keys = ['chest','wrist_right']` のログ、(b) 初回指令の乖離が腕 14 関節で閾値 20 未満（gripper 2 本は除外され参考値でログ）、(c) chunk 内 dt が 1/30 秒に収まる、(d) **腕が動いている最中に Ctrl-C を 1 回**押して、`left ... torque disable 成功` と `right ... torque disable 成功` が bus ごとに出て両腕が脱力する。Ctrl-C は 1 回だけ（2 回目は後片付け自体を中断する）。Damiao の error latch で脱力しなければ PSU OFF → 10 秒 → ON。
8. 実駆動: `--dry_run=false --max_steps=600` に変え、タグを `bi-okra-cellN-1` 等に。1 本ごとに `~/rollouts/<UTC>-<tag>/` の `notes.md` に目視所見と判定（L1〜L4）を書き、rollout 台帳（`rollout-log-*.md`）に 1 エントリ。J3 限界張り付きが出たら止める（09-19 の安全所見）。
9. 回収: Mac または Vector から `pull_rollout.sh <UTC>-<tag>`（軽量メタ）、生データは `convert_trajlog_to_lerobot.py` で `Orboh/bi_openarm_okra_rollouts_v1` へ。

根拠: `scripts/g4/run_rollout.sh` ヘッダと本文（両腕例・自動付与・layout 自己矛盾ガード・`--log_obs_every_step` guard）、`scripts/g4/eval_openarm.py` docstring と `_check_max_relative_target`・`SafeRunner._buses`・`_arm_only_deviation`（unit D 報告）、pipeline-design §6.6（PolicyServer 起動・二段ゲート・Ctrl-C 検証・乖離ゲート）・§5.-2（HF → Thor 10MB/s・embodiment の突合）、devlog 2026-08-24（続）（dict 化の根拠と P99.9 値）、memory `project_openarm-jev-decision-layer` gotcha（max_relative_target と復元トルク）、devlog 2026-09-18〜19（`--lang_instruction` 未指定の穴・J3 張り付き）。

---

## 7. 未確定事項（人が決める）

1. **タスク定義**: 両腕オクラの言語指示文（`TASK`）と役割分担（右で把持・左でかご保持か、両腕交互か）。文字列は収集・`--lang_instruction`・データセット名の `TASK_SLUG` の 3 か所で一致させる必要があるので先に固める。エピソード尺（`EPISODE_TIME_S`。両腕は 20 秒で足りない可能性・打ち切り到達を避ける値）。
2. **板橋シーン YAML の実測**: `scenes/itabashi-desk-v1.yaml`（名前は仮）の机高さ・昇降・かご位置・散らし範囲・照明。Kota の `tape-desk-v1.yaml` は TODO 40 件のまま未コミットなので、新規に作って全 TODO を埋める。緑格子かご（TBR 所有・持ち出せず）の代替容器をどうするか（09-19 の実測でリリースは容器の見た目に依存）。
3. **Kota の 9/5 FT 条件**（EC2 インスタンス型・step 数・費用・wrist 版 embodiment の内容）: 未回答。2 カメラ FT の唯一の先行実績なので、§5-4 の step/batch の目安を実績で置き換えられる。併せて Thor の未コミット（`modality.json` wrist 版・`initial_pose_right.yaml` 9/5 版・lerobot `lerobot_record.py`）の意図確認と、左右 YAML の対コミット。
4. **左 wrist カメラ**: 今回は無し（端子無し）。追加時は `WRIST_LEFT=true WRIST_LEFT_SERIAL=...` で obs が `chest_wristr_wristl` になり別データセット・別モデル系統になる。
5. **HF の人手操作 2 件**（dataset と model の器作成・トークンへの write 付与）を誰がいつやるか（§3-1・§5-7a）。
6. **max_relative_target の両腕実測値**: bottle 由来の初期値で dry run まで進め、okra デモの P99.9 に置き換えるタイミング（初回実駆動前）。
7. **AWS 9/30 失効前に回すか**: 費用は 10 ドル未満なので失効前に回す実益は小さいが、収集 50 本が 9 月中に揃うかで判断。EC2 は共有機体で Mac 側の π0（openpi）作業とも同居するので bridge で調整。