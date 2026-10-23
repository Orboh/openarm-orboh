# orboh-sim で GR00T N1.7 を FT する手順（共用ユーザー spot）

作成: 2026-10-21 / 対象: 固定の Linux PC から `kota0` プロファイル（SSO ユーザー spot）で操作する人
実績: 2026-10-21 に g5.8xlarge 上で準備（変換〜dataloader 疎通）まで全段通過。学習は A10G では VRAM 不足（§3）。

本線の手順書は `docs/bimanual-pipeline-runbook.md` §4〜5（`act-okura-pick` 用）。この文書は orboh-sim 固有の差分をまとめる。

## 0. orboh-sim 固有の前提

| 項目 | 内容 |
|---|---|
| インスタンス | `i-0db430a2eff6cd78c`（us-east-1d） |
| 普段の型 | **g5.8xlarge**（A10G 24GB）。型を変えたら終わったら g5.8xlarge に戻す（m5.xlarge ではない） |
| 共用 | シミュレーション（リムレスホイール等）と共用。使う前後に machine-bridge か Slack に書く |
| ルートの EBS | 400GB のうち空き約 14GB（他用途で埋まっている）。**GR00T 関係は一切置かない** |
| 作業場所 | ローカル NVMe `/opt/dlami/nvme`（約 780GB）。**止めると消える**ので、起動のたびに準備スクリプトを流し直す |
| GR00T | 既存の `/home/ubuntu/Isaac-GR00T` は N1.6。N1.7 は NVMe 上に別に clone する |
| HF トークン | `/home/ubuntu/.cache/huggingface/token`（Kota0612）。Kota0612 の private データセットを読める |
| 自動停止 | `orboh-idle-watch`: GPU 使用率 5% 未満が 60 分続くと停止。準備中（GPU を使わない）は `sudo orboh-idle-keep 3h` で延長する |
| spot の権限 | 起動・停止・型変更・SSM コマンドのみ。reboot・コンソール出力・IAM は不可 |

## 1. ログインと起動（PC で）

    aws sso login --profile kota0
    aws ec2 start-instances --profile kota0 --instance-ids i-0db430a2eff6cd78c
    aws ec2 wait instance-running --profile kota0 --instance-ids i-0db430a2eff6cd78c
    # SSM が Online になるまで 1〜3 分
    aws ssm describe-instance-information --profile kota0 \
      --filters Key=InstanceIds,Values=i-0db430a2eff6cd78c --query 'InstanceInformationList[0].PingStatus' --output text

学習に使うなら、起動の前に型を g6e に変える（stopped のときだけできる）:

    aws ec2 modify-instance-attribute --profile kota0 --instance-id i-0db430a2eff6cd78c --instance-type g6e.4xlarge

`InsufficientInstanceCapacity` なら g6e の 4xlarge / 2xlarge / 8xlarge / 12xlarge を順に試す（runbook §5-1）。

## 2. 準備（変換〜dataloader 疎通・約 15 分）

    export DS_ID=Kota0612/bi_openarm_takeei_wood_l1_chest_wristr_wristl_v1
    export OPENARM_CAMERAS=chest,wrist_right,wrist_left   # データセットのカメラに合わせる
    scripts/ec2/ssm_run.sh exec <(echo 'orboh-idle-keep 3h')
    scripts/ec2/ssm_run.sh start scripts/ec2/gr00t_n17_prepare.sh
    scripts/ec2/ssm_run.sh tail  scripts/ec2/gr00t_n17_prepare.sh   # STOPPED と PREPARE_DONE が出たら完了

`DATALOADER_PASS` が出ていれば学習に進める。

## 3. 学習

前提: Kota0612（トークンのアカウント）が `nvidia/Cosmos-Reason2-2B`（gated）にアクセスできること。
N1.7 は base を含む全チェックポイントがこれを backbone として読むので、無いと学習の起動直後に止まる。
確認: `hf download nvidia/Cosmos-Reason2-2B config.json --local-dir /tmp/cr2` がエラーにならないこと。

    export RUN_NAME=ft_bi_takeei_wood_l1_v1 STEPS=10000 BATCH=32
    export HF_MODEL_REPO=Kota0612/gr00t-bi-takeei-wood-l1-v1   # 任意。先に model/private で作っておく
    export CKPT_DEST=s3://orboh-models/gr00t-ckpt              # 保存のたびに checkpoint を上げる（推奨・下記）
    scripts/ec2/ssm_run.sh start scripts/ec2/gr00t_n17_train.sh
    scripts/ec2/ssm_run.sh tail  scripts/ec2/gr00t_n17_train.sh    # 最終行に watchdog の vram/peak/avail

2026-10-23 追加（学習設定の実験 E1〜E3 用）。どれも env で変える:

| env | 既定 | 内容 |
|---|---|---|
| `SAVES` | 8 | 残す checkpoint の本数。`--save_steps` は `STEPS / SAVES`（割り切れないと起動前に止める）。`--save_only_model` 付き（1 本 約 12.6GB・再開はできない） |
| `CKPT_DEST` | なし | `s3://...`（`$RUN_NAME/checkpoint-N` に置く）か `hf:<repo>`。保存のたびにバックグラウンドで上げ、最後に全本上がったか確かめる。足りなければ `CKPT_PUSH_INCOMPLETE` で止まる＝**止める前に上げ直す** |
| `SEED` | 42 | `gr00t/patches/0001-finetune-seed.patch`（prepare で当てる）で `--seed` を足している。upstream には無い |
| `USE_WANDB` / `WANDB_PROJECT` | 1 / `gr00t-openarm` | loss 曲線は W&B に送る（run 名 = `RUN_NAME`）。ubuntu の `~/.netrc` に W&B の認証が無いと起動前に止める |

- 8 本 × 12.6GB ≒ 100GB。HF（Kota0612 は無料プラン）の private 容量に収まらないおそれがあるので、checkpoint は S3、
  選んだ 1 本だけを `HF_MODEL_REPO` に上げる。EC2 のロールには `orboh-models` への書き込み権限（`orboh-models-write`）がある。
- 最終モデルと一緒に `trainer_state.json`（loss の記録）も `HF_MODEL_REPO` に上がる。
- 2026-10-21 の学習は「最後に 1 回だけ上げる」作りで、上がらないまま停止して NVMe ごと失った（HF のリポジトリは空のまま）。

### 3-1. checkpoint ごとの open-loop 誤差

    export TRAJ_IDS=0,1,2,3          # 検証用データセットの episode 番号（§3-2 で取り分けたもの）
    scripts/ec2/ssm_run.sh start scripts/ec2/gr00t_n17_openloop.sh
    scripts/ec2/ssm_run.sh tail  scripts/ec2/gr00t_n17_openloop.sh   # 最後に checkpoint ごとの平均 MSE の表

GR00T 同梱の `gr00t/eval/open_loop_eval.py` を checkpoint ごとに回すだけ。出力は `/opt/dlami/nvme/gr00t/eval/$RUN_NAME/`
（`summary.tsv`・`summary_mean.tsv`・`checkpoint-*/traj_*.jpeg`）。NVMe なので止める前に S3 に写す:
`aws s3 sync /opt/dlami/nvme/gr00t/eval/$RUN_NAME s3://orboh-models/gr00t-eval/$RUN_NAME`。

### 3-2. 検証用の手本を取り分ける（新しく録るデータセットごと）

学習に使った手本で 3-1 を回すと誤差が小さく出て選定に使えない。収集後、学習の前に各条件（配置カード）から 2〜3 本を外す。
LeRobot 標準の `lerobot-edit-dataset` の split を使う（元のデータセットは消えない）:

    lerobot-edit-dataset --repo_id Kota0612/<dataset> \
      --operation.type split \
      --operation.splits '{"train": [<学習に使う episode 番号>], "val": [<検証用の episode 番号>]}' \
      --push_to_hub true
    # → Kota0612/<dataset>_train と Kota0612/<dataset>_val ができる

- 検証用の番号は配置カードごとに選ぶ（どの episode がどのカードかは収集の manifest で確かめる）。
- 学習は `DS_ID=..._train`、3-1 は `DS_ID=..._val` で prepare を流してから回す。`_val` の episode 番号は 0 から振り直されるので、
  `TRAJ_IDS` は `0,1,...` になる。

- A10G（g5.8xlarge・24GB）では学習できない。2026-10-21 に batch 4 でも optimizer の初期化で OOM（22.5GB 使用）。
  モデル読込・dataloader・forward/backward までは通ることは確認済み。本番は必ず g6e（L40S 48GB）で回す
- 出力は NVMe の `/opt/dlami/nvme/gr00t/out/$RUN_NAME`。**止める前に HF に上げる**（止めると消える）

## 4. 終わったら

    aws ec2 stop-instances --profile kota0 --instance-ids i-0db430a2eff6cd78c
    aws ec2 wait instance-stopped --profile kota0 --instance-ids i-0db430a2eff6cd78c
    # g6e にしていたら戻す
    aws ec2 modify-instance-attribute --profile kota0 --instance-id i-0db430a2eff6cd78c --instance-type g5.8xlarge

machine-bridge / Slack に終了を書く。

## 5. つまずいたところ（2026-10-21）

| 症状 | 原因と対処 |
|---|---|
| 起動しても SSM が `None` のまま、send-command が `InvalidInstanceId` | stop → start し直したら Online になった。ロール `orboh-dcv-role` には `AmazonSSMManagedInstanceCore` が付いている |
| `uv sync` が `torchcodec-*.whl: Invalid zip file structure` | `GIT_LFS_SKIP_SMUDGE=1` で clone すると wheel が LFS ポインタのまま。`git lfs pull --include "*.whl"` で取る（スクリプトに入れ済み） |
| `check_dataloader.py` が `unexpected keyword argument 'video_backend'` | Isaac-GR00T e574928（2026-06-19）以降で引数が無くなった。受け付ける版でだけ渡すよう修正済み |
| `launch_finetune.py` が `Unrecognized arguments: True` | tyro の bool フラグは値を付けない（`--save_only_model`） |
| `Cannot download the VLM backbone 'nvidia/Cosmos-Reason2-2B'` | gated（自動承認）。Kota0612 でモデルのページを開き、ライセンス本文のいちばん下の同意ボタンを押す。反映に 1 分ほどかかる。トークン側は fine-grained なら「public gated repos を読む」権限が要る（orboh-sim の `kimodo` は付いている） |
| A10G で `torch.OutOfMemoryError`（batch 4 でも） | 3B モデル＋Adam の状態が 24GB に収まらない。g6e に上げる |
