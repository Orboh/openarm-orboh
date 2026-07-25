# data-collection-logs

テレオペによるエピソード収集(`lerobot-record`)セッションの**実行ログを恒久保存**する場所。
lerobot-record はデータセット(parquet+動画)は書くが、**実行ログ(config・タイムスタンプ・エピソード毎の
record/再録/reset・警告)を残さない**——ターミナルのスクロールバックにしか無く、閉じたら消える。この穴を埋める。

## いま入っているもの(手動キャプチャ)

- `2026-07-20-1749-bottle-pnp-right-10ep.log` / `2026-07-20-1814-bottle-pnp-right-10ep.log`
  — Thor スクロールバックから手貼りで救出した 10ep×2 run(bottle pick-place・右腕)。両方 `push_to_hub=False`・
    両ログの `repo_id` は `Orboh/openarm_bottle_pnp_chest_v1`(タイムスタンプなし)。
    **【2026-07-25 訂正】以前ここに書いていた「同一 repo_id=一方が上書きされた可能性大」は根拠が確認できない。**
    `lerobot-record` は設定ダンプ(`lerobot_record.py:446`)を `stamp_repo_id()`(`:512`)より 66 行前に出すため、
    **ログに残る repo_id は必ずスタンプ付与前の値**であり、実際の書き込み先を示さない。つまり上書きの有無は
    このログからは判定不能=**未証明**。加えてローカルの in-place 上書きは upstream が禁じている
    (`datasets/dataset_metadata.py:710` の `mkdir(parents=True, exist_ok=False)` が FileExistsError で停止)。
    **【2026-07-25 決着=上書きは「偽」】** Thor 上で `ls -1 ~/.cache/huggingface/lerobot/Orboh/` を確認した結果、
    `openarm_bottle_pnp_chest_v1_20260714_191628` / `_20260720_174958` / `_20260720_181437` / `_merged25` の
    **4 ディレクトリが独立して現存**＝3 run すべて無傷。`stamp_repo_id()` が実際にパスを分離していた。
    再録の嵐(run1 ep3 は約16回)=開始姿勢が不定でテレオペが難しかった痕跡。

## 恒久ログ機構(提案・未実装)

`record_pick_place_openarm.sh` を次のように補強する(実装は別途承認後):

1. **実行ログの自動保存**: 実行全体を `tee data-collection-logs/$(ts)-<repo>-<side>.log` に流す(stdout+stderr)。
2. **マニフェスト(sidecar)**: 実行前に `<ts>.manifest.json` を書く — timestamp / repo_id / side / num_episodes /
   カメラ構成 / openarm-orboh & lerobot の git commit / calibration id。実行後に「実際に保存された ep 数・再録回数・
   開始姿勢サマリ(各関節 min/max/std)」を追記。
3. **~~上書きガード(ローカル)~~ → 不要と判定(2026-07-25)**: ローカル収集先の上書きは upstream が既に禁止済み
   (`dataset_metadata.py:710`)。さらに `stamp_repo_id()` が Python 内で repo_id を変えるため、**shell 側の事前
   チェックは実際の書き込み先を知り得ず恒久的に空振りし、誤った安心を与える**。かわりに守るべきは
   **merge→push 経路**: `lerobot-edit-dataset merge` の `--new_repo_id` は自動採番されず、`push_to_hub` は
   `create_repo(exist_ok=True)`＋`upload_folder`(削除パターンなし)で既存 HF リポジトリに重ね書きし、
   **旧 chunk が orphan として残る**。push 前に HF 側リポジトリの存在確認＋命名規則を入れる(約10行)。
   全社データ管理標準(orboh-company-vault `07.../Kento/02-Tech-Plan/orboh-data-management-standard.md`)に紐付ける。

## 運用

- 生ログは `*.log`=git 管理外(このディレクトリの `.gitignore`)。肥大化させない。共有が要るものは要約を
  vault(Dev-Log)へ。データセット本体の受入・命名・可視性はデータ管理標準に従う。
