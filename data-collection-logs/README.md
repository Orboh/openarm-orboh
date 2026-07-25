# data-collection-logs

テレオペによるエピソード収集(`lerobot-record`)セッションの**実行ログを恒久保存**する場所。
lerobot-record はデータセット(parquet+動画)は書くが、**実行ログ(config・タイムスタンプ・エピソード毎の
record/再録/reset・警告)を残さない**——ターミナルのスクロールバックにしか無く、閉じたら消える。この穴を埋める。

## いま入っているもの(手動キャプチャ)

- `2026-07-20-1749-bottle-pnp-right-10ep.log` / `2026-07-20-1814-bottle-pnp-right-10ep.log`
  — Thor スクロールバックから手貼りで救出した 10ep×2 run(bottle pick-place・右腕)。両方 `push_to_hub=False`・
    **同一 repo_id `Orboh/openarm_bottle_pnp_chest_v1`**=一方が上書きされた可能性大(データ消失リスクの実例)。
    再録の嵐(run1 ep3 は約16回)=開始姿勢が不定でテレオペが難しかった痕跡。

## 恒久ログ機構(提案・未実装)

`record_pick_place_openarm.sh` を次のように補強する(実装は別途承認後):

1. **実行ログの自動保存**: 実行全体を `tee data-collection-logs/$(ts)-<repo>-<side>.log` に流す(stdout+stderr)。
2. **マニフェスト(sidecar)**: 実行前に `<ts>.manifest.json` を書く — timestamp / repo_id / side / num_episodes /
   カメラ構成 / openarm-orboh & lerobot の git commit / calibration id。実行後に「実際に保存された ep 数・再録回数・
   開始姿勢サマリ(各関節 min/max/std)」を追記。
3. **上書きガード**: 収集先データセットが既存なら**警告して停止 or 自動採番**(同一 repo_id 上書きによる消失を防ぐ)。
   全社データ管理標準(orboh-company-vault `07.../Kento/02-Tech-Plan/orboh-data-management-standard.md`)に紐付ける。

## 運用

- 生ログは `*.log`=git 管理外(このディレクトリの `.gitignore`)。肥大化させない。共有が要るものは要約を
  vault(Dev-Log)へ。データセット本体の受入・命名・可視性はデータ管理標準に従う。
