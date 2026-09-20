# scripts/jev — Jev（TypeSafe System One）判断層で OpenArm を動かす

設計正本: orboh-company-vault `07Member's-free-space/Kento/02-Tech-Plan/openarm-jev-decision-layer-design.md`

## 何をするか

Jev は関節角を出せない。有限のアクション（`j1_plus` … `j7_minus` / `hold` / `done`）から1つ選ばせ、
実行は lerobot `openarm_follower` の位置指令 30Hz（`max_relative_target` クランプ付き）で行う。
判断 2Hz・実行 30Hz の二層。判断が無い間は hold（最後の指令を再送）。

- `jev_arm.py` 本体。`--decider=rules`（Jev 不要・決定論）／`--decider=jev`（本番・`TYPESAFE_API_KEY` 必須）
- `run_jev.sh` 記録ラッパー。`~/rollouts/jev/<UTC>-<tag>/` に console.log / jev_arm.log / decisions.jsonl / traj.csv / command.txt

## 安全

- 別端末から `touch /tmp/openarm_estop` で hold → torque disable → disconnect（ソフト E-stop。安全プロトコル TODO#3 の最小実装・このスクリプト内のみ）
- Ctrl-C 1回で同じ停止処理。2回目は即終了（腕の脱力を目視確認）
- 唯一確実な停止は DC PSU の電源断。実駆動時は必ず手が届く位置で
- `confidence < 0.6` の判断・API エラー・リース 3 秒超過は hold。401/403 は即中止。累積 $2 で停止
- `step_deg` は 10 度以下（既定 5）。グリッパは `--enable_gripper=true` を明示しない限り候補に入れない
- **hold は「意図姿勢 q_des の保持」**。executor は実測に再基準化しない（2026-09-20 に実測起点だったため重力で垂れた位置を毎判断受け入れ、手首が 74°→1° まで落ちた）
- **追従誤差ガード** `--track_err_max_deg`（既定 8）: 関節 1..7 のどれかで |実測 − 前 tick の指令| が超えたら停止処理（重力負け・接触・モータ異常）
- **`max_relative_target` は MIT 位置制御では復元トルク上限（kp × clamp）**。小数 1.0 は手首 0.44 Nm・肩 4.2 Nm で、開始姿勢の重力負荷（手首≈1.2 / 肩≈6.5 Nm）に負ける。実機は関節別 dict（`run_jev.sh` の `JEV_CLAMP`＝肩肘 2.0 / 手首 4.0 / グリッパ 1.0）。起動時に関節ごとの上限を表示し、負荷を下回る関節に警告する

## 段階（この順で）

1. `--no_robot=true --decider=rules` 通電なし。ループ・記録・E-stop ファイルの動作確認
2. `--no_robot=true --decider=jev` API キー取得後の初回。レイテンシ・token・confidence の実測
3. 実機 `--dry_run=true --decider=rules` 接続・自動整列・Ctrl-C 停止の確認（通電あり・指令なし）
4. 実機 `--dry_run=false --decider=rules` 決定論で 1 関節 10 度。腕が期待方向へ動くか
5. 実機 `--dry_run=false --decider=jev` 本番

腕なしでガードの回帰: `--no_robot=true --decider=rules --no_robot_sag_deg_s='{"joint_7": 30.0}'` で joint_7 が垂れる模擬をし、追従誤差で停止することを確認する。

## 実機の記録（2026-09-20・右腕）

- 段階 2（腕なし Jev）: 4 判断で +10.00° 到達。ウォーム往復 約 200ms・入力 約 1,390 token/call・confidence 0.95/0.59/0.67/0.94（0.59 は閾値で hold）
- 段階 3（dry-run）: 接続・整列・E-stop（touch から 1 秒以内に torque disable）とも成功
- 段階 4（rules 実駆動・修正前）: joint_4 +10° の指令に対し joint_7 −72.5°・joint_1 +36.7°・joint_4 −21.9° の想定外動作。原因は上記「復元トルク上限」と executor の実測再基準化。本 README の対策を実装（実機での再確認は次回）

コマンド例は `run_jev.sh` 冒頭。
