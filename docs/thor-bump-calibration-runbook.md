# Thor 実機チューニング・検証 runbook: bump-to-stop 較正と開始姿勢固定

対象: OpenArm leader/follower(左右)。Vector 側でオフライン検証済みのコードを Thor 実機で仕上げるための手順。
コード: lerobot fork `~/projects/lerobot`(branch `feat/record-headless-keyboard`)
中核: `src/lerobot/motors/damiao/damiao_bump_calibration.py`
起動: `openarm-orboh/scripts/calibrate_openarm.sh <left|right> bump_to_stop`

重要: 以下は全て実機でのみ確定できる。Vector 上では式・シーケンス・CLI 配線のみ検証済みで、
しきい値・ストッパ角・安全性は未検証。初回は必ず1関節ずつ様子を見ながら実施する。

---

## 0. 仕組みの要約

- bump-to-stop は各アーム関節(J1..J7、グリッパ除外)をソフト MIT でストッパに当て、
  接触時の native 生角と既知ストッパ角から offset = raw_at_stop - stop_angle を計算する。
- 論理ゼロ(URDF 真下垂れ)は較正終了時の姿勢に依存しない(公式スクリプトの
  move_to_precise_home TODO を式で回避)。
- 接触判定: |vel| < 速度しきい値 かつ |torque| > トルクしきい値 かつ トルク符号が押し込み方向、
  を2サンプル連続。
- グリッパは決してバンプしない。offset は 0 に固定。前提として flash zero を完全閉鎖で焼いておく。
- **導出の正本(なぜこの式で機械基準になるか・誤差伝播・stop 角の測定式・限界)** →
  `~/Documents/Humanoid-Research/04-Hardware/OpenArm/OpenArm-Zero-Calibration-and-Start-Pose.md`
  の「bump-to-stop の零位はなぜ機械基準になるか(導出と stop 角の測定手順)」。
  本ファイルは手順のみを持ち、理論は二重化しない。

---

## 1. 事前準備

- CAN up 確認: `ip link | grep can_`(can_fol_l / can_lead_l / can_fol_r / can_lead_r)
- 前後の disable cycle(Damiao quirks: stale 半enabled 対策):
  `cansend <canX> 00N#FFFFFFFFFFFFFFFD` を全モータへ、または既存手順に従う
- モータ8個全応答の確認(J6 脱落既往あり。全値0・温度0 のモータがいたら power cycle)
- グリッパ手動ゼロ(未実施なら。leader/follower 両方):
  1. `python scripts/openarm_gripper_zero.py <canX>`
  2. グリッパを手で完全閉鎖に保持して ENTER
  3. AFTER 値が ~0.00 deg であること
  ※ 破損済みグリッパは閉鎖位置が再現しないため再ゼロ無効(部品交換が先)
- ワークスペース確保: 腕の全掃引域(肩パン ±、肘 0..140°、手首各軸)に物・人がないこと
- 非常停止: 電源スイッチと Ctrl-C に即手が届く体勢。初回は2人推奨(1人が電源係)

---

## 2. 初回実行(1腕目)と bump 順序の安全確認

1. `bash scripts/calibrate_openarm.sh right bump_to_stop`(まず片側 follower から)
2. 警告プロンプトで side 表記を確認(右腕に left シーケンスを流すと逆ストッパへ向かい危険)
3. 掃引順を目視確認(公式順のポート):
   J4(肘伸ばし・負方向) → J2 微動+J3(+90) → J4 を+90(前腕水平化) →
   J5(+90) → J6(+45) → J7(+90) → J2(下限) → J1(下限/上限) → J4 戻し
4. 各バンプで確認すること:
   - ストップ接触の直前に減速し、接触ログ `bump_to_stop(joint_x): stop contact after ...` が出る
   - 接触後に押し込みが解除される(モータがストッパへ唸り続けない)
   - 隣接する構造物・ケーブルに干渉しない
5. 異常時は即 Ctrl-C(finally で disable_torque が走り脱力する)。脱力しないモータが
   残ったら power cycle(disable latch 既往)

---

## 3. しきい値チューニング(per-joint)

デフォルト(公式値の deg 換算、`damiao_bump_calibration.py`):
- joint_1: torque > 5.0 Nm, |vel| < 0.72 deg/s
- joint_2..7: torque > 2.0 Nm, |vel| < 5.73 deg/s

症状と調整(CLI 上書き例は下):
- 途中で誤検知して止まる(ストッパ未到達でログが出る):
  そこの姿勢での重力・摩擦トルクがしきい値を超えている。該当関節の torque しきい値を上げる
  (+0.5 Nm 刻み)。特に J5/J6/J7 は前腕水平姿勢でバンプするため重力モーメントが乗りやすい
- ストッパに当たっても検知しない(travel limit abort になる):
  torque しきい値を下げる、または速度しきい値を上げる
- 検知が遅くストッパを強く押す: torque しきい値を下げる(ソフトゲイン kp45 なので過大には
  なりにくいが、幾何的に弱い J6 は特に保守的に)

上書き例(follower):
```
lerobot-calibrate --robot.type=openarm_follower --robot.port=can_fol_r \
  --robot.side=right --robot.id=follower_right \
  --robot.calibration_anchor=bump_to_stop \
  --robot.bump_torque_thresholds_nm='{joint_5: 2.5, joint_6: 1.5}' \
  --robot.rezero_on_connect=false --robot.align_on_connect=false
```
確定値は calibrate_openarm.sh に環境変数として焼き込むか、このファイルに記録して共有する。

---

## 4. per-unit ストッパ角(MECH_LIM)の検証と上書き

公称値(URDF リミット準拠、右): J1 -80 / J2 -10 / J3 +90 / J4 0 / J5 +90 / J6 +45 / J7 +90 deg
(左は J1 +80 / J2 +10、他は同じ)。個体の組付け公差でズレるため、較正後に検証する:

1. 較正完了直後、腕は真下垂れ近傍に戻っている。ログに
   `... reads X deg from hang-down after the sequence` 警告が出ていないこと(>10 deg で警告)
2. 手で(torque off)腕を正確に真下に垂らし、teleop などで論理角を読む
   (簡易には `capture_initial_pose.py --no-gravity` のライブ表示が使える)
3. 全関節 |論理角| <= 0.05 rad(~2.9 deg)であること
4. 特定関節だけ系統的にズレる場合、その関節の実ストッパ角が公称からズレている。
   **測定式** (導出は下記ポインタ):

       stop_真 = stop_仮定 − (自由懸垂で読んだ論理角)

   仮定した stop 角の誤差は、自由懸垂での論理読みに 1:1 でそのまま現れる。
   つまり 4-2/4-3 で読んだ値をそのまま公称から引けば実ストッパ角が出る
   (符号注意: 読みが +2.0 deg なら公称 −80 → 実 −82.0)。外部の角度計は不要。
   得た値を `--robot.bump_stop_angles_deg='{joint_1: -82.0}'` の形で上書きし
   再較正 → 再検証。収束するまで反復。確定した個体値は本ファイルに記録:

per-unit 確定値の記録欄(較正のたび更新):
- follower_right: (未計測)
- leader_right: (未計測)
- follower_left: (未計測)
- leader_left: (未計測)

---

## 5. ±0.05 rad 基準検証(較正の合否判定)

- 方法: 上記 4-2/4-3。真下垂れで全関節 |論理角| <= 0.05 rad なら合格
- **±0.05 rad (~2.9 deg) は公式 docs の受入基準**(Orboh が決めた値ではない):
  https://docs.openarm.dev/tutorial/setup/ 「all joint positions should read within
  ±0.05 rad of zero when the arm is in its reference pose」。
  公式の bump スクリプト側には合否判定が無い(`ideal` との差分計算までで終わる)。
- 加えて既知姿勢での交差確認: 肘だけ 90° に曲げて J4 が ~90 deg を指すこと
- leader / follower 両方で実施(それぞれ独立に較正されるため)
- **最終的な合否は絶対精度ではなく再現性と左右一致で見る**: §6 の電源断を挟んだ
  offset 一致と、§7 の leader/follower 開始一致が実質的な判定。理由は導出ノートの
  「構造上の帰結」節(人の判断が毎回から1度へ移るので、絶対誤差は 1 回分に律速される)。

---

## 6. 電源断永続チェック

1. 較正合格後、腕の電源 OFF → 10 秒 → ON
2. teleop をドライラン起動(`bash scripts/run_teleop_right.sh` 等)。connect() は
   再ゼロ焼きをスキップし(homing offset 有 → 自動判定)、check_start_position を通過して
   整列動作に入るはず
3. 起動直後の読み値が電源断前と一致すること(ズレたら DM の native フレームが
   ブート時に wrap している可能性 → 較正ログの ±120 deg 警告を確認。native 零位を
   openarm-can-cli set_zero で真下垂れ近傍に一度焼き直し、電源再投入後に再較正)

---

## 7. leader / follower 開始一致確認

1. `bash scripts/run_teleop_<side>.sh` 起動
2. 整列後、leader と follower が同じ物理姿勢(ready 姿勢 or 公式既定の肘36°)で
   静止していること。目視で骨格が一致し、グリッパは両方閉鎖
3. 追従開始直後にジャンプがないこと(開始一致が取れていれば初回指令差はほぼ0)

---

## 8. record 側 E2E(収集の開始固定)

1. ready 姿勢が未キャプチャなら `bash scripts/capture_initial_pose.sh <side>`
2. `NUM_EPISODES=2 EPISODE_TIME_S=5 bash scripts/record_pick_place_openarm.sh` でドライラン
3. 確認:
   - connect 時に両腕が ready 姿勢へ整列してから記録が始まる
   - エピソード間の reset_time_s は手動ウィンドウ。leader を手で ready 姿勢に戻す
     運用を徹底する(record 側に自動復帰はない)
   - 記録された各エピソードの先頭フレームの関節角がほぼ同一(±数 deg)であること
     をデータセットで確認(エピソード先頭の observation.state を比較)

---

## 9. inference 側 E2E(rollout の開始固定)

1. `POLICY_PATH=<学習済み> NUM_EPISODES=2 EPISODE_TIME_S=5 bash scripts/run_rollout_lerobot.sh`
2. 確認:
   - connect 時に follower が ready 姿勢へ整列 → その姿勢が initial_position として
     ログに記録される(`Captured initial robot position`)
   - 各エピソード後、follower が自動で initial_position(= ready 姿勢)へ戻る
     (--strategy.reset_to_initial_position=true)
   - 終了時も同姿勢へ復帰してから disconnect(--return_to_initial_position=true)
   - グリッパが開始時に閉鎖ゼロであること
3. 収集時の開始姿勢と推論時の開始姿勢が物理的に同一であることを最終確認
   (これが本件の目的。8-3 の先頭フレームと 9-2 の開始観測を突き合わせる)

---

## 10. 既知の abort と対処

- `no CAN response ... mid-bump`: モータ無応答 0.2 秒で中断・脱力。配線・電源・J6 脱落既往を確認
- `swept N deg without stop contact`: 可動域+30 deg 掃引しても未検知。しきい値過大か
  side 誤り。§3 で調整
- 較正後の hang-down 警告(>10 deg): 誤検知バンプかストッパ角の個体ズレ。§4 で切り分け
- 脱力しない/半enabled: power cycle(Damiao quirks ノート参照)

