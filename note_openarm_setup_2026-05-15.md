# OpenArm（バイマニュアル）のセットアップで遭遇した5つの問題と全部の解決ログ

ヒューマノイドロボティクスのスタートアップで CTO をしている上田と申します。
Enactic 社製のオープンソース双腕ロボット「**OpenArm**」を Ubuntu 22.04 マシンに新規セットアップし、**Leader-Follower のバイラテラル遠隔操作（force feedback 付き）が両腕で動く** ところまで到達した日のログです。

公式ドキュメント通りに進めても、実機で**5つの想定外**にぶつかりました。それぞれの**原因と解決策**を、後続の人がスムーズに辿れるよう全部記録しました。

> 本稿は **2026-05-15** 1 日分のセットアップ作業から書き起こしています。

---

## OpenArm とは

Enactic 社が公開しているオープンソースの **双腕ロボット**。Damiao 製ブラシレスモータ 8 個 × 2 アーム（左右）× 2 ペア（Leader / Follower）= **計 32 モータ**を CAN-FD 経由で制御し、人間が Leader 側を手で動かすと Follower 側が追従する**バイラテラル遠隔操作**ができます。

- 公式ドキュメント: <https://docs.openarm.dev/>
- 公式 GitHub: <https://github.com/enactic/openarm>

---

## ハードウェア構成

| 項目 | 内容 |
|---|---|
| PC | ThinkPad P16 Gen2, Ubuntu 22.04, Linux 6.8 |
| CAN アダプタ | **PEAK PCAN-USB Pro FD ×2台** (各2チャネル = 4 CAN ports) |
| USB 接続 | USB-A → USB-C 変換 (PC 側 USB-A ポート1個のみ) |
| ロボット | OpenArm v10 bimanual (4 アーム × 8 モータ) |
| ROS | ROS 2 humble |

---

## 公式の手順（理想）

OpenArm 公式 docs では以下の順序が示されています:

1. Motor ID Configuration (工場出荷時設定済み)
2. Setup CAN Interface
3. Motor Test
4. Motor Configuration (Zero Position Calibration)
5. Hands-On: Demo Run
6. Leader-Follower Teleop (Unilateral → Bilateral)

これを「**そのまま走らせれば動くはず**」と思って始めたのですが、実機 5 箇所で詰まりました。

---

## 詰まったポイント 1: conda が ROS のビルドを邪魔する

ROS 2 humble のワークスペース (`~/openarm_ros2_ws`) で `colcon build` すると:

```
ModuleNotFoundError: No module named 'catkin_pkg'
CMake Error: execute_process(/home/kota-ueda/miniconda3/bin/python3 ...)
```

CMake が PATH 先頭にある **conda の Python3.13** を拾ってしまい、conda には `catkin_pkg` が入っていないので失敗していました。

### 解決

```bash
conda deactivate
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
rm -rf ~/openarm_ros2_ws/build ~/openarm_ros2_ws/install ~/openarm_ros2_ws/log
colcon build --packages-select openarm_description --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

ポイント:

- `conda deactivate` してもパスは残るので、`PATH` から `miniconda` を含む行を明示的に除去
- すでに失敗した build キャッシュを `rm -rf` で完全消去
- `-DPython3_EXECUTABLE=/usr/bin/python3` で再発防止

これは ROS + Anaconda 環境では非常によくあるトラブルですね。

---

## 詰まったポイント 2: CAN バスとアームのマッピングが「想定と逆」

最初、目視でケーブルを追って:

- can0 = Leader R
- can1 = Leader L
- can2 = Follower R
- can3 = Follower L

と決めつけました（公式 launch スクリプトのデフォルトもこの並び）。

ところが**動作テスト**してみると逆。`can2` を駆動したら**リーダーの右腕**が動きました。

### 確定手順

各 CAN バスで motor 5° だけ動かして「**どの物理アームが動いたか目視確認**」するスクリプト (`openarm_motion_check.py`) を書いて 4 本全部テスト。結果:

| CAN | 実際のアーム |
|---|---|
| **can0** | Follower R |
| **can1** | Follower L |
| **can2** | Leader R |
| **can3** | Leader L |

つまり Adapter A (USB ポート 1-5) が **フォロワー両腕**、Adapter B (USB ポート 1-12) が **リーダー両腕** に繋がっていた。

PEAK PCAN-USB Pro FD は **シリアル番号が udev に露出していない**（2台のアダプタが完全に区別不可）ので、USB ポート位置でしか区別できません。USB ポートに物理ラベルを貼って、毎回同じ位置に挿す運用が必要。

### 運用スクリプト

USB のパス (`usb-0:5`, `usb-0:12`) から自動でロールを引き当てて `~/.config/openarm/can_map.env` を吐く `openarm_canup.sh` を作りました。

```bash
sudo /home/kota-ueda/Desktop/OpenArm/scripts/openarm_canup.sh
cat ~/.config/openarm/can_map.env
# → export OPENARM_LEADER_R=can2 など出力
```

以降のスクリプトはこれを `source` するだけで再起動・再接続後も正しいアームに繋がります。

---

## 詰まったポイント 3: 較正スクリプトが固まる（モータの "stale state"）

公式の零位較正コマンド:

```bash
/usr/bin/python3 /usr/bin/openarm-can-zero-position-calibration --canport can0 --arm-side right_arm
```

を走らせると `Enabling... → Enabled...` の後 **1分待っても何も動かない**。LED を見ると **8 モータ中グリッパ 1 個だけが緑**で、他 7 個は赤のまま。

### 原因

直前に `openarm-can-demo` を試しに走らせていて、モータ内部の状態が「半 enabled」のまま残っていた。新規 `init_arm_motors` の呼び出しがその残留状態を解消できず無限ループに入った。

### 解決

事前に **全 8 モータに明示 disable を送ってクリーンな状態に**:

```bash
for ID in 001 002 003 004 005 006 007 008; do
  cansend can0 ${ID}#FFFFFFFFFFFFFFFD
  sleep 0.05
done
```

→ 全部赤を目視確認 → 較正コマンド再実行 → **1秒以内に動き始めて完走** ✅

教訓: **OpenArm のスクリプトを実行する前後には必ず `cansend disable` で全モータをクリアする** 運用ルールを敷くべき。公式バイナリは exit 時に必ず disable してくれるわけではないようです。

---

## 詰まったポイント 4: グリッパの zero がずれていて teleop 起動でグリッパが破損

4 本全部の較正を完走させた後、いきなり右腕で unilateral teleop を起動:

```bash
~/openarm_teleop/script/launch_unilateral.sh right_arm can2 can0
```

→ Follower 側のグリッパが**機械リミットを越えて勢いよく開き、部品が破損**しました 😭

### 原因

公式の零位較正スクリプトが **グリッパ (J8) だけ正しく較正していない** のが根本原因。

スクリプトを読むと:

```python
if j == JointID.GRIPPER:
    print('ideal[GRIPPER] skipped')
    continue
```

と書かれていて、ideal-zero の計算をスキップしている。実行ログでも `[INFO] mechanical stop (Joint J?)` 行が J1〜J7 までは出るのに **GRIPPER だけ一行も無い**ことに後で気づきました。

つまり「グリッパの zero = 較正開始時にたまたまそこにあった位置」になる。**リーダーとフォロワーの zero が物理的に同じ姿勢を指していなかった**ため、teleop で leader pos をそのまま follower にコピーすると、follower 側の物理リミット越え → 破損。

### 解決

**手動でグリッパ単体の zero を焼き直す** Python スクリプトを書きました:

```python
arm.init_arm_motors(...)
arm.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18)
arm.set_callback_mode_all(oa.CallbackMode.STATE)
gripper = arm.get_gripper()

# ユーザーがグリッパを物理的に完全閉鎖位置で押さえている間に...
gripper.enable_all()
# 現在角を読み (記録用)
# disable してから set_zero_all (DM プロトコルの要件)
gripper.disable_all()
gripper.set_zero_all()
# 再 enable して書き込み後の値が ~0 になっているか確認
```

これを 4 アーム全部に対して実行 → 全グリッパが「完全閉鎖位置 = 0」で揃った状態に。

### 結果

teleop を再起動すると、**起動直後のグリッパ動作が intended な「軽く開く initial pose」止まり**に変わりました。リミット越えなし。

---

## 詰まったポイント 5: Bilateral teleop が数十秒で固まる（実は USB EMI 問題だった）

Unilateral は安定。しかし **bilateral に切り替えると数十秒〜数分で固まる**。Follower が反応せず、Leader が硬くなる。

最初は openarm_teleop のソフト側を疑って色々調べましたが原因不明。`dmesg` を見たら一発で判明:

```
peak_usb 1-5:1.0 can0: Rx urb aborted (-71)
peak_usb 1-5:1.0 can1: Rx urb aborted (-71)
usb usb1-port5: disabled by hub (EMI?), re-enabling...
usb 1-5: USB disconnect, device number 37
```

**カーネル自身が "EMI?" と疑問符付きでメッセージを出している**。USB ハブが「ノイズで信号読めない」と判断して、ポートを自動で切断していた。

### 原因

- Bilateral 制御は重力補償で **全 follower モータに常時電流**が流れる (Unilateral では大半 passive)
- モータ電源ケーブルに大電流が流れると周囲に磁界ができる
- 近接配線していた **PEAK アダプタの USB データケーブルにノイズが誘導**される
- USB プロトコルがエラーを検出 → ハブがポート切断 → CAN リンク DOWN

物理層の EMI 問題。Adapter A (USB ポート 1-5) の **2 チャネル (can0, can1) が同時に**落ちる。両 ch が物理的に同じ USB デバイス内にあるため、USB の問題で両方共倒れする。

### 解決

PEAK アダプタの USB ケーブルを **モータ電源ケーブルから物理的に離す**:

- 最低 **10cm 以上**の隙間
- **並走させない** (直角交差を推奨)
- **束ねない / ジップタイで motor cable に固定しない**

これだけで unilateral も bilateral も完全に安定するようになりました。

### 万一それでも再発する場合の追加対策

1. **USB ケーブルにクリップ式フェライトコア** (1個 100〜300円、両端に装着)
2. **セルフパワー USB ハブ** (金属筐体 シールド付きが望ましい)
3. ガルバニックアイソレーション or オプティカルアイソレーション付きの**産業用 CAN アダプタ**に交換

これは **OpenArm 公式 GitHub に未報告**の知見です（一番近い既知 issue は openarm_teleop#9 で、メンテナーが「CAN interface が原因かもしれない」と推測しているところまで）。

---

## 達成したこと

✅ Ubuntu 22.04 + ROS 2 humble に OpenArm 開発環境を構築
✅ 32 モータ全数の通信確認・零位較正完了
✅ Adapter ↔ Leader/Follower マッピング自動検出ツール
✅ グリッパ単独 re-zero ツール (公式較正の不備をカバー)
✅ Unilateral teleop (右腕) 動作確認
✅ **Bilateral teleop (右腕) 動作確認**
✅ USB EMI 問題の根本原因特定と物理対策

---

## 次にやること

- 左腕の grippers re-zero → 左腕 unilateral/bilateral 検証
- 両腕 bimanual 同時 teleop
- enactic/openarm_teleop に GitHub Issue 提出
  - `launch_bilateral.sh` の `~/openarm_teleop_tmp/build/bilateral_control` typo
  - PCAN-USB Pro FD での EMI 知見
- セットアップ済み知見を Claude Code カスタムエージェント化 (`.claude/agents/openarm-engineer.md`)

---

## まとめ — OpenArm セットアップで初心者が気をつけるべき5箇条

1. **conda は ROS のビルド前にちゃんと無効化する** — PATH から miniconda を除去するまで
2. **CAN バスとアームの対応は実機の動作で確認する** — 目視のケーブル追跡だけでは間違える
3. **OpenArm スクリプトを連続実行する前後は必ず明示 disable を入れる** — モータの stale state でハングする
4. **公式較正後、teleop を起動する前に必ずグリッパの zero を手動で確認/焼き直す** — 公式較正のグリッパ処理は不完全
5. **bilateral teleop で固まったら、まず `sudo dmesg` を見る** — ソフトでなく USB EMI が原因のことがある

「ハードウェアの物理的な要因まで疑う」というセンスは、シミュレーションで開発しているだけでは身につかない学びでした。

---

#OpenArm #ロボティクス #ヒューマノイドロボット #ROS2 #CAN #EMI #遠隔操作 #バイラテラル制御 #Damiao #PEAKSystem #PCAN #Ubuntu #DebugLog #Enactic #Orboh
