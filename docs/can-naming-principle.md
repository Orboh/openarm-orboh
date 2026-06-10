# OpenArm CAN 命名の原理 — なぜ leader/follower が逆転・流動的に崩れるのか

> この文書について: leader/follower 入れ替わり問題の根本原因と、channel_id+dev_id 方式で解決する原理の記録。PR `feat/robust-can-id` のレビュー用。

## 症状

配線（アダプタとアームの接続）を変えていないのに、**teleop 起動のたび・再起動・USB抜き差し・ポート変更・別PCで leader と follower が入れ替わる**。起動ごとに役割が違う＝「流動的」に崩れる。

## 根本原因の連鎖

### 1. カーネルの can0/can1/... は「USB認識順の先着順」で付く

SocketCAN のインターフェース名 `can0, can1, ...` は、カーネルが USB デバイスを認識した順に先着で割り当てる番号にすぎない。**物理アダプタにも物理アームにも固定で紐づかない。**

### 2. 認識順は非決定的＝起動ごとに変わりうる（"流動的"の正体）

2台のアダプタのどちらが先に enumerate されるかは、起動タイミング・USB初期化のレース・挿すポート・ハブ有無で変わる。だから「今日 can2 だったアダプタが次回 can0」になる。これが**再起動/抜き差しのたびに役割が揺れる**直接の理由。

### 3. 2台の PEAK は udev からは見分けがつかない

PCAN-USB Pro FD は工場出荷状態で **同じ USB vendor:product（`0c72:0011`）**、かつ **ユニークなシリアルを udev に出さない**。→ udev は「どちらのアダプタか」を識別できず、単純な「シリアル→名前」ルールが書けない。

### 4. 生の can 番号を決め打ちするスクリプトは必ず壊れる

「can2=leader / can0=follower」のように生名を固定で叩くと、§2 の採番揺れで簡単に逆転する。これが初期症状。

## なぜ「USBポートキー」方式でも別PC/ポートで崩れるか（第1の対策の限界）

採番揺れ対策として、udev で **USBポート（`KERNELS=="3-8:1.0"` / `"3-7:1.0"`）をキー**に固定名（can_lead_*/can_fol_*）を付けた。同じポートに挿し続ける限り再起動には強い。だが本質は「ポートに名前を縛った」だけなので:

- **ポートを入れ替えると逆転する**: 名前がポートに追従するため、アダプタ（とそれに繋がるアーム）が反対の役割名を受け取る。
  - 実測（本PR検証）: leader アダプタ(`44444444`)を 3-8→3-7 に挿し替えたら、ポートキー rule が `can_fol_*` と名付け、teleop で leader/follower が逆転した（下記「検証」）。
- **別PCでは通用しない**: USBポートの番号・トポロジはマザーボードごとに異なる（「3-8」が存在しない/別位置にある）。ルールがマッチせず名前が付かない、または誤って付く。

→ USBポートは「PC × 挿し位置」に依存する量。**再起動には耐えるが、ポート変更・別PCには耐えない**（＝可搬性がない）。

## 解決: アダプタと一緒に動く一意な識別子＝channel_id をキーにする

PEAK アダプタは NVRAM に **channel_id** を保持できる（`ethtool -E` で書き込み。後勝ち・再起動/別PCでも保持・いつでも変更可）。

- leader アダプタ=`44444444` / follower アダプタ=`22222222` を焼く（＝アダプタに押す「役割タグ」）。
- channel_id は**アダプタに内在して一緒に移動する**ので、どのポート・どのPCでも「`44444444` のアダプタ＝leader」。
- アダプタ内の2チャンネル(R/L)は **dev_id**（`0x0`/`0x1`、チャンネル内在）で区別。
- → `can_lead_r` ≡「channel_id `44444444` かつ dev_id `0x0` のチャンネル」と定義できる。**USBポート・PC・認識順に一切依存しない。**

補足: このモデルは channel_id を **per-device（アダプタ単位）**でしか保持しない（同一アダプタの両chが同値に collapse する）。よって R/L は channel_id では割れず、`dev_id` で割る。この「channel_id（アダプタ）× dev_id（チャンネル）」の組合せでちょうど4チャンネルが一意に決まる。

## 実装（固定名を付ける2経路・併用）

- **udev ルール**（`udev/80-openarm-can.rules`・channel_id+dev_id キー）: 挿すと自動で固定名。ただし **hotplug 直後は channel_id の sysfs 反映が間に合わず空振りする競合**がある（再起動はクリーン）。
- **`scripts/canup.sh`**（実行時リネーム）: インターフェース確定後に channel_id+dev_id を読み、固定名へリネーム→UP。**名前が完全にスワップしていても temp 名経由の2パスで衝突なく収束**させる。udev 競合の保険であり、teleop 起動ラッパが内部で呼ぶ。

## 検証（本PR・このPCで実施）

スワップ前ベースライン:
- `can_lead_r/l` = channel_id `44444444`, port `3-8`
- `can_fol_r/l` = channel_id `22222222`, port `3-7`

**(a) 旧ポートキー方式が壊れることの確認** — 2台のアダプタの USB ポートを入れ替え（44444444→3-7 / 22222222→3-8）、旧ルールのまま状態を読む:
```
can_lead_r ch=22222222 port=3-8   ← 本来 follower のアダプタが can_lead_ に
can_fol_r  ch=44444444 port=3-7   ← 本来 leader のアダプタが can_fol_ に
```
→ 名前がポートに追従し**逆転**。この状態で `10_launch_unilateral.sh right_arm can_lead_r can_fol_r` を起動すると、`can_lead_r`（=物理 follower）が leader 扱いになり、**teleop で leader/follower が逆転**することを実機確認（起動自体は両腕 `Arm motor count: 7` で正常、役割だけ逆）。

**(b) channel_id 方式で直ることの確認** — `sudo bash scripts/canup.sh` 実行後:
```
[up] can_lead_r channel_id=44444444 port=3-7   ← ポートが変わっても leader は 44444444 のまま
[up] can_fol_r  channel_id=22222222 port=3-8
```
→ ポート非依存で固定名が channel_id に追従（名前が完全スワップしていたが、canup.sh の temp名2パスで衝突なく訂正。`[stage]→[rename]` で can_fol_*↔can_lead_* を入替）。
- **unilateral 右**（`10_launch_unilateral.sh right_arm can_lead_r can_fol_r`）: leader/follower の逆転なし（OK）。
- **bimanual**（`16_launch_bimanual.sh`）: 右・左とも `Arm motor count: 7`、右 leader→右 follower / 左 leader→左 follower で追従、leader/follower・L/R とも逆転/取り違えなし（OK）。

→ **このPCでは、USBポートを入れ替えても channel_id 方式で leader/follower・L/R が崩れないことを実機確認**（unilateral＋bimanual, 2026-06-10）。残: 別PCでの確認（同梱の udev ルール/`canup.sh` をそのPCに入れて同手順）。
