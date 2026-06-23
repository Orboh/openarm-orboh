# OpenArm 左腕 IK（eye-in-hand 把持の粗接近）

オクラ把持の eye-in-hand パイプラインの **IK 粗接近** 部分。D435i でオクラの3D位置を取り、IK で腕を近傍へ運ぶ。そこから先（手首 D405 を見た微調整把持）は ACT が担う。

## 設計判断: なぜ lerobot で動かすか（DimOS との関係）

共有の DimOS リポ（`DimOS_base_G1-TOYOTA-BODY`）には既に次がある:

- cartesian IK 制御ループ一式（`cartesian_ik_task` + coordinator + `PinocchioIK`）
- OpenArm ハードウェア adapter（python-can で Damiao を直接駆動）
- **G1 用の okra ACT ブリッジ**（`act_bridge.py` + `scripts/act_service.py`・ZMQ 推論サービス・データセット `Orboh/okura-sub-lerobot`）

つまり「DimOS が IK 粗接近 → ACT 微調整」は G1 で実装済み。

ただし **DimOS の OpenArm adapter は lerobot と同じ1本の CAN バスを直接奪い合う**（同時起動不可）。そして我々の把持スタック（teleop・重力補償・較正・グリッパ・record・ACT 学習/推論）は全部 lerobot 上にある。

**OpenArm の de-risk**（研究課題: IK が生む開始姿勢のばらつきを ACT が吸収して掴めるか）には **lerobot 単独**が最短:

- IK の数学（pinocchio 減衰最小二乗・DimOS の `PinocchioIK` と同型）を**薄いループとして lerobot 駆動に載せる**だけ。
- teleop/record/ACT が1スタックに収まり、バス競合も無い。DimOS フレームワークは持ち込まない。

**DimOS+ACT は G1 production のアーキ**（DimOS が腕を所有し、ACT を ZMQ 推論サービスとして呼ぶ）。この de-risk が通ったら、DimOS の既存 okra-ACT ブリッジを雛形に G1 へ移植する。lerobot は ACT を作る工場（デモ収集＋学習＋推論サービス）として継続。

要するに **対立ではなく役割分担**: いまの OpenArm 検証は lerobot 単独、最終 G1 は DimOS+ACT。lerobot は技を作る側、DimOS は G1 で動かす側。物理込み把持シム（MuJoCo）は不要（ACT は実機デモ学習・把持本番は実機）と判断済み。

## ファイル

- `openarm_left_ik.py` — 再利用 IK（pinocchio reduced 左腕7-DOF・EE=TCP frame・position-only 粗接近・lerobot 度↔ラジアン橋渡し）
- `drive_left_ik.py` — 実機 `verify`（零点/符号チェック）+ `reach`（位置のみ IK 段階接近・`max_relative_target` clamp・dry-run 既定）。Thor 実行。
- `viz_meshcat.py` / `viz_skeleton.py` — 運動学3D（Meshcat・実物メッシュ）/棒人間（matplotlib）の可視化。実機不要・Vector。エピソード（始点→終点）再生対応。

## 実行環境

- IK の数学: pinocchio（Thor / Vector 両方の env に有）。
- 実機 reach / verify: **Thor**（CAN・lerobot・較正がある）。
- 可視化: **Vector**（meshcat・ディスプレイ・メッシュは `~/projects/OpenArm/src/openarm_description` を package_dirs で解決）。

## 状態 / 次

- オフライン IK 検証 合格（FK/IK 往復 0 fails・位置誤差サブmm）。
- 実機: 零点/符号 OK（joint_1/4・base系 +x前/+y左/+z上 確定）。reach 実到達は次回検証。
- 次: reach 実測 → オクラ検出（D435i）→ hand-eye 較正 → 収集（IK開始点→teleop→record）→ 学習 → ACT。
