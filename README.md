# 都立産業技術高専 荒川キャンパス 高専ロボコン2026 システム設計仕様書

本リポジトリおよび本ディレクトリ（`system-design`）は、高専ロボコン2026全国大会に出場する都立産業技術高専荒川キャンパスチームのロボットにおける、回路系統・通信系統・制御系統の統合設計仕様書およびアーキテクチャ図を管理するものです。

---

## 1. ドキュメント一覧

| ドキュメント | 内容 | 主な対象 |
|---|---|---|
| [回路・電源系統 設計仕様書](hardware_circuit_spec.md) | 駆動系(5S)/制御系(4S/3S/外部DC無停電)電源構成、EMS回路、モータ・ESC一覧、空圧系統 | 電源・回路・ハードウェア |
| [通信系統 設計仕様書](communication_spec.md) | Ethernet(L2SW), UDP通信プロトコル(Jetson↔Mainboard), 3系統CAN分離・絶縁アーキテクチャ | 通信・基板間インターフェース |
| [ソフトウェア制御系統 設計仕様書](software_control_spec.md) | ROS 2パッケージ構成、CUDA 3D自己位置推定、Multi-Source BFS、Pure Pursuit + 終端P制御 | 制御・ソフトウェア・認識 |
| [gn10-navigation 仕様書](gn10_navigation.md) | ナビゲーションノード詳細、コストマップ仕様、パラメータ、VSCode連携仕様 | 自律走行・経路追従 |
| [操縦キーコンフィグ](teleop_key_config.md) | 手動操縦時および手動テスト時のコントローラ・キーボード割り当て | 操縦・テスト |

---

## 2. UML・アーキテクチャ図一覧

### 2.1 回路・電源系統
* [電源系統図 (PlantUML)](wireing/power_wireing_diagram.pu) / [画像 (PNG)](wireing/PowerWiringDiagram_v5_Power.png)
* [CAN/信号系統図 (PlantUML)](wireing/logic_wireing_diagram.pu) / [画像 (PNG)](wireing/PowerWiringDiagram_v6_Signal.png)
* [電源・信号統合配線図 (PlantUML)](wireing/wireing_diagram.pu) / [画像 (PNG)](wireing/PowerWiringDiagram_v6_Combined.png)

### 2.2 通信系統
* [通信系統統合アーキテクチャ図 (PlantUML)](communication/communication_architecture.pu)

### 2.3 ソフトウェア・制御系統
* [ROS 2 ソフトウェア制御アーキテクチャ図 (PlantUML)](automation/ros2_system_architecture.pu)
* [ロボット統合システム状態遷移図 (PlantUML)](automation/system_state_machine.pu)
* [メイン基板・Jetson協調シーケンス図 (PlantUML)](automation/operation_sequence.pu)
* [ナビゲーションノード構成図 (PlantUML)](automation/navigation_node.pu) / [画像 (PNG)](automation/navigation_node.png)
* [ナビゲーション状態遷移図 (PlantUML)](automation/navigation_state.pu) / [画像 (PNG)](automation/navigation_state.png)

---

## 3. システム主要諸元

* **メイン計算機**: NVIDIA Jetson Orin (Ubuntu 22.04 LTS, ROS 2 Humble)
* **メイン制御基板**: STM32 マイコンボード (リアルタイムOS, 100Hz UDP通信)
* **主要センサ**:
  * 3D LiDAR: Livox MID-360S (点群・6軸IMU内蔵, Ethernet接続)
  * RGB-D カメラ: Intel RealSense D435/D455 (射出布追跡用, USB 3.0接続)
* **アクチュエータ構成**:
  * 足回り: 3輪オムニホイール (DJI M3508 + C620 ESC x3, CAN3絶縁バス)
  * 射出機構: 高速ベルト直動 (BLDC-NEO + VESC, CAN3絶縁バス)
  * 装填・アーム: DJI M2006 + C610 ESC x4, マブチ MS-385PH + HTMD v2.2c, MG996R サーボ
  * 空圧機構: 24V 電磁弁 3系統 (旗, 机1, 机2), 電空レギュレータ調圧
* **電源構成**:
  * 駆動系: LiPo 5S (18.5V〜21.0V) 30A ヒューズ保護 + パワーFET非常停止遮断
  * 制御系: LiPo 4S/3S または 外部AC-DC 19V 自動無停電給電 (LPMB)
