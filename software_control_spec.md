# ソフトウェア制御系統 設計仕様書 (Software & Control Specification)

都立産業技術高専 荒川キャンパス 高専ロボコン2026 全国大会出場チーム

---

## 1. ソフトウェアシステム概要

本機体は、ROS 2 Humble および Ubuntu 22.04 LTS (Jetson Orin) をベースとし、LiDAR点群処理・自己位置推定・経路計画・終端位置決め制御・画像追跡をモジュール化して実装している。

### 1.1 パッケージ構成と責務

| パッケージ名 | 主要ノード | 責務・役割 | 実装言語 / 技術 |
|---|---|---|---|
| **`gn10_navigation`** | `costmap_generator_node`<br>`path_planner_node`<br>`path_follower_node`<br>`target_manager_node` | フィールド障害物地図生成、8方向A*経路探索、Pure Pursuit + 終端P追従制御、目標座標管理 | C++20 / ROS 2 Action / Service |
| **`gn10_pointcloud_localization`** | `gn10_pointcloud_localization_node`<br>`localization_status_monitor_node` | 3D点群・IMUを用いた高精度自己位置推定、動的障害物・地面点群の分離、品質監視 | C++ / CUDA ESDF 3Dマッチング |
| **`lidar_perception_system`** | `moving_bucket_detector`<br>`track_freshness_guard_node` | 移動バケツの3D点群からの抽出、位置・速度・共分散推定、鮮度フィルタ | C++ / CUDA グリッドクラスタリング |
| **`opponent_tracker`** | `opponent_tracker_node` | 相手機体および相手バケツ位置・速度の動的点群追跡 | C++ / Eigen / 幾何クラスタリング |
| **`auto_mode_manager`** | `auto_mode_manager_node`<br>`navigation_safety_monitor.py` | 自律動作ステートマシン管理、通信・障害物・推定品質の統合安全監視 | Python 3 / ROS 2 Lifecycle風設計 |
| **`robot_port`** | `udp_node` | Jetson ROS 2 ↔ Mainboard STM32 間の高信頼リアルタイムUDP通信ブリッジ | C++ / POSIX Non-blocking Socket |
| **`realsense_dust_cloth`** | `cloth_tracker_cuda_node`<br>`trajectory_probability_analyzer.py`| RealSense RGB-Dによる射出布（雑巾）の高速追跡と着弾軌跡・確率解析 | C++ / CUDA / OpenCV / Python |
| **`gn10_shot_manager`** | `shooting_gui.py`<br>`trial_recorder.py`<br>`shooting_editor.py` | 試行データ記録、射出パラメータ微調整GUI、投擲座標GUIエディタ | Python 3 / PyQt / YAML / Git連携 |

---

## 2. 自己位置推定・点群処理アルゴリズム (`gn10_pointcloud_localization`)

1. **3D ESDF (Euclidean Signed Distance Field) マッチング**:
   * 事前に競技フィールドのCAD/PCDデータから3D ESDFマップを生成。
   * Livox MID-360Sから得られる点群を、ESDFの符号付き距離勾配を用いてGPU（CUDA）並列計算により高速フィッティング。
   * IMUの高周波角速度・加速度を相補フィルタ／カルマンフィルタで統合し、高速旋回・急加減速時にもドリフトのない `map -> base_link` TFを高周期（50Hz〜100Hz）で発行。
2. **点群分離パイプライン**:
   * フィールドの静的壁・机・ポールをESDF照合で判定。
   * 地面点群（`/ground_cloud`）、静的障害物（`/obstacle_cloud`）、および未知の動的物体（`/dynamic_cloud`）へリアルタイムに分離出力。

---

## 3. ナビゲーションおよび運動制御アルゴリズム (`gn10_navigation`)

### 3.1 コストマップ生成 (`costmap_generator_node`)
* フィールド定義JSON（`nhk2026_map.json`）および動的障害物点群から障害物セルを登録。
* 障害物セルを初期点とする **Multi-Source BFS（幅優先探索）** を適用し、計算量 $O(W \times H)$ で滑らかなインフレーション勾配を形成。
* コスト関数:
  $$C(d) = \begin{cases} 254 & (d \le r_{\text{robot}}) \\ C_{\text{base}} \times \left(1.0 - \dfrac{d - r_{\text{robot}}}{M}\right) & (r_{\text{robot}} < d \le r_{\text{robot}} + M) \\ 0 & (d > r_{\text{robot}} + M) \end{cases}$$

### 3.2 経路計画 (`path_planner_node`)
* 波及コストをペナルティ項に組み込んだ **8方向 A\* アルゴリズム** により、計算負荷を最小限に抑えつつ障害物からマージンを取った安全かつ最短格子経路を計算。

### 3.3 2段階ハイブリッド追従制御 (`path_follower_node`)
競技規則上の投擲精度を担保するため、追従アルゴリズムを距離に応じて切り替えるハイブリッド制御を実装。

1. **高速追従フェーズ (残距離 $d_{\text{goal}} \ge 0.15\,\text{m}$)**:
   * **Pure Pursuit制御**: 前方注視距離 $L_{\text{lookahead}} = 0.4\,\text{m}$ 先の目標点に向けた目標進行方向ベクトルを計算し、全方位オムニホイールの並進速度を出力。
2. **精密終端接近フェーズ ($0.03\,\text{m} < d_{\text{goal}} < 0.15\,\text{m}$)**:
   * **位置偏差フィードバック (P制御)**: 残距離ベクトルに比例した速度指令を与え、下限クランプ速度（$0.05\,\text{m/s}$）で確実にクリープ接近。
   * $$v = \max(0.05, d_{\text{goal}} \times K_p)$$
3. **完全停止・位置決め完了 ($d_{\text{goal}} \le 0.03\,\text{m}$)**:
   * 目標許容誤差 **3cm 以内** を満たした段階で並進・旋回速度をゼロにし、`NavigationStatus::Goal` を発行。

### 3.4 移動バケツ追従モード (Tracking モード)
* メイン基板から `TargetId::MovingBucket` が選択された場合、並進移動は手動操縦または停止とし、LiDAR点群から抽出された `map -> bucket_bottom` の相対方位角に対する **旋回P制御** のみをアクティブ化。
* バケツの動きに機体正面（射出機構）を常時正対させ、リアルタイム照準を達成する。

---

## 4. 安全監視・フェールセーフ系 (`navigation_safety_monitor`)

1. **点群衝突予測 (Staleness & Collision Guard)**:
   * 機体進行方向の直近エリアに予期せぬ障害物（審判、相手機体、脱落した布など）を検知した場合、即座に減速または停止指令を介入発行。
2. **自己位置推定品質ゲート (`localization_status_monitor`)**:
   * 点群マッチングの共分散行列および受信フレームレートを監視。信頼度が低下（`UNRELIABLE`）した場合は自動走行モードの移行を禁止し、手動操縦へ退避させる。
