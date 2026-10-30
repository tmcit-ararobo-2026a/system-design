# 通信系統 設計仕様書 (Communication Specification)

都立産業技術高専 荒川キャンパス 高専ロボコン2026 全国大会出場チーム

---

## 1. 通信ネットワーク全体構成

本機体では、リアルタイム制御、大容量センサデータ伝送、および耐ノイズ性を両立するため、**Ethernet (1000BASE-T/100BASE-TX)** と **CAN (Controller Area Network)** を階層的に組み合わせた通信アーキテクチャを採用している。

```
[開発/遠隔PC]   [Livox MID-360]   [Mainboard (STM32)]
       \               |               /
        \              |              /
     +-----------------------------------+
     |    ギガビット L2 スイッチングハブ   |
     +-----------------------------------+
                       | Ethernet (UDP/IP)
              [Jetson Orin (ROS 2)]
                       |
     +-----------------+-----------------+
     | CAN1 (周辺機器) | CAN2 (電源/EMS) | CAN3 (ESCモータ: 絶縁)
     | (1Mbps)         | (500kbps)       | (1Mbps)
     v                 v                 v
   Trinity24V        VConv / PMB       C620 Hub (足回りx3)
   HTMD (昇降)       RemoteEMS         C610 Hub (装填・アームx4)
   KNI-Servo/LED                       VESC Hub (射出ベルト)
```

---

## 2. Ethernet / UDP 通信仕様 (Jetson Orin ↔ Mainboard)

Jetson Orin上のROS 2環境とSTM32メインマイコン間は、産業用Ethernetスイッチを介した高信頼・低遅延UDP/IP通信（パッケージ: `robot-port`）により直接接続される。

### 2.1 ネットワークパラメータ
* **Jetson IPアドレス**: `192.168.1.100` (Subnet: `255.255.255.0`)
* **Mainboard IPアドレス**: `192.168.1.10`
* **LiDAR (MID-360S) IP**: `192.168.1.1xx` (デフォルトLivox IP)
* **送信ポート / 受信ポート**: デフォルト `5000` / `5001`
* **通信レート**: 100 Hz (10 ms周期 確定通信)

### 2.2 プロトコルおよびメッセージ構造

#### (1) 送信メッセージ (Jetson → Mainboard)
* **トピック名**: `/robot/operation/cmd_vel` (`geometry_msgs/msg/Twist`)
  * `linear.x`: 機体前後速度指令 [m/s]
  * `linear.y`: 機体左右速度指令 [m/s]
  * `angular.z`: 機体旋回角速度指令 [rad/s]
* **トピック名**: `/robot/operation` (`robot_port/msg/RobotOperation`)
  * `navigation_status`: ナビゲーション稼働状態 (`Standby`, `Ready`, `Moving`, `Tracking`, `Goal`, `Fail`)
  * `target_id`: 現在の照準・移動目標ID
  * `target_roll_angle`: 射出角補正指令 [rad]
  * `belt_target_speed`: 射出ベルト直動 目標回転数 [rpm]
  * `air_launcher_select`: 動作させる空圧電磁弁番号 (0: None, 1: 旗, 2: 机1, 3: 机2)

#### (2) 受信メッセージ (Mainboard → Jetson)
* **トピック名**: `/robot/feedback` (`robot_port/msg/RobotFeedback`)
  * 機体オドメトリ情報 (各輪エンコーダ値, 推定速度)
  * バッテリ電圧 (5S駆動系, 4S/3S制御系)
  * 機構状態 (装填位置スイッチ, アームリミット, 電磁弁状態)
* **トピック名**: `/robot/communication_status` (`robot_port/msg/BridgeHealth`)
  * 送受信パケットカウンタ、パケットロス率、RTT遅延時間
  * タイムアウト検出フラグ (50ms以上受信途絶で安全停止トリガ)

---

## 3. CAN バス系統分離アーキテクチャ

単一のCANバスへの負荷集中やノイズ回り込みを防ぐため、物理的に独立した3系統のCANネットワークを構築している。

### 3.1 各系統の役割と通信レート

| 系統名 | 通信レート | 接続ノード | 主な伝送データ | アイソレーション |
|---|---|---|---|---|
| **CAN1 (周辺・補機)** | 1 Mbps | Mainboard, Trinity 24V, HTMD v2.2c, KNI-Servo, KNI-Servo-LED | 電磁弁開閉指令、電空レギュレータ調圧値、昇降DCモータ指令、サーボ角度、LED表示パターン | 非絶縁 (制御系GND共通) |
| **CAN2 (電源・安全)** | 500 kbps | Mainboard, Jetson Orin (CAN2), VConv, PMB, RemoteEMSBoard | 各部電圧・電流モニタ、FET遮断制御、非常停止状態通知、遠隔コマンド | 非絶縁 (ロジックGND共通) |
| **CAN3 (ESCモータ)** | 1 Mbps | Mainboard, C620 Hub, C610 Hub, VESC Hub, 各種モータドライバ | 各輪トルク/電流指令、アーム位置指令、射出回転数指令、ESCエンコーダ・温度フィードバック | **高速フォトカプラ完全絶縁** |

### 3.2 CAN3 絶縁ハブの構造
* Mainboard側のCANコントローラ出力は、ハブ基板上で高速デジタルアイソレータおよび絶縁型DC-DCコンバータにより電気的に絶縁される。
* これにより、BLDCモータやM3508等の加減速時に発生する急峻なグラウンドプレーンの電位変動（スイッチングノイズ）を完全に遮断し、Mainboardマイコンのハングアップや通信エラーフレーム発生を皆無にしている。

---

## 4. 安全通信・フェールセーフ機構

1. **UDP ハートビート監視**:
   Jetson側 `udp_node` および Mainboard 側リアルタイムOSタスクの双方がパケット受信カウンタを常時監視。100ms（10周期）以上の通信途絶を検知した場合、Mainboard側で全モータへの出力を自動的にゼロに落とすフェールセーフ（Watchdog停止）が即座に作動する。
2. **ハードウェア EMS 割り込み**:
   EMSスイッチ押下時はソフトウェア通信を介さず、PMB基板上のハードウェアロジックにより直接FET遮断が行われるため、マイコンやOSの暴走時にも確実にモータ動力をカットできる。
