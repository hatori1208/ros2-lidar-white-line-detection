# ROS 2 LiDAR White Line Detection

LiDARの反射強度とDBSCANクラスタリングを用いて、道路上の白線候補を検出するROS 2用Pythonノードです。点群を一定時間蓄積し、クラスタの点数とXY方向の広がりを使って、実線・破線に分類します。

> A ROS 2 Python node for detecting road white lines from LiDAR intensity using DBSCAN clustering. Detected clusters are classified into solid and dashed line candidates.

## 主な機能

- 地面点群（`PointCloud2`）とオドメトリ（`Odometry`）を受信
- 点群を`odom`座標系に変換し、直近3秒分を蓄積
- 反射強度で白線候補を抽出し、XY平面上でDBSCANクラスタリング
- クラスタの点数と広がりで実線・破線候補を分類
- 判定結果を`PointCloud2`として配信。統合結果の`white_lines`は2秒間蓄積

## 処理の流れ

```text
/pcd_segment_ground (地面点群)    /odom/wheel_imu (自己位置・姿勢)
             │                         │
             └───────────┬─────────────┘
                         ▼
            odom座標系への変換・3秒蓄積
                         ▼
              反射強度 > 40 の点を抽出
                         ▼
             DBSCAN (XY座標でクラスタ化)
                         ▼
               ノイズ・小クラスタ除外
                         ▼
               実線・破線候補に分類
                 ┌───────┴───────┐
                 ▼               ▼
       /white_line_solid  /white_line_dashed
                 └───────┬───────┘
                         ▼
             /white_lines (2秒蓄積)
```

## 入出力トピック

### 入力

| トピック | メッセージ型 | 用途 |
| --- | --- | --- |
| `/pcd_segment_ground` | `sensor_msgs/msg/PointCloud2` | 地面点群。`x`, `y`, `z`, `intensity`を使用 |
| `/odom/wheel_imu` | `nav_msgs/msg/Odometry` | 座標変換用の位置とヨー角。`odom`パラメータで変更可能 |

### 出力

| トピック | メッセージ型 | 用途 |
| --- | --- | --- |
| `/white_lines` | `sensor_msgs/msg/PointCloud2` | 実線・破線候補を統合し、直近2秒分を保持した点群 |
| `/white_line_solid` | `sensor_msgs/msg/PointCloud2` | 実線と判定された点群 |
| `/white_line_dashed` | `sensor_msgs/msg/PointCloud2` | 破線と判定された点群 |

出力点群の`frame_id`は`odom`です。上記は名前空間を付けずに起動した場合のトピック名です。

## 主な設定値

| 項目 | コード中の変数 | 設定値 | 意味 |
| --- | --- | --- | --- |
| 地面点群の蓄積時間 | `duration` | `3.0` 秒 | 直近の点群を保持する期間 |
| 白線結果の蓄積時間 | `white_result_duration` | `2.0` 秒 | `/white_lines`で結果を保持する期間 |
| 反射強度しきい値 | `intensity_threshold` | `40.0` | `intensity > 40`の点を抽出 |
| DBSCANの近傍半径 | `dbscan_eps` | `0.30` m | XY平面上の距離 |
| DBSCANの最小サンプル数 | `dbscan_min_samples` | `6` | クラスタリングの密度条件 |
| クラスタ候補の最小点数設定 | `cluster_size_threshold` | `25` 点 | `25`点未満のクラスタは除外 |
| 座標の丸め間隔 | `ground_pixel` | `20`（=5 cm刻み） | 位置座標を0.05 m単位に丸める |
| 点群の保持範囲 | `MAP_LIM_X/Y_MIN/MAX` | 現在位置からXY各±30 m | 過去の蓄積点群を範囲で制限 |

### 実線・破線の判定条件

各クラスタについて、点数を`N`、X/Y方向の標準偏差を`spread_x`・`spread_y`とすると、次の順番で判定します。

1. `N < 25` のクラスタを除外
2. **実線候補**：`N > 180` かつ (`spread_x > 0.5` または `spread_y > 0.5`)
3. **破線候補**：実線条件に該当せず、`N > 30`
4. それ以外は出力しない

※ これらは現行実装のヒューリスティックな判定条件です。必ずしも白線だけを検出できることや、実線・破線を常に正しく区別できることを保証しません。

## 動作環境・依存ライブラリ

- ROS 2
- Python 3
- ROS 2 Pythonパッケージ：`rclpy`, `sensor_msgs`, `nav_msgs`, `std_msgs`
- Pythonライブラリ：`numpy`, `scikit-learn`

このノード単体では、LiDARの生点群から地面を抽出しません。別ノードで生成された`/pcd_segment_ground`とオドメトリが必要です。

## 実行方法

以下はPythonファイルを直接実行する例です（ROS 2と依存ライブラリのインストールが済んでいることが前提）。

```bash
# 使用するROS 2環境を読み込み（<distro>を使用中の名前に変更）
source /opt/ros/<distro>/setup.bash

# ROS 2ワークスペースを使っている場合
source ~/ros2_ws/install/setup.bash

# このリポジトリ内で実行
python3 reflection_to_pcd_dbscan_white_only.py
```

オドメトリのトピックを変更する場合：

```bash
python3 reflection_to_pcd_dbscan_white_only.py --ros-args -p odom:=/your_odom_topic
```

ROS 2パッケージとして`ros2 run`で起動する場合は、別途`setup.py`等に実行エントリーポイントの登録が必要です。

## RViz2での確認

1. 点群とオドメトリを配信するノード（またはrosbag）を起動する
2. 本ノードを起動する
3. RViz2の**Fixed Frame**を`odom`に設定する
4. **Add → PointCloud2**から`/white_lines`、`/white_line_solid`、`/white_line_dashed`を表示する

実際の認識結果のスクリーンショットを`docs/`などに追加すると、検出結果が伝わりやすくなります。

## 注意点・制約

- 入力`PointCloud2`は、各点の先頭16バイトに`x`, `y`, `z`, `intensity`が**float32で順番に配置されている**形式を想定しています。異なるフィールド配置では読み取り処理の変更が必要です。
- オドメトリからは位置とヨー角のみ使用し、ロール・ピッチは座標変換に反映しません。
- 反射強度はLiDARの機種や計測条件に依存します。しきい値の調整が必要な場合があります。
- 現行実装は候補点や有効クラスタが0点になると早期に処理を終了し、空の点群をPublishしません。下流ノードでは出力が更新されないケースに注意してください。
- クラスタの点数や広がりによる分類は、密度や走行状況によって変わる可能性があります。

