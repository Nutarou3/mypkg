## mypkg
本リポジトリは、千葉工業大学のロボットシステム学の課題用パッケージです。ROS 2 を用いて、指定した時刻に予定を通知するリマインダーシステムを提供します。

## 実行例
課題の締め切り」や「ミーティング」などの予定を登録しておくと、時間になった瞬間にターミナル上への強調表示と、OSのデスクトップ通知（ポップアップ）で知らせてくれます。
```
# 1. システムの起動（別のターミナルで動作）
$ ros2 launch mypkg reminder_launch.py

# 2. 予定の登録（現在時刻の1分後などを指定）
$ ros2 topic pub /add_reminder std_msgs/msg/String "{data: '2025-12-31 21:00:00,課題の提出期限です'}" --once

# 3. 指定時刻の出力（通知ノード側のログ）
[notifier_node]: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[notifier_node]:            [REMINDER] 課題の提出期限です
[notifier_node]: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[notifier_node]: OS notification sent successfully.
```
## インストール方法
```
cd ~/ros2_ws/src
git clone [https://github.com/gentoku3/mypkg.git](https://github.com/gentoku3/mypkg.git)
cd ~/ros2_ws
colcon build --packages-select mypkg
source install/setup.bash
```
## 使い方
- 1.システムの一括起動:
・管理ノード（reminder_node）と通知ノード（notifier_node）を同時に起動します。
```
ros2 launch mypkg reminder_launch.py
```
- 2.リマインダーの登録
・標準的な ROS 2 のトピック送信コマンドを使用して予定を追加します。
```
ros2 topic pub /add_reminder std_msgs/msg/String "{data: 'YYYY-MM-DD HH:MM:SS,メッセージ'}" --once
```
## 動作環境
・OS: WSL (Ubuntu) または Linux 環境
・ROS 2: Humble Hawksbill
## 構成ノードと通信
・reminder_node: 予定のデータベース管理および時間監視（~/.ros_reminders.json に自動保存）。
・notifier_node: /reminder_alert を購読し、ターミナルとデスクトップ通知へ出力。
## 引用・参考資料
・ryuichiueda/my_slides (robosys_2022)
## ライセンス
Copyright (c) 2025 Gentoku Morimoto.
Licensed under the GNU General Public License v3.0 (GPL 3.0).
