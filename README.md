## mypkg
本リポジトリは、千葉工業大学のロボットシステム学の課題用パッケージです。ROS 2 を用いて、指定した時刻にターミナル上とdiscord上に予定を通知するリマインダーシステムを提供します。

## 実行例
課題の締め切り」や「ミーティング」などの予定を登録しておくと、時間になった瞬間にターミナル上への強調表示と、OSのデスクトップ通知（ポップアップ）で知らせてくれます。
```
# 1. システムの起動（別のターミナルで動作）
$ ros2 launch mypkg reminder_launch.py
```
```
# 2. 予定の登録（現在時刻の1分後などを指定）
$ ros2 topic pub /add_reminder std_msgs/msg/String "{data: '2025-12-31 21:00:00,課題の提出期限です'}" --once
```
```
# 3. 指定時刻の出力（通知ノード側のログ）
[notifier_node]: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[notifier_node]:            [REMINDER] 課題の提出期限です
[notifier_node]: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[notifier_node]: OS notification sent successfully.
```
## 使い方
**1.システムの一括起動**

・管理ノード（reminder_node）と通知ノード（notifier_node）を同時に起動します。
```
ros2 launch mypkg reminder_launch.py
```
**2.リマインダーの登録** 

・標準的な ROS 2 のトピック送信コマンドを使用して予定を追加します。
```
ros2 topic pub /add_reminder std_msgs/msg/String "{data: 'YYYY-MM-DD HH:MM:SS,メッセージ'}" --once
```
## discordへの実装方法
**通知をdiscordへ送るには以下の手順で実行します**

- 1.新たなサーバーを作成する。または、既存のサーバーを開く
- 2.テキストチャンネルから"チャンネルの編集"を開く
- 3.連携サービスを開く
- 4."ウェブフック"を開き、"新しいウェブフック"を作成する
- 5.作成したものを開き"ウェブフックをコピー"をクリックする
- 6.コピーしたURLを"notifier_node.py"内の以下の場所にペーストする
```
26 self.webhook_url = ""
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
