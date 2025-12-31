#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2025 Gentoku Morimoto.
# Licensed under the GNU General Public License v3.0.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from datetime import datetime

class ReminderNode(Node):
    """
    リマインダーを管理し、指定時刻に通知トピックを配信するノード。
    予定はホームディレクトリの隠しファイルに保存され、再起動しても保持されます。
    """
    def __init__(self):
        super().__init__('reminder_node')
        
        # 通知用パブリッシャー
        self.pub_alert = self.create_publisher(String, '/reminder_alert', 10)
        
        # 登録用サブスクライバー
        self.sub_add = self.create_subscription(
            String, 
            '/add_reminder', 
            self.add_callback, 
            10
        )
        
        # データの保存先設定
        self.db_path = os.path.expanduser('~/.ros_reminders.json')
        self.reminders = self.load_data()
        
        # 1秒ごとに時刻をチェックするタイマー
        self.timer = self.create_timer(1.0, self.check_time)
        
        self.get_logger().info('Reminder Node has started. Waiting for reminders...')

    def load_data(self):
        """ファイルから予定を読み込む"""
        if os.path.exists(self.db_path):
            try:
                with open(self.db_path, 'r') as f:
                    return json.load(f)
            except Exception:
                pass
        return []

    def save_data(self):
        """予定をファイルへ保存する"""
        with open(self.db_path, 'w') as f:
            json.dump(self.reminders, f)

    def add_callback(self, msg):
        """
        予定登録を受け取った時の処理
        入力形式: 'YYYY-MM-DD HH:MM:SS,メッセージ'
        """
        try:
            parts = msg.data.split(',', 1)
            if len(parts) < 2:
                raise ValueError("Format must be 'YYYY-MM-DD HH:MM:SS,Message'")
            
            time_str, content = parts[0].strip(), parts[1].strip()
            
            # 日付形式のチェック
            datetime.strptime(time_str, '%Y-%m-%d %H:%M:%S')
            
            self.reminders.append({'time': time_str, 'message': content})
            self.save_data()
            
            # テストスクリプトがこの「Registered:」という単語を探します
            self.get_logger().info(f'Registered: [{content}] at {time_str}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to add reminder: {e}')

    def check_time(self):
        """毎秒実行され、予定時刻になったものを配信する"""
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # まだ時間が来ていない予定と、時間が来た予定を分ける
        new_list = [r for r in self.reminders if now < r['time']]
        triggered = [r for r in self.reminders if now >= r['time']]
        
        for r in triggered:
            alert_msg = String()
            alert_msg.data = f"【REMINDER】{r['message']} (Scheduled: {r['time']})"
            self.pub_alert.publish(alert_msg)
            self.get_logger().info(f"Alert published: {r['message']}")
            
        # リストを更新して保存（通知済みを削除）
        if triggered:
            self.reminders = new_list
            self.save_data()

def main(args=None):
    rclpy.init(args=args)
    node = ReminderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

