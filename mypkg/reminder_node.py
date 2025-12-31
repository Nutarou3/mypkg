#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2025 Gentoku Morimoto
# Licensed under the GPL-3.0-only.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from datetime import datetime

class ReminderNode(Node):
    """
    リマインダーを管理し、指定時刻にトピックで通知するノード。
    予定はJSONファイルに保存され、再起動しても保持されます。
    """
    def __init__(self):
        super().__init__('reminder_node')
        
        # 通知用パブリッシャー
        self.pub_alert = self.create_publisher(String, '/reminder_alert', 10)
        
        # 登録用サブスクライバー
        # 入力形式例: "2025-12-31 23:59:00,ハッピーニューイヤー"
        self.sub_add = self.create_subscription(
            String,
            '/add_reminder',
            self.add_callback,
            10
        )
        
        # 永続化用データの読み込み
        self.db_path = os.path.expanduser('~/.ros_reminders.json')
        self.reminders = self.load_data()
        
        # 1秒ごとに時刻をチェックするタイマー
        self.timer = self.create_timer(1.0, self.check_time)
        
        self.get_logger().info('Reminder Node has started. Waiting for reminders...')

    def load_data(self):
        """JSONファイルからデータを読み込む"""
        if os.path.exists(self.db_path):
            try:
                with open(self.db_path, 'r') as f:
                    return json.load(f)
            except Exception as e:
                self.get_logger().error(f'Failed to load data: {e}')
        return []

    def save_data(self):
        """データをJSONファイルに書き出す"""
        try:
            with open(self.db_path, 'w') as f:
                json.dump(self.reminders, f)
        except Exception as e:
            self.get_logger().error(f'Failed to save data: {e}')

    def add_callback(self, msg):
        """トピックを受け取ってリマインダーを登録する"""
        try:
            # カンマで日時と内容を分割
            parts = msg.data.split(',', 1)
            if len(parts) < 2:
                raise ValueError("Invalid format. Use 'YYYY-MM-DD HH:MM:SS,Message'")
            
            time_str = parts[0].strip()
            content = parts[1].strip()
            
            # 日付形式が正しいかチェック
            datetime.strptime(time_str, '%Y-%m-%d %H:%M:%S')
            
            self.reminders.append({'time': time_str, 'message': content})
            self.save_data()
            self.get_logger().info(f'Registered: [{content}] at {time_str}')
            
        except Exception as e:
            self.get_logger().error(f'Registration error: {e}')

    def check_time(self):
        """現在時刻と照らし合わせ、時間が来たら通知を送信する"""
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        new_list = []
        triggered = False
        
        for r in self.reminders:
            if now >= r['time']:
                # 通知用メッセージを作成
                msg = String()
                msg.data = f"【REMINDER】{r['message']} (Scheduled: {r['time']})"
                self.pub_alert.publish(msg)
                self.get_logger().info(f"Alert published: {r['message']}")
                triggered = True
            else:
                # まだ時間が来ていないものは残す
                new_list.append(r)
        
        # 変化があった場合のみデータを更新して保存
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
