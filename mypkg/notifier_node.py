#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2025 Gentoku Morimoto
# Licensed under the GPL-3.0-only.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import shutil
import urllib.request
import json

class NotifierNode(Node):
    """
    リマインダーを「ターミナル」「OSデスクトップ」「Discord」の3箇所に通知するノード。
    LINE Notifyの終了に伴い、Discord Webhookに対応しています。
    """
    def __init__(self):
        super().__init__('notifier_node')
        
        # --- Discord通知の設定 (任意) ---
        # Discordのチャンネル設定から「ウェブフックURL」を取得してここに貼り付けてください
        # 空のままでも、ターミナル上での通知は機能します
        self.webhook_url = "https://discord.com/api/webhooks/1455895508865253466/3dP1FIzmu4fHCn4fMsEyooy1eNd18fRKpjwCQEQOrWYTaKL4R6L9PvjqVXGRSg0eCDip" 
        
        self.subscription = self.create_subscription(
            String,
            '/reminder_alert',
            self.listener_callback,
            10
        )
        self.get_logger().info('Notifier Node started. Waiting for alerts...')
        if not self.webhook_url:
            self.get_logger().info('Hint: Set webhook_url to receive notifications on Discord.')

    def listener_callback(self, msg):
        # 1. ターミナルへの強調表示 (最優先・確実)
        self.display_terminal_alert(msg.data)

        # 2. Discordへの通知 (URLが設定されている場合のみ)
        if self.webhook_url:
            self.send_discord_notification(msg.data)

        # 3. OSのデスクトップ通知 (WSL2等では失敗するため、エラーを無視)
        self.send_os_notification(msg.data)

    def display_terminal_alert(self, text):
        """ターミナルに目立つように枠線付きで表示する"""
        term_width = shutil.get_terminal_size().columns
        border = "!" * term_width
        print(f"\n{border}")
        print(f"  [REMINDER] {text}  ".center(term_width, " "))
        print(f"{border}\n")

    def send_discord_notification(self, message):
        """Discord Webhookを使用してメッセージを送信する"""
        payload = {
            "content": f"🔔 **リマインダー通知**\n{message}"
        }
        data = json.dumps(payload).encode("utf-8")
        
        req = urllib.request.Request(
            self.webhook_url, 
            data=data, 
            headers={"Content-Type": "application/json", "User-Agent": "ROS2-Notifier"}
        )
        
        try:
            with urllib.request.urlopen(req) as res:
                if res.getcode() == 204: # Discord Webhook 成功
                    self.get_logger().info('Discord notification sent successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to send Discord notification: {e}')

    def send_os_notification(self, message):
        """OSの通知コマンドを実行。失敗しても赤いログを出さない設定"""
        try:
            subprocess.run([
                'notify-send', 
                '【ROS 2 Reminder】', 
                message, 
                '--icon=appointment-soon'
            ], check=False, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = NotifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
