from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    管理ノード(reminder_node)と通知ノード(notifier_node)を
    一括で起動するためのローンチ設定
    """
    return LaunchDescription([
        # 1. 予定を管理・保存するメインノード
        Node(
            package='mypkg',
            executable='reminder_node',
            name='reminder_manager_node',
            output='screen'
        ),
        # 2. 指定時刻にデスクトップ通知を出すノード
        Node(
            package='mypkg',
            executable='notifier_node',
            name='reminder_notifier_node',
            output='screen'
        ),
    ])
