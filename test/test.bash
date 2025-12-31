#!/bin/bash
# 
# Copyright (c) 2025 Gentoku Morimoto.
# Licensed under the GNU General Public License v3.0.

# 1. ROS 2の環境を読み込む（これで ros2 コマンドが使えるようになります）
source /opt/ros/humble/setup.bash

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws

# 2. ビルドとセットアップ
colcon build --packages-select mypkg
source install/setup.bash

# 3. リマインダーノードをバックグラウンドで起動
ros2 run mypkg reminder_node > /tmp/mypkg_test.log 2>&1 &
PID=$!

# ノードが立ち上がるのを待つ
sleep 5

# 4. テスト用の予定を登録（5秒後にセット）
TARGET_TIME=$(date -d "5 seconds" +"%Y-%m-%d %H:%M:%S")
ros2 topic pub /add_reminder std_msgs/msg/String "{data: '$TARGET_TIME,TEST_MSG'}" --once

# 登録処理とログ出力を待つ
sleep 5

# 5. ログを確認（"Registered" という文字があれば成功）
if grep -q "Registered" /tmp/mypkg_test.log; then
    echo "Test Passed"
    kill $PID
    exit 0
else
    echo "Test Failed: Confirmation log not found."
    cat /tmp/mypkg_test.log
    kill $PID
    exit 1
fi
