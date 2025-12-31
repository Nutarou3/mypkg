#!/bin/bash
# 
# Copyright (c) 2025 Gentoku Morimoto.
# Licensed under the GNU General Public License v3.0.

source /opt/ros/humble/setup.bash

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build --packages-select mypkg
source install/setup.bash

# 1. ノードを起動
ros2 run mypkg reminder_node > /tmp/mypkg_test.log 2>&1 &
PID=$!

sleep 5

# 2. テスト用の予定を登録
TARGET_TIME=$(date -d "5 seconds" +"%Y-%m-%d %H:%M:%S")
ros2 topic pub /add_reminder std_msgs/msg/String "{data: '$TARGET_TIME,TEST'}" --once

# 登録完了を待つ
sleep 10

# 3. 判定：ログに「Registered」が含まれているか確認
# 実際のログ出力 [Registered: [TEST] at ...] に合わせます
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
