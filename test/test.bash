#!/bin/bash
# SPDX-FileCopyrightText: 2025 Gentoku Morimoto
# SPDX-License-Identifier: GPL-3.0-only

dir=~
[ "$1" != "" ] && dir="$1"

# これが重要！ROS 2のコマンドを使えるようにする
source /opt/ros/humble/setup.bash

cd $dir/ros2_ws

# ビルドして環境を読み込む
colcon build --packages-select mypkg
source install/setup.bash

# --- テスト実行 ---

# 1. ノードをバックグラウンドで起動
ros2 run mypkg reminder_node > /tmp/mypkg_test.log 2>&1 &
PID=$!

# 起動待ち
sleep 5

# 2. テスト用の予定を登録
TARGET_TIME=$(date -d "5 seconds" +"%Y-%m-%d %H:%M:%S")
ros2 topic pub /add_reminder std_msgs/msg/String "{data: '$TARGET_TIME,TEST'}" --once

# 3. ログを確認
sleep 5
cat /tmp/mypkg_test.log | grep "Registered"
if [ $? -eq 0 ]; then
    echo "Test Passed"
    kill $PID
    exit 0
else
    echo "Test Failed"
    kill $PID
    exit 1
fi

