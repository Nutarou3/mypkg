#!/bin/bash
# 
# Copyright (c) 2025 Gentoku Morimoto.
# Licensed under the GNU General Public License v3.0.

# 1. ROS 2のベース環境を読み込む（これがないとros2コマンドが使えません）
source /opt/ros/humble/setup.bash

# ワークスペースの場所を設定
dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws

# 2. ビルドと反映
colcon build --packages-select mypkg
source install/setup.bash

# 3. テストの実行（talker/listenerではなくreminderをテストします）
# バックグラウンドで起動し、ログを保存
ros2 run mypkg reminder_node > /tmp/mypkg_test.log 2>&1 &
PID=$!

# ノード起動待ち
sleep 5

# 予定を1つ登録してみる
TARGET_TIME=$(date -d "5 seconds" +"%Y-%m-%d %H:%M:%S")
ros2 topic pub /add_reminder std_msgs/msg/String "{data: '$TARGET_TIME,TEST'}" --once

# 反応を待つ
sleep 5

# 4. 判定：ログに「Reminder added（登録完了）」が出ているか確認
if grep -q "Reminder added" /tmp/mypkg_test.log; then
    echo "Test Passed"
    kill $PID
    exit 0
else
    echo "Test Failed: Confirmation log not found."
    cat /tmp/mypkg_test.log
    kill $PID
    exit 1
fi
