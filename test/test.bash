#!/bicd ~/ros2_ws/src/mypkgn/bash
# SPDX-FileCopyrightText: 2025 Gentoku Morimoto
# SPDX-License-Identifier: GPL-3.0-only

dir=~
[ "$1" != "" ] && dir="$1"

# 1. ROS 2の環境を読み込む
source /opt/ros/humble/setup.bash

cd $dir/ros2_ws

# 2. ビルドとセットアップ
colcon build --packages-select mypkg
source install/setup.bash

# 3. リマインダーノードをバックグラウンドで起動
ros2 run mypkg reminder_node > /tmp/mypkg_test.log 2>&1 &
PID=$!

# ノード起動待ち
sleep 5

# 4. テスト用の予定を登録（5秒後）
TARGET_TIME=$(date -d "5 seconds" +"%Y-%m-%d %H:%M:%S")
# timeout コマンドでハングアップ防止（10秒で打ち切り）
timeout 10 ros2 topic pub /add_reminder std_msgs/msg/String "{data: '$TARGET_TIME,TEST_MSG'}" --once

# 5. ログの確認（最大20秒間待機して監視）
# タイミングによって失敗しないよう、ループで確認します
echo "Waiting for log..."
for i in {1..20}; do
    if grep -q "Registered" /tmp/mypkg_test.log; then
        echo "Test Passed: Found 'Registered' in logs."
        kill $PID
        exit 0
    fi
    sleep 1
done

# ループを抜けてしまったら失敗
echo "Test Failed: Confirmation log not found after waiting."
cat /tmp/mypkg_test.log
kill $PID
exit 1
