#!/bin/bash

# ベースディレクトリを設定
# GitHub Actionsでは通常 /root になる
dir=~
[ "$1" != "" ] && dir="$1"
cd $dir/ros2_ws
colcon build
source install/setup.bash
timeout 10 ros2 launch mypkg talk_listen.launch.py > /tmp/mypkg.log

cat /tmp/mypkg.log |
grep 'Liten: 10'
