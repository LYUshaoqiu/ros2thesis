#!/usr/bin/env bash

# ============= 配置区域，根据你的实际环境修改 =============

# # 1) ROS 发行版环境，这里以 "humble" 为例
# source /opt/ros/humble/setup.bash

# 2) 如果你有自己的 workspace，也要 source 它的 setup
#    source ~/my_ros2_ws/install/setup.bash
# ============= 进入项目目录 =============
cd /home/shaoqiu/ros2/ros2thesis-main/src/ros2_demo_project


# ============= 启动后台任务 =============

# 启动 ros2 launch
# 将输出重定向到 /dev/null，如果想保留日志可以重定向到文件
nohup ros2 launch ros2_demo_project system_demo_launch.py \
    > /dev/null 2>&1 &

# 启动 python3 api_server.py
nohup python3 /home/shaoqiu/ros2/ros2thesis-main/src/ros2_demo_project/src/api_server.py \
    > /dev/null 2>&1 &

# 启动 rviz2
nohup rviz2 > /dev/null 2>&1 &

# ============= 等待服务启动 =============
# 可选：睡眠几秒，确保 api_server 或 ros2 节点初始化完成
sleep 2

# ============= 打开浏览器访问前端 =============
xdg-open "http://0.0.0.0:8000"

echo "All processes have been started in the background."
echo "Open http://0.0.0.0:8000/gui/ in your browser if it didn't open automatically."
