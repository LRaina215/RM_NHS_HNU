#!/bin/bash
sh reset.sh
# 指定ROS的日志文件夹并赋予权限，否则会运行失败
export ROS_LOG_DIR=/home/robomaster/shaobing/log
# 激活ros的环境
source /opt/ros/galactic/setup.bash
source /home/robomaster/shaobing/install/setup.bash
# # 设置串口权限
# sudo chmod 777 /dev/ttyACM0
# 设置 OpenCV 库路径
export LD_LIBRARY_PATH=/usr/local/opencv4.5.4/lib:$LD_LIBRARY_PATH

# 定义需要监控的节点和启动命令
declare -A NODES=(
    ["hik_camera"]="ros2 launch hik_camera hik_camera.launch.py"
    ["rm_description"]="ros2 launch rm_description model.launch.py"
    ["auto_aim"]="ros2 launch auto_aim_bringup auto_aim.launch.py"  # 合并 armor_detector 和 armor_solver
    ["sentry_up_serial"]="ros2 launch bubble_protocol sentry_up_serial_launch.py"
)

# ANSI 颜色代码
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # 重置颜色

# 全局标志文件
FLAG_FILE="/tmp/autostart_enabled"

# 创建标志文件
touch "$FLAG_FILE"

# 构建工作空间
echo -e "${GREEN}Building workspace with colcon build...${NC}"
cd /home/robomaster/shaobing
colcon build
if [ $? -ne 0 ]; then
    echo -e "${RED}colcon build failed! Exiting...${NC}"
    exit 1
fi
echo -e "${GREEN}Workspace build completed!${NC}"

# 启动 rqt 函数
#start_rqt() {
#    echo "Starting rqt..."
#    gnome-terminal -- bash -c "rqt; exec bash"
#}

# 直接启动 rqt
start_rqt

# 监控节点状态
while [ -f "$FLAG_FILE" ]; do
    for NODE_NAME in "${!NODES[@]}"; do
        # 检查节点是否启动
        PIDS=$(ps -ef | grep "${NODES[$NODE_NAME]}" | grep -v grep | awk '{print $2}')
        if [ "$PIDS" != "" ]; then
            echo -e "${GREEN}$NODE_NAME is running!${NC}"
        else
            # 如果节点没有启动则启动
            echo -e "${RED}Start $NODE_NAME ! ! ! ! !${NC}"
            # 在新的终端窗口中启动节点，并传递环境变量
            gnome-terminal -- bash -c "export LD_LIBRARY_PATH=/usr/local/opencv4.5.4/lib:$LD_LIBRARY_PATH; echo 'Running $NODE_NAME...'; ${NODES[$NODE_NAME]}; exec bash"
        fi
    done
    sleep 5
done

echo "Autostart script stopped."
