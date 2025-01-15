#!/bin/bash

# 指定ROS的日志文件夹并赋予权限，否则会运行失败
#export ROS_LOG_DIR=/home/robomaster/zxzzb/log
# 激活ros的环境
source /opt/ros/galactic/setup.bash
source /home/robomaster/zxzzb/install/setup.bash
# # 设置串口权限
# sudo chmod 777 /dev/ttyACM0

# 设置 OpenCV 库路径
export LD_LIBRARY_PATH=/usr/local/opencv4.5.4/lib:$LD_LIBRARY_PATH

# 定义需要监控的节点和启动命令
declare -A NODES=(
    ["hik_camera"]="ros2 launch hik_camera hik_camera.launch.py"
    ["rm_description"]="ros2 launch rm_description model.launch.py"
    ["armor_detector"]="ros2 run armor_detector armor_detector_node"
    ["armor_solver"]="ros2 run armor_solver armor_solver_node"
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

# 定义临时文件路径
LOG_DIR="/tmp/ros_nodes_logs"
mkdir -p "$LOG_DIR"

# 定义监听特定warning的函数
function monitor_specific_warning() {
    while [ -f "$FLAG_FILE" ]; do
        for NODE_NAME in "${!NODES[@]}"; do
            LOG_FILE="$LOG_DIR/${NODE_NAME}_output.log"
            if [ -f "$LOG_FILE" ]; then
                # 检查日志文件中是否包含特定的warning文本
                if grep -qi "Red-blue message is None. No message published." "$LOG_FILE"; then
                    echo -e "${RED}Specific warning detected in $NODE_NAME! Terminating the terminal...${NC}"
                    
                    # 查找与该节点相关的终端PID
                    TERMINAL_PID=$(ps -ef | grep "${NODES[$NODE_NAME]}" | grep -v grep | awk '{print $2}')
                    if [ "$TERMINAL_PID" != "" ]; then
                        # 强制终止终端
                        kill -9 "$TERMINAL_PID"
                        echo -e "${RED}Terminal for $NODE_NAME has been terminated.${NC}"
                    fi
                    
                    # 清空日志文件，避免重复检测
                    > "$LOG_FILE"
                fi
            fi
        done
        sleep 5 # 每隔5秒检查一次
    done
}

# 启动特定warning监听
monitor_specific_warning &

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
            
            # 定义日志文件路径
            LOG_FILE="$LOG_DIR/${NODE_NAME}_output.log"
            
            # 在新的终端窗口中启动节点，并重定向输出到日志文件
            gnome-terminal -- bash -c "export LD_LIBRARY_PATH=/usr/local/opencv4.5.4/lib:$LD_LIBRARY_PATH; echo 'Running $NODE_NAME...'; ${NODES[$NODE_NAME]} > $LOG_FILE 2>&1; exec bash" &
        fi
    done
    sleep 5
done

echo "Autostart script stopped."
