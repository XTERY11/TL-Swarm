#!/bin/bash

# HiFi Simulator 停止Demo脚本

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "stop HiFi Simulator Demo${NC}"

# 停止所有相关进程
# echo -e "${YELLOW}📋 停止ROS2进程...${NC}"
pkill -f "ros2 launch" 2>/dev/null
pkill -f "sim_gcs" 2>/dev/null
pkill -f "sim_flight" 2>/dev/null
pkill -f "performance" 2>/dev/null
pkill -f "radio_comm_simulator" 2>/dev/null
pkill -f "default_server_endpoint" 2>/dev/null
pkill -f "pbtm_ros2_node" 2>/dev/null

# 停止Unity进程
# echo -e "${YELLOW}📋 停止Unity进程...${NC}"
pkill -f "hifi_simulator_unity" 2>/dev/null

# 等待进程完全停止
sleep 1

# 检查是否还有残留进程
# echo -e "${YELLOW}📊 检查残留进程...${NC}"
if pgrep -f "ros2" > /dev/null; then
    # echo -e "${RED}⚠️  仍有ROS2进程运行，强制停止...${NC}"
    pkill -9 -f "ros2" 2>/dev/null
fi

if pgrep -f "hifi_simulator" > /dev/null; then
    # echo -e "${RED}⚠️  仍有HiFi Simulator进程运行，强制停止...${NC}"
    pkill -9 -f "hifi_simulator" 2>/dev/null
fi

echo -e "All closed ${NC}"
# echo -e "${BLUE}💡 提示：如果仍有问题，请重启终端${NC}" 