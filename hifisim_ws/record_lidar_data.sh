#!/bin/bash
# 专门录制激光雷达数据的脚本

set -e

# 参数
RECORD_TIME=${1:-30}  # 录制时间(秒)
OUTPUT_DIR=${2:-"lidar_experiment_$(date +%Y%m%d_%H%M%S)"}

echo "开始录制激光雷达数据..."
echo "录制时间: ${RECORD_TIME}秒"
echo "输出目录: ${OUTPUT_DIR}"

# 等待核心节点就绪
bash ./check_core_nodes.sh

# 录制激光雷达相关话题
ros2 bag record \
    --include-services \
    --include-hidden-topics \
    -o "${OUTPUT_DIR}" \
    /agent002/lidar01 \
    /agent002/global/nwu_pose \
    /agent002/collision_crash \
    /simulation_setup_service \
    --duration ${RECORD_TIME}

echo "录制完成: ${OUTPUT_DIR}"