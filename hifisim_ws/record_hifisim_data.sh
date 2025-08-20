# 文件名：record_hifisim_data.sh
#!/usr/bin/env bash
set -e

AGENT_NUM=${1:-2}                                      # 默认 2 架 UAV，可通过第 1 参数修改
OUT_DIR=${2:-$(date +%Y%m%d_%H%M%S)_hifisim_bag}       # 保存目录

# 1) 等待核心节点上线（可复用上面的脚本）
bash ./check_core_nodes.sh

#############################################
# 2) 动态扫描并拼装录制列表                #
#############################################

# 起始固定服务（Topics 仍需至少一个占位）
TOPICS=("/simulation_setup_service")

# 收集当前 ROS graph 中所有 topic 列表一次，避免循环内多次调用
ALL_TOPICS=$(ros2 topic list)

for i in $(seq -f "%03g" 1 $AGENT_NUM); do
  # ① UAV 关键话题
  TOPICS+=("/agent${i}/global/nwu_pose")
  TOPICS+=("/agent${i}/collision_crash")
  TOPICS+=("/agent${i}/radio_broadcast")

  # ② 相机：匹配 /camera 或 /camera01 前缀的所有 Image 话题
  while read -r cam_topic; do
    [[ -n "$cam_topic" ]] && TOPICS+=("$cam_topic")
  done < <(echo "$ALL_TOPICS" | grep "/agent${i}/camera")

  # ③ 点云：匹配包含 /points 关键字的 PointCloud2 话题
  while read -r pcl_topic; do
    [[ -n "$pcl_topic" ]] && TOPICS+=("$pcl_topic")
  done < <(echo "$ALL_TOPICS" | grep "/agent${i}.*points")
done

# 去重（bash >=4.0）
read -r -a TOPICS <<< "$(printf '%s\n' "${TOPICS[@]}" | awk '!seen[$0]++')"

echo "即将录制以下 topics（含相机 / 点云 / 广播等）："
printf '  %s\n' "${TOPICS[@]}"

#############################################
# 3) 开始录包（新增 --include-services）   #
#############################################

ros2 bag record \
  --include-services \
  --include-hidden-topics \
  -o "$OUT_DIR" \
  "${TOPICS[@]}"