# 1) 杀掉可能残留的所有关键进程
pkill -f "hifi_simulator_unity\.x86_64|default_server_endpoint|ego_planner_node|pbtm_ros2_node|controller_adapter\.py|pose_to_odom_adapter\.py|cloud_relay\.py|sim_gcs|performance|radio_comm_simulator" || true
sleep 0.5

# 2) 重启 ROS 2 daemon，清掉图缓存
ros2 daemon stop; sleep 1; ros2 daemon start

# 3) 清理 Fast-DDS 共享内存 (避免 RTPS/SHM 残留端口/资源)
find /dev/shm -maxdepth 1 -type s -name "fastrtps_*" -delete 2>/dev/null || true

# 4) 释放 10000 端口（若还在监听，说明 Endpoint 未杀干净）
ss -ltnp | grep 10000 || echo "port 10000 free"

# 5) 可选清理日志（方便新一轮排查）
rm -rf ~/.ros/log/* 2>/dev/null || true