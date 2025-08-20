#!/usr/bin/env bash
set -euo pipefail
# Usage: ./sh/run_unity_endpoint.sh [ROS_PORT] [HOST_IP]
# Defaults: ROS_PORT=10000 HOST_IP=127.0.0.1

ROS_PORT="${1:-10000}"
HOST_IP="${2:-127.0.0.1}"

log(){ echo "[UE] $*"; }

# 0) Clean existing
log "Kill old Unity/Endpoint if any ..."
pkill -f default_server_endpoint || true
pkill -f hifi_simulator_unity\.x86_64 || true
sleep 0.5

# 1) Start Unity binary (client) with ROS_IP=host:port
log "Start Unity binary ..."
ROS_IP="${HOST_IP}:${ROS_PORT}" nohup "$HOME/Unity/build/hifi_simulator_unity.x86_64" \
  > /tmp/unity.log 2>&1 & echo $! | xargs -I{} bash -lc 'echo [UE] Unity PID: {}'

# 2) Start Endpoint (server)
log "Start ROS TCP Endpoint on 0.0.0.0:${ROS_PORT} ..."
nohup ros2 run tcp_endpoint_ros2 default_server_endpoint \
  --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=${ROS_PORT} \
  > /tmp/ros_tcp_endpoint.log 2>&1 & echo $! | xargs -I{} bash -lc 'echo [UE] Endpoint PID: {}'

# 3) Wait Unity connects and registers services
log "Waiting for /init_testcase service ... (do NOT start sim_gcs yet)"
until ros2 service list | grep -q "^/init_testcase$"; do sleep 0.5; done
ros2 service type /init_testcase | grep -q hifisim_msg_ros2/srv/InitTestCaseService || { log "init_testcase wrong type"; exit 1; }

log "Unity+Endpoint ready. Next step in a NEW terminal:"
echo "  ros2 run hifisim_task_ros2 sim_gcs   # 按提示输入 testcase_id 并回车发送 sim_started"
echo "验证: ros2 service list | grep /simulation_setup_service"
echo "日志: tail -f /tmp/ros_tcp_endpoint.log | tail -f /tmp/unity.log" 