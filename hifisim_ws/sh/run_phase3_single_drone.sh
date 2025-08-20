#!/usr/bin/env bash
set -euo pipefail
# Usage: ./sh/run_phase3_single_drone.sh [goal_x] [goal_y] [goal_z]
# Defaults: 5.0 5.0 5.0

GOAL_X="${1:-5.0}"
GOAL_Y="${2:-5.0}"
GOAL_Z="${3:-5.0}"

log(){ echo "[P3-1] $*"; }

# 0) Clean old processes (planner/adapter/relays)
log "Killing old processes ..."
pkill -f ego_planner_node || true
pkill -f controller_adapter.py || true
pkill -f pose_to_odom_adapter.py || true
pkill -f cloud_relay.py || true
sleep 0.3

# 1) Start Pose→Odom adapter
log "Start Pose→Odom adapter ..."
nohup python3 /home/ctx/hifisim_ws/pose_to_odom_adapter.py \
  --in /agent001/global/sim_nwu_pose \
  --out /drone_0_odom \
  --frame world --child base_link \
  >/tmp/odom_0.log 2>&1 & echo $!

# 2) Start Cloud relay (Unity → planner)
log "Start Cloud relay ..."
nohup python3 /home/ctx/hifisim_ws/cloud_relay.py \
  --in /agent001/lidar01 \
  --out /drone_0_cloud \
  >/tmp/cloud_0.log 2>&1 & echo $!

# 3) Start Ego-Planner (short topic names)
log "Start Ego-Planner with goal=(${GOAL_X}, ${GOAL_Y}, ${GOAL_Z}) ..."
nohup bash -lc "ros2 launch ego_planner advanced_param.launch.py \
  drone_id:=0 odometry_topic:=odom cloud_topic:=cloud \
  flight_type:=2 planning_horizon:=7.5 \
  point_num:=1 point0_x:=${GOAL_X} point0_y:=${GOAL_Y} point0_z:=${GOAL_Z} \
  map_size_x_:=120.0 map_size_y_:=120.0 map_size_z_:=40.0" \
  >/tmp/ego_planner_0.log 2>&1 & echo $!

# 4) Start controller_adapter (encode mission_code=3)
log "Start controller_adapter (mission_code=3) ..."
nohup python3 /home/ctx/hifisim_ws/controller_adapter.py \
  --agent_id 1 --hz 20 --horizon 4.0 --mission_code 3 \
  >/tmp/ctrl_1.log 2>&1 & echo $!

sleep 0.8

# 5) Health checks
log "Check topics ..."
ros2 topic info /drone_0_planning/bspline | cat || true
ros2 topic info /agent001/trajectory/points | cat || true

log "To TAKEOFF once (sec=1), run this line in another terminal:"
echo "ros2 topic pub -1 /agent001/trajectory/points trajectory_msgs/msg/JointTrajectory \"{header: {stamp: {sec: 0, nanosec: 0}}, joint_names: ['agent001'], points: [{positions: [0.0, 0.0, 0.0], time_from_start: {sec: 1, nanosec: 0}}]}\"" 