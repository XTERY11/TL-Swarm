#!/bin/bash
set -euo pipefail

# Usage: ./run_phase2_two_drones.sh
# Requires: source /opt/ros/humble/setup.bash && source /home/ctx/hifisim_ws/install/setup.bash

log() { echo "[Phase2] $*"; }

# 1) Clean old background processes
log "Killing old adapters/planners/feeders if any ..."
kill_patterns=("pose_to_odom_adapter.py" "cloud_relay.py" "odom_relay.py" "ego_planner_node" "waypoint_feeder.py" "waypoint_feeder_node.py" "signpost_router.py")
for pat in "${kill_patterns[@]}"; do
	pkill -f "$pat" || true
	sleep 0.1
done

# 2) Start adapters for agent001→drone_0 and agent002→drone_1
log "Starting adapters for agent001→drone_0 ..."
nohup python3 /home/ctx/hifisim_ws/pose_to_odom_adapter.py --in /agent001/global/sim_nwu_pose --out /drone_0_odom --frame map --child base_link >/tmp/pose_to_odom_0.log 2>&1 & echo $!
nohup python3 /home/ctx/hifisim_ws/cloud_relay.py --in /agent001/lidar01 --out /drone_0_cloud >/tmp/cloud_a1_to_d0.log 2>&1 & echo $!
nohup python3 /home/ctx/hifisim_ws/odom_relay.py --in /drone_0_odom --out /drone_0_/drone_0_odom >/tmp/odom_ns_0.log 2>&1 & echo $!
nohup python3 /home/ctx/hifisim_ws/cloud_relay.py --in /drone_0_cloud --out /drone_0_/drone_0_cloud >/tmp/cloud_ns_0.log 2>&1 & echo $!

log "Starting adapters for agent002→drone_1 ..."
nohup python3 /home/ctx/hifisim_ws/pose_to_odom_adapter.py --in /agent002/global/sim_nwu_pose --out /drone_1_odom --frame map --child base_link >/tmp/pose_to_odom_1.log 2>&1 & echo $!
nohup python3 /home/ctx/hifisim_ws/cloud_relay.py --in /agent002/lidar01 --out /drone_1_cloud >/tmp/cloud_a2_to_d1.log 2>&1 & echo $!
nohup python3 /home/ctx/hifisim_ws/odom_relay.py --in /drone_1_odom --out /drone_1_/drone_1_odom >/tmp/odom_ns_1.log 2>&1 & echo $!
nohup python3 /home/ctx/hifisim_ws/cloud_relay.py --in /drone_1_cloud --out /drone_1_/drone_1_cloud >/tmp/cloud_ns_1.log 2>&1 & echo $!

sleep 1

# 3) Quick health checks for odom and cloud (non-fatal)
log "Quick check: /drone_0_/drone_0_odom and /drone_1_/drone_1_odom (2s timeout)"
(timeout 2 ros2 topic echo /drone_0_/drone_0_odom --once >/dev/null 2>&1 && echo "  drone_0 odom OK") || echo "  drone_0 odom NO MSG"
(timeout 2 ros2 topic echo /drone_1_/drone_1_odom --once >/dev/null 2>&1 && echo "  drone_1 odom OK") || echo "  drone_1 odom NO MSG"

log "Quick check: /drone_0_/drone_0_cloud and /drone_1_/drone_1_cloud (2s timeout)"
(timeout 2 ros2 topic echo /drone_0_/drone_0_cloud --once >/dev/null 2>&1 && echo "  drone_0 cloud OK") || echo "  drone_0 cloud NO MSG"
(timeout 2 ros2 topic echo /drone_1_/drone_1_cloud --once >/dev/null 2>&1 && echo "  drone_1 cloud OK") || echo "  drone_1 cloud NO MSG"

# 4) Start signpost routers (PoseArray) and waypoint feeders (auto-advance by odom)
log "Starting signpost_router for agent001 and agent002 ..."
nohup python3 /home/ctx/hifisim_ws/signpost_router.py --agent_id 1 --testcase_id 1 --safe_z 1.8 --pre_dist 1.5 --post_dist 1.5 >/tmp/router_1.log 2>&1 & echo $!
nohup python3 /home/ctx/hifisim_ws/signpost_router.py --agent_id 2 --testcase_id 1 --safe_z 1.8 --pre_dist 1.5 --post_dist 1.5 >/tmp/router_2.log 2>&1 & echo $!

log "Starting waypoint_feeder_node (auto advance) for agent001 and agent002 ..."
nohup python3 /home/ctx/hifisim_ws/waypoint_feeder_node.py --agent_id 1 --reach_thresh 1.0 --resend 2.0 >/tmp/feeder_node_1.log 2>&1 & echo $!
nohup python3 /home/ctx/hifisim_ws/waypoint_feeder_node.py --agent_id 2 --reach_thresh 1.0 --resend 2.0 >/tmp/feeder_node_2.log 2>&1 & echo $!

# 5) Start planners in order: first drone_0, then drone_1
log "Launching ego_planner for drone_0 ..."
nohup ros2 launch ego_planner advanced_param.launch.py drone_id:=0 odometry_topic:=/drone_0_odom cloud_topic:=/drone_0_cloud odom_world:=/drone_0_odom >/tmp/ego_planner_0.log 2>&1 & echo $!
sleep 2

# Verify bspline from drone_0 (non-fatal)
log "Waiting bspline from drone_0 (2s timeout)"
(timeout 2 ros2 topic echo /drone_0_planning/bspline --once >/dev/null 2>&1 && echo "  drone_0 bspline OK") || echo "  drone_0 bspline NO MSG yet"

log "Launching ego_planner for drone_1 ..."
nohup ros2 launch ego_planner advanced_param.launch.py drone_id:=1 odometry_topic:=/drone_1_odom cloud_topic:=/drone_1_cloud odom_world:=/drone_1_odom >/tmp/ego_planner_1.log 2>&1 & echo $!

sleep 2

# Show key subscriptions of drone_1 to confirm sequential-start dependency
log "Subscriptions of /drone_1_ego_planner_node (should include /drone_0_planning/swarm_trajs)"
ros2 node info /drone_1_ego_planner_node | sed -n "/Subscribers:/,/Publishers:/p" || true

log "Now monitor:"
echo "  ros2 topic echo /drone_0_planning/bspline | head -n 5"
echo "  ros2 topic echo /drone_1_planning/bspline | head -n 5"
echo "Tips: 若出现 in obstacle，先确保起飞高度≈1.8m；必要时先暂停 /drone_N_/drone_N_cloud 再恢复。" 