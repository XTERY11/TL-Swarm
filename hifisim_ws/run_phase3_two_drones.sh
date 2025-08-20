#!/bin/bash
set -euo pipefail
# Usage: ./run_phase3_two_drones.sh

log() { echo "[Phase3] $*"; }

log "Killing old controller adapters if any ..."
pkill -f controller_adapter.py || true
sleep 0.2

log "Starting controller adapters (agent001, agent002) ..."
nohup python3 /home/ctx/hifisim_ws/controller_adapter.py --agent_id 1 --hz 10 --horizon 2.0 >/tmp/ctrl_1.log 2>&1 & echo $!
nohup python3 /home/ctx/hifisim_ws/controller_adapter.py --agent_id 2 --hz 10 --horizon 2.0 >/tmp/ctrl_2.log 2>&1 & echo $!

sleep 1
log "Check Unity control topics (JointTrajectory) ..."
ros2 topic info /agent001/trajectory/points | cat || true
ros2 topic info /agent002/trajectory/points | cat || true

echo "Tail logs: tail -n 30 /tmp/ctrl_1.log; tail -n 30 /tmp/ctrl_2.log" 