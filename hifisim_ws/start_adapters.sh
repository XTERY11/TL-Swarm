#!/bin/bash
set -euo pipefail

# Usage:
#   ./start_adapters.sh N
# Example:
#   ./start_adapters.sh 2   # starts adapters for agent001..agent00N
# Mapping: agent%03d -> drone_(id-1)

if [ $# -lt 1 ]; then
	echo "Usage: $0 NUM_AGENTS" >&2
	exit 1
fi

NUM_AGENTS=$1

for ((i=1; i<=NUM_AGENTS; i++)); do
	agent_id=$(printf "%03d" "$i")
	drone_idx=$((i-1))
	pose_in="/agent${agent_id}/global/sim_nwu_pose"
	odom_out="/drone_${drone_idx}_odom"
	cloud_in="/agent${agent_id}/lidar01"
	cloud_out="/drone_${drone_idx}_cloud"
	echo "[Adapters] agent${agent_id}: ${pose_in} -> ${odom_out}; ${cloud_in} -> ${cloud_out}"
	nohup python3 /home/ctx/hifisim_ws/pose_to_odom_adapter.py --in "${pose_in}" --out "${odom_out}" --frame map --child base_link >/dev/null 2>&1 &
	nohup python3 /home/ctx/hifisim_ws/cloud_relay.py --in "${cloud_in}" --out "${cloud_out}" >/dev/null 2>&1 &
done

echo "All adapters started for 1..${NUM_AGENTS}" 